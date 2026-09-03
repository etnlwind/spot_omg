/*
 * ESP32-WROOM-32D Bluetooth SPP <-> STM32 UART Bridge
 *
 * Wiring:
 *   ESP32 GPIO17 (TX) -> STM32 PC11 (USART3_RX)
 *   ESP32 GPIO16 (RX) <- STM32 PC10 (USART3_TX)
 *   ESP32 GND         -> STM32 GND
 *
 * USB serial is used only for firmware upload and diagnostics.
 */

#include <BluetoothSerial.h>
#include <Update.h>
#include <esp_gap_bt_api.h>
#include <esp_idf_version.h>
#include <esp_system.h>

HardwareSerial STM32Serial(1);
BluetoothSerial SerialBT;

static constexpr const char* BLUETOOTH_DEVICE_NAME = "SpotOMG-Bridge";
static constexpr uint32_t STM32_BAUD = 115200;
static constexpr int STM32_RX = 16;  // GPIO16 <- STM32 PC10 TX
static constexpr int STM32_TX = 17;  // GPIO17 -> STM32 PC11 RX
static constexpr int PAIRING_RESET_BUTTON = 0;  // BOOT button
static constexpr uint32_t PAIRING_RESET_HOLD_MS = 3000;
static constexpr uint32_t DISCOVERABLE_REFRESH_MS = 5000;
static constexpr size_t STM32_UART_RX_BUFFER_SIZE = 4096;
static constexpr size_t BRIDGE_CHUNK_SIZE = 256;
// Arduino-ESP32 BluetoothSerial has a fixed 512-byte RX queue. Keep only one
// small block in flight and acknowledge it before the host sends another.
static constexpr size_t OTA_ACK_INTERVAL = 256;
static constexpr size_t OTA_COMMAND_SIZE = 96;
static constexpr uint32_t OTA_RECEIVE_TIMEOUT_MS = 30000;

static uint32_t pairingButtonPressedAt = 0;
static uint32_t lastDiscoverableRefresh = 0;
static bool pairingResetHandled = false;
static char otaCommand[OTA_COMMAND_SIZE];
static size_t otaCommandLength = 0;
static bool otaCommandActive = false;

static void sendOtaError(const char* reason)
{
  SerialBT.printf("$SPOTOTA ERROR %s\n", reason);
  Serial.printf("Bluetooth OTA failed: %s\n", reason);
}

static bool receiveBluetoothUpdate(size_t imageSize, const char* expectedMd5)
{
  if (imageSize == 0 || imageSize > ESP.getFreeSketchSpace()) {
    sendOtaError("invalid-size");
    return false;
  }
  if (strlen(expectedMd5) != 32) {
    sendOtaError("invalid-md5");
    return false;
  }
  for (size_t i = 0; i < 32; ++i) {
    if (!isxdigit(static_cast<unsigned char>(expectedMd5[i]))) {
      sendOtaError("invalid-md5");
      return false;
    }
  }

  if (!Update.begin(imageSize, U_FLASH) || !Update.setMD5(expectedMd5)) {
    sendOtaError("begin-failed");
    Update.abort();
    return false;
  }

  Serial.printf("Bluetooth OTA receiving %u bytes\n",
                static_cast<unsigned>(imageSize));
  SerialBT.println("$SPOTOTA READY");
  uint8_t buffer[BRIDGE_CHUNK_SIZE];
  size_t received = 0;
  size_t nextAcknowledgement = min(OTA_ACK_INTERVAL, imageSize);
  uint32_t lastDataAt = millis();
  while (received < imageSize) {
    if (!SerialBT.hasClient()) {
      sendOtaError("disconnected");
      Update.abort();
      return false;
    }
    size_t count = 0;
    while (count < sizeof(buffer) && received + count < imageSize &&
           SerialBT.available() > 0) {
      const int value = SerialBT.read();
      if (value >= 0) {
        buffer[count++] = static_cast<uint8_t>(value);
      }
    }
    if (count > 0) {
      if (Update.write(buffer, count) != count) {
        sendOtaError("write-failed");
        Update.abort();
        return false;
      }
      received += count;
      lastDataAt = millis();
      if (received >= nextAcknowledgement) {
        SerialBT.printf("$SPOTOTA ACK %u\n", static_cast<unsigned>(received));
        nextAcknowledgement = min(received + OTA_ACK_INTERVAL, imageSize);
      }
    } else {
      if (millis() - lastDataAt >= OTA_RECEIVE_TIMEOUT_MS) {
        sendOtaError("timeout");
        Update.abort();
        return false;
      }
      delay(1);
    }
  }

  if (!Update.end(true)) {
    sendOtaError("verification-failed");
    return false;
  }
  SerialBT.println("$SPOTOTA OK");
  Serial.println("Bluetooth OTA complete; restarting");
  delay(500);
  ESP.restart();
  return true;
}

static void handleOtaCommand()
{
  otaCommand[otaCommandLength] = '\0';
  unsigned long imageSize = 0;
  char md5[33] = {};
  char extra = '\0';
  const int fields = sscanf(otaCommand, "$SPOTOTA ESP32 %lu %32s %c",
                            &imageSize, md5, &extra);
  if (fields != 2) {
    sendOtaError("invalid-command");
  } else {
    receiveBluetoothUpdate(static_cast<size_t>(imageSize), md5);
  }
  otaCommandLength = 0;
  otaCommandActive = false;
}

static bool makeBluetoothDiscoverable()
{
#ifdef ESP_IDF_VERSION_MAJOR
  const esp_err_t result =
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
#else
  const esp_err_t result =
      esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
#endif
  if (result != ESP_OK) {
    Serial.printf("Bluetooth discoverable mode failed: %s (%d)\n",
                  esp_err_to_name(result), static_cast<int>(result));
    return false;
  }
  return true;
}

static bool clearBluetoothBonds()
{
  int bondCount = esp_bt_gap_get_bond_device_num();
  if (bondCount < 0) {
    Serial.println("Bluetooth bond query failed");
    return false;
  }
  if (bondCount == 0) {
    Serial.println("No stored Bluetooth bonds");
    return true;
  }

  esp_bd_addr_t* devices = static_cast<esp_bd_addr_t*>(
      malloc(sizeof(esp_bd_addr_t) * static_cast<size_t>(bondCount)));
  if (devices == nullptr) {
    Serial.println("Bluetooth bond reset failed: out of memory");
    return false;
  }

  int listedCount = bondCount;
  const esp_err_t listResult =
      esp_bt_gap_get_bond_device_list(&listedCount, devices);
  if (listResult != ESP_OK) {
    Serial.printf("Bluetooth bond list failed: %s (%d)\n",
                  esp_err_to_name(listResult), static_cast<int>(listResult));
    free(devices);
    return false;
  }

  int removed = 0;
  for (int index = 0; index < listedCount; ++index) {
    const esp_err_t removeResult =
        esp_bt_gap_remove_bond_device(devices[index]);
    if (removeResult == ESP_OK) {
      ++removed;
    } else {
      Serial.printf("Bluetooth bond %d removal failed: %s (%d)\n",
                    index + 1, esp_err_to_name(removeResult),
                    static_cast<int>(removeResult));
    }
  }
  free(devices);

  Serial.printf("Bluetooth bonds removed: %d/%d\n", removed, listedCount);
  return removed == listedCount;
}

static void onBluetoothConfirmRequest(uint32_t numericValue)
{
  Serial.printf("Bluetooth pairing confirmation: %06lu\n",
                static_cast<unsigned long>(numericValue));
  SerialBT.confirmReply(true);
}

static void onBluetoothAuthComplete(bool success)
{
  Serial.println(success ? "Bluetooth pairing succeeded"
                         : "Bluetooth pairing failed");
  if (!success) {
    // Keep the bridge visible so the user can retry or clear stale bonds.
    makeBluetoothDiscoverable();
    Serial.println("Retry pairing, or hold BOOT for 3 seconds to clear bonds");
  }
}

static void printStatus()
{
  Serial.println();
  Serial.println("==============================================");
  Serial.println("ESP32-WROOM-32D Bluetooth <-> STM32 UART Bridge");
  Serial.println("==============================================");
  Serial.print("Bluetooth : ");
  Serial.println(BLUETOOTH_DEVICE_NAME);
  Serial.println("STM32 UART: USART3 115200 8N1");
  Serial.println("ESP32 TX  : GPIO17 -> STM32 PC11 RX");
  Serial.println("ESP32 RX  : GPIO16 <- STM32 PC10 TX");
  Serial.println("USB serial: diagnostics only");
  Serial.println("Pair reset: hold BOOT for 3 seconds after startup");
  Serial.println("Firmware OTA: supported over paired Bluetooth SPP");
  Serial.println("==============================================");
}

void setup()
{
  Serial.begin(115200);

  const uint32_t started = millis();
  while (!Serial && (millis() - started) < 2000) {
    delay(10);
  }

  Serial.printf("Reset reason: %d\n", static_cast<int>(esp_reset_reason()));
  pinMode(PAIRING_RESET_BUTTON, INPUT_PULLUP);

  if (!STM32Serial.setRxBufferSize(STM32_UART_RX_BUFFER_SIZE)) {
    Serial.println("STM32 UART RX buffer allocation failed");
  }
  STM32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX, STM32_TX);

  SerialBT.enableSSP();
  SerialBT.onConfirmRequest(onBluetoothConfirmRequest);
  SerialBT.onAuthComplete(onBluetoothAuthComplete);

  bool bluetoothStarted = false;
  for (int attempt = 1; attempt <= 3 && !bluetoothStarted; ++attempt) {
    bluetoothStarted = SerialBT.begin(BLUETOOTH_DEVICE_NAME);
    if (!bluetoothStarted) {
      Serial.printf("Bluetooth SPP startup failed (attempt %d/3)\n", attempt);
      SerialBT.end();
      delay(500);
    }
  }
  if (bluetoothStarted) {
    Serial.printf("Bluetooth SPP ready: %s\n", BLUETOOTH_DEVICE_NAME);
    makeBluetoothDiscoverable();
  } else {
    Serial.println("Bluetooth unavailable; restarting ESP32 in 3 seconds");
    delay(3000);
    ESP.restart();
  }

  printStatus();
}

void loop()
{
  const uint32_t now = millis();
  const bool pairingButtonPressed = digitalRead(PAIRING_RESET_BUTTON) == LOW;
  if (pairingButtonPressed) {
    if (pairingButtonPressedAt == 0) {
      pairingButtonPressedAt = now;
    } else if (!pairingResetHandled &&
               now - pairingButtonPressedAt >= PAIRING_RESET_HOLD_MS) {
      pairingResetHandled = true;
      Serial.println("BOOT held: clearing Bluetooth pairing information");
      clearBluetoothBonds();
      makeBluetoothDiscoverable();
      Serial.println("Pairing reset complete; search for SpotOMG-Bridge again");
    }
  } else {
    pairingButtonPressedAt = 0;
    pairingResetHandled = false;
  }

  if (!SerialBT.hasClient() &&
      now - lastDiscoverableRefresh >= DISCOVERABLE_REFRESH_MS) {
    lastDiscoverableRefresh = now;
    makeBluetoothDiscoverable();
  }

  uint8_t buffer[BRIDGE_CHUNK_SIZE];

  // Bluetooth SPP -> STM32. Collect a chunk before writing so long commands
  // do not pay one UART call per byte.
  while (SerialBT.available() > 0) {
    if (!otaCommandActive && SerialBT.peek() == '$') {
      otaCommandActive = true;
      otaCommandLength = 0;
    }
    if (otaCommandActive) {
      const int value = SerialBT.read();
      if (value < 0) {
        break;
      }
      if (value == '\n') {
        handleOtaCommand();
      } else if (value != '\r') {
        if (otaCommandLength + 1 >= sizeof(otaCommand)) {
          sendOtaError("command-too-long");
          otaCommandLength = 0;
          otaCommandActive = false;
        } else {
          otaCommand[otaCommandLength++] = static_cast<char>(value);
        }
      }
      continue;
    }
    size_t count = 0;
    while (count < sizeof(buffer) && SerialBT.available() > 0 &&
           SerialBT.peek() != '$') {
      const int value = SerialBT.read();
      if (value < 0) {
        break;
      }
      buffer[count++] = static_cast<uint8_t>(value);
    }
    if (count > 0 && STM32Serial.write(buffer, count) != count) {
      Serial.println("STM32 UART TX short write");
    }
  }

  // STM32 -> Bluetooth SPP. A large UART RX buffer absorbs diagnostic bursts,
  // and block writes avoid overflowing it while sending one SPP byte at a time.
  while (STM32Serial.available() > 0) {
    size_t count = 0;
    while (count < sizeof(buffer) && STM32Serial.available() > 0) {
      const int value = STM32Serial.read();
      if (value < 0) {
        break;
      }
      buffer[count++] = static_cast<uint8_t>(value);
    }
    if (count > 0 && SerialBT.hasClient()) {
      const size_t written = SerialBT.write(buffer, count);
      if (written != count) {
        Serial.printf("Bluetooth SPP TX short write: %u/%u\n",
                      static_cast<unsigned>(written),
                      static_cast<unsigned>(count));
      }
    }
  }

  delay(1);
}
