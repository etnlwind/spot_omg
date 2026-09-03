/*
 * ESP32-WROOM-32D Bluetooth SPP <-> STM32 UART Bridge
 *
 * Wiring:
 *   ESP32 GPIO4 (TX)  -> STM32 PC11 (USART3_RX)
 *   ESP32 GPIO5 (RX)  <- STM32 PC10 (USART3_TX)
 *   ESP32 GND         -> STM32 GND
 *
 * USB serial is used only for firmware upload and diagnostics.
 */

#include <BluetoothSerial.h>
#include <esp_gap_bt_api.h>
#include <esp_idf_version.h>
#include <esp_system.h>

HardwareSerial STM32Serial(1);
BluetoothSerial SerialBT;

static constexpr const char* BLUETOOTH_DEVICE_NAME = "SpotOMG-Bridge";
static constexpr uint32_t STM32_BAUD = 115200;
static constexpr int STM32_RX = 5;  // GPIO5 <- STM32 PC10 TX
static constexpr int STM32_TX = 4;  // GPIO4 -> STM32 PC11 RX
static constexpr int PAIRING_RESET_BUTTON = 0;  // BOOT button
static constexpr uint32_t PAIRING_RESET_HOLD_MS = 3000;
static constexpr uint32_t DISCOVERABLE_REFRESH_MS = 5000;

static uint32_t pairingButtonPressedAt = 0;
static uint32_t lastDiscoverableRefresh = 0;
static bool pairingResetHandled = false;

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
  Serial.println("ESP32 TX  : GPIO4 -> STM32 PC11 RX");
  Serial.println("ESP32 RX  : GPIO5 <- STM32 PC10 TX");
  Serial.println("USB serial: diagnostics only");
  Serial.println("Pair reset: hold BOOT for 3 seconds after startup");
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

  // Bluetooth SPP -> STM32
  while (SerialBT.available() > 0) {
    STM32Serial.write(static_cast<uint8_t>(SerialBT.read()));
  }

  // STM32 -> Bluetooth SPP; mirror to USB for diagnostics only.
  while (STM32Serial.available() > 0) {
    const uint8_t byte = static_cast<uint8_t>(STM32Serial.read());
    Serial.write(byte);

    if (SerialBT.hasClient()) {
      SerialBT.write(byte);
    }
  }

  delay(1);
}
