/* ESP32 BLE GATT UART <-> STM32 USART3 bridge. */
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Update.h>
#include <esp_app_format.h>
#include <esp_system.h>
#include <mbedtls/sha256.h>

HardwareSerial STM32Serial(1);
static constexpr const char* DEVICE_NAME = "SpotOMG-Bridge";
static constexpr const char* SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char* RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char* TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char* DIAG_UUID = "6e400004-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr uint32_t STM32_BAUD = 115200;
static constexpr int STM32_RX = 16;
static constexpr int STM32_TX = 17;
static constexpr size_t UART_RX_BUFFER_SIZE = 4096;
static constexpr size_t BLE_NOTIFY_CHUNK = 180;
static constexpr uint32_t POWER_STABILIZE_MS = 2000U;
static constexpr uint32_t BLE_ADVERTISING_RETRY_MS = 5000U;
static constexpr uint8_t BLE_ADVERTISING_MAX_ATTEMPTS = 6U;
static constexpr uint32_t ESP32_OTA_IDLE_TIMEOUT_MS = 15000U;
static constexpr uint32_t ESP32_OTA_RESTART_DELAY_MS = 1500U;
static BLECharacteristic* txCharacteristic = nullptr;
static portMUX_TYPE diagnosticMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t consoleRxBytes = 0;
static uint32_t consoleRxAt = 0;
static uint32_t notifyAccepted = 0;
static uint32_t notifyFailed = 0;
static int lastNotifyStatus = -1;
static uint32_t lastNotifyCode = 0;

// SUCCESS_NOTIFY means accepted by the BLE stack, not acknowledged by the host.
class TxCallbacks final : public BLECharacteristicCallbacks {
 public:
  void onStatus(BLECharacteristic*, Status status, uint32_t code) override {
    portENTER_CRITICAL(&diagnosticMux);
    lastNotifyStatus = static_cast<int>(status);
    lastNotifyCode = code;
    if (status == SUCCESS_NOTIFY) ++notifyAccepted;
    else ++notifyFailed;
    portEXIT_CRITICAL(&diagnosticMux);
  }
};

class DiagnosticCallbacks final : public BLECharacteristicCallbacks {
 public:
  void onRead(BLECharacteristic* characteristic) override {
    uint32_t bytes, receivedAt, accepted, failed, code;
    int status;
    portENTER_CRITICAL(&diagnosticMux);
    bytes = consoleRxBytes;
    receivedAt = consoleRxAt;
    accepted = notifyAccepted;
    failed = notifyFailed;
    status = lastNotifyStatus;
    code = lastNotifyCode;
    portEXIT_CRITICAL(&diagnosticMux);
    char text[180];
    snprintf(text, sizeof(text),
             "bridge=rxdiag-v1 uptime=%lu reset=%d rx=%lu rx_at=%lu "
             "accepted=%lu failed=%lu status=%d code=%lu",
             static_cast<unsigned long>(millis()),
             static_cast<int>(esp_reset_reason()),
             static_cast<unsigned long>(bytes),
             static_cast<unsigned long>(receivedAt),
             static_cast<unsigned long>(accepted),
             static_cast<unsigned long>(failed), status,
             static_cast<unsigned long>(code));
    characteristic->setValue(text);
  }
};
static volatile bool clientConnected = false;
static volatile bool advertisingActive = false;
static volatile bool advertisingRestartPending = false;
static volatile uint32_t disconnectedAt = 0;
static uint32_t lastAdvertisingAttemptAt = 0;
static volatile uint8_t advertisingAttemptsSinceSuccess = 0;
static constexpr const char* STM32_IMAGE_TMP = "/stm32.tmp";
static constexpr const char* STM32_IMAGE_PATH = "/stm32.bin";
static constexpr size_t STM32_MAX_IMAGE_SIZE = 320U * 1024U;
static constexpr size_t STM32_FLASH_BLOCK = 1024U;
static File stm32Image;
static mbedtls_sha256_context stm32Sha;
static bool stm32UploadActive = false;
static volatile bool stm32FlashPending = false;
static size_t stm32ExpectedSize = 0;
static size_t stm32ReceivedSize = 0;
static uint8_t stm32ExpectedSha[32];
static mbedtls_sha256_context esp32Sha;
static bool esp32UploadActive = false;
static volatile bool esp32RestartPending = false;
static size_t esp32ExpectedSize = 0;
static size_t esp32ReceivedSize = 0;
static uint32_t esp32LastChunkAt = 0;
static uint32_t esp32RestartAt = 0;
static uint8_t esp32ExpectedSha[32];

static const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "external-pin";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt-watchdog";
    case ESP_RST_TASK_WDT: return "task-watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

static void onBleGapEvent(esp_gap_ble_cb_event_t event,
                          esp_ble_gap_cb_param_t* parameter) {
  if (event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT) {
    advertisingActive =
        parameter->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS;
    if (advertisingActive) {
      advertisingAttemptsSinceSuccess = 0;
    }
  } else if (event == ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT) {
    advertisingActive = false;
  }
}

static void startBleAdvertising(const char* reason) {
  if (clientConnected) return;
  lastAdvertisingAttemptAt = millis();
  if (advertisingAttemptsSinceSuccess < UINT8_MAX) {
    ++advertisingAttemptsSinceSuccess;
  }
  BLEDevice::startAdvertising();
  Serial.printf("BLE advertising requested: %s (attempt %u)\n", reason,
                advertisingAttemptsSinceSuccess);
}

static void bleReply(const String& line) {
  if (!clientConnected || txCharacteristic == nullptr) return;
  String framed = "$STM32OTA " + line + "\n";
  txCharacteristic->setValue(
      reinterpret_cast<uint8_t*>(const_cast<char*>(framed.c_str())),
      framed.length());
  txCharacteristic->notify();
}

static void esp32OtaReply(const String& line) {
  if (!clientConnected || txCharacteristic == nullptr) return;
  String framed = "$ESPOTA " + line + "\n";
  txCharacteristic->setValue(
      reinterpret_cast<uint8_t*>(const_cast<char*>(framed.c_str())),
      framed.length());
  txCharacteristic->notify();
}

static bool parseHexSha(const char* text, uint8_t output[32]) {
  if (strlen(text) != 64U) return false;
  for (size_t index = 0; index < 32U; ++index) {
    char pair[3] = {text[index * 2U], text[index * 2U + 1U], '\0'};
    char* end = nullptr;
    const unsigned long value = strtoul(pair, &end, 16);
    if (end == nullptr || *end != '\0') return false;
    output[index] = static_cast<uint8_t>(value);
  }
  return true;
}

static void abortEsp32Upload(bool notifyClient, const char* reason) {
  if (!esp32UploadActive) return;
  Update.abort();
  mbedtls_sha256_free(&esp32Sha);
  esp32UploadActive = false;
  esp32ExpectedSize = 0;
  esp32ReceivedSize = 0;
  if (notifyClient) esp32OtaReply("ERROR " + String(reason));
}

static bool beginEsp32Upload(const std::string& command) {
  unsigned int size = 0;
  char sha[65] = {};
  if (sscanf(command.c_str(), "$ESPOTA BEGIN %u %64s", &size, sha) != 2 ||
      size < 8U || !parseHexSha(sha, esp32ExpectedSha)) {
    esp32OtaReply("ERROR invalid-header");
    return false;
  }
  if (stm32UploadActive || stm32FlashPending || esp32RestartPending) {
    esp32OtaReply("ERROR busy");
    return false;
  }
  if (!Update.begin(size, U_FLASH)) {
    esp32OtaReply("ERROR " + String(Update.errorString()));
    return false;
  }
  mbedtls_sha256_init(&esp32Sha);
  if (mbedtls_sha256_starts_ret(&esp32Sha, 0) != 0) {
    Update.abort();
    mbedtls_sha256_free(&esp32Sha);
    esp32OtaReply("ERROR sha256-init");
    return false;
  }
  esp32ExpectedSize = size;
  esp32ReceivedSize = 0;
  esp32LastChunkAt = millis();
  esp32UploadActive = true;
  esp32OtaReply("READY");
  return true;
}

static void acceptEsp32Chunk(const uint8_t* data, size_t size) {
  if (size == 0U || esp32ReceivedSize + size > esp32ExpectedSize) {
    abortEsp32Upload(true, "size");
    return;
  }
  if (esp32ReceivedSize == 0U) {
    uint32_t appDescriptorMagic = 0;
    if (size < 36U || data[0] != ESP_IMAGE_HEADER_MAGIC) {
      abortEsp32Upload(true, "invalid-image");
      return;
    }
    memcpy(&appDescriptorMagic, data + 32U, sizeof(appDescriptorMagic));
    if (appDescriptorMagic != ESP_APP_DESC_MAGIC_WORD) {
      abortEsp32Upload(true, "not-application-image");
      return;
    }
  }
  if (Update.write(const_cast<uint8_t*>(data), size) != size) {
    const String error = Update.errorString();
    abortEsp32Upload(false, "write");
    esp32OtaReply("ERROR " + error);
    return;
  }
  if (mbedtls_sha256_update_ret(&esp32Sha, data, size) != 0) {
    abortEsp32Upload(true, "sha256-update");
    return;
  }
  esp32ReceivedSize += size;
  esp32LastChunkAt = millis();
  if (esp32ReceivedSize != esp32ExpectedSize) {
    esp32OtaReply("ACK " + String(esp32ReceivedSize));
    return;
  }

  uint8_t actualSha[32];
  const int shaResult = mbedtls_sha256_finish_ret(&esp32Sha, actualSha);
  mbedtls_sha256_free(&esp32Sha);
  esp32UploadActive = false;
  if (shaResult != 0 ||
      memcmp(actualSha, esp32ExpectedSha, sizeof(actualSha)) != 0) {
    Update.abort();
    esp32OtaReply("ERROR sha256");
    return;
  }
  if (!Update.end()) {
    esp32OtaReply("ERROR " + String(Update.errorString()));
    return;
  }
  esp32OtaReply("OK");
  esp32RestartAt = millis();
  esp32RestartPending = true;
  Serial.printf("ESP32 OTA verified: %u bytes; restarting soon\n",
                static_cast<unsigned>(esp32ReceivedSize));
}

static bool beginStm32Upload(const std::string& command) {
  unsigned int size = 0;
  char sha[65] = {};
  if (sscanf(command.c_str(), "$STM32OTA BEGIN %u %64s", &size, sha) != 2 ||
      size < 8U || size > STM32_MAX_IMAGE_SIZE ||
      !parseHexSha(sha, stm32ExpectedSha)) {
    bleReply("ERROR invalid-header");
    return false;
  }
  if (esp32UploadActive || esp32RestartPending) {
    bleReply("ERROR busy");
    return false;
  }
  if (stm32Image) stm32Image.close();
  SPIFFS.remove(STM32_IMAGE_TMP);
  stm32Image = SPIFFS.open(STM32_IMAGE_TMP, FILE_WRITE);
  if (!stm32Image) {
    bleReply("ERROR storage");
    return false;
  }
  stm32ExpectedSize = size;
  stm32ReceivedSize = 0;
  mbedtls_sha256_init(&stm32Sha);
  mbedtls_sha256_starts_ret(&stm32Sha, 0);
  stm32UploadActive = true;
  bleReply("READY");
  return true;
}

static void acceptStm32Chunk(const uint8_t* data, size_t size) {
  if (size == 0U || stm32ReceivedSize + size > stm32ExpectedSize ||
      stm32Image.write(data, size) != size) {
    stm32UploadActive = false;
    stm32Image.close();
    SPIFFS.remove(STM32_IMAGE_TMP);
    bleReply("ERROR write");
    return;
  }
  mbedtls_sha256_update_ret(&stm32Sha, data, size);
  stm32ReceivedSize += size;
  if (stm32ReceivedSize != stm32ExpectedSize) {
    bleReply("ACK " + String(stm32ReceivedSize));
    return;
  }

  uint8_t actualSha[32];
  mbedtls_sha256_finish_ret(&stm32Sha, actualSha);
  mbedtls_sha256_free(&stm32Sha);
  stm32Image.flush();
  stm32Image.close();
  stm32UploadActive = false;
  if (memcmp(actualSha, stm32ExpectedSha, sizeof(actualSha)) != 0) {
    SPIFFS.remove(STM32_IMAGE_TMP);
    bleReply("ERROR sha256");
    return;
  }
  SPIFFS.remove(STM32_IMAGE_PATH);
  if (!SPIFFS.rename(STM32_IMAGE_TMP, STM32_IMAGE_PATH)) {
    bleReply("ERROR rename");
    return;
  }
  bleReply("STORED");
  stm32FlashPending = true;
}

class ServerCallbacks final : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer*) override {
    clientConnected = true;
    advertisingActive = false;
    advertisingRestartPending = false;
    advertisingAttemptsSinceSuccess = 0;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer*) override {
    clientConnected = false;
    abortEsp32Upload(false, "disconnect");
    disconnectedAt = millis();
    advertisingRestartPending = true;
    Serial.println("BLE client disconnected; scheduling advertising");
  }
};

class RxCallbacks final : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    const std::string value = characteristic->getValue();
    if (!value.empty()) {
      if (esp32UploadActive) {
        acceptEsp32Chunk(
            reinterpret_cast<const uint8_t*>(value.data()), value.size());
        return;
      }
      if (stm32UploadActive) {
        acceptStm32Chunk(
            reinterpret_cast<const uint8_t*>(value.data()), value.size());
        return;
      }
      if (value.rfind("$STM32OTA BEGIN ", 0) == 0) {
        beginStm32Upload(value);
        return;
      }
      if (value.rfind("$ESPOTA BEGIN ", 0) == 0) {
        beginEsp32Upload(value);
        return;
      }
      const size_t written = STM32Serial.write(
          reinterpret_cast<const uint8_t*>(value.data()), value.size());
      if (written != value.size()) {
        Serial.printf("STM32 UART TX short write: %u/%u\n",
                      static_cast<unsigned>(written),
                      static_cast<unsigned>(value.size()));
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(POWER_STABILIZE_MS);
  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("ESP32 boot: reset=%s (%d), power stabilization=%lums\n",
                resetReasonName(resetReason), static_cast<int>(resetReason),
                static_cast<unsigned long>(POWER_STABILIZE_MS));
  if (!STM32Serial.setRxBufferSize(UART_RX_BUFFER_SIZE)) {
    Serial.println("STM32 UART RX buffer allocation failed");
  }
  STM32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX, STM32_TX);
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed; STM32 OTA disabled");
  }
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setCustomGapHandler(onBleGapEvent);
  BLEDevice::setMTU(185);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService* service = server->createService(SERVICE_UUID);
  txCharacteristic = service->createCharacteristic(
      TX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  txCharacteristic->setCallbacks(new TxCallbacks());
  txCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic* diagnostic = service->createCharacteristic(
      DIAG_UUID, BLECharacteristic::PROPERTY_READ);
  diagnostic->setCallbacks(new DiagnosticCallbacks());
  BLECharacteristic* rxCharacteristic = service->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxCharacteristic->setCallbacks(new RxCallbacks());
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  startBleAdvertising("startup");
  Serial.println("SpotOMG BLE UART bridge ready");
}

static bool readStm32Line(String& line, uint32_t timeoutMs) {
  line = "";
  const uint32_t started = millis();
  while (static_cast<uint32_t>(millis() - started) < timeoutMs) {
    while (STM32Serial.available() > 0) {
      const int value = STM32Serial.read();
      if (value < 0) break;
      if (value == '\n') {
        line.trim();
        return true;
      }
      if (value != '\r' && line.length() < 120U) line += static_cast<char>(value);
    }
    delay(1);
  }
  return false;
}

static bool waitForStm32Prefix(const char* prefix, uint32_t timeoutMs,
                               String* matched = nullptr) {
  const uint32_t started = millis();
  String line;
  while (static_cast<uint32_t>(millis() - started) < timeoutMs) {
    const uint32_t remaining = timeoutMs - (millis() - started);
    if (!readStm32Line(line, remaining)) return false;
    if (line.startsWith(prefix)) {
      if (matched != nullptr) *matched = line;
      return true;
    }
  }
  return false;
}

static uint32_t updateCrc32(uint32_t crc, uint8_t value) {
  crc ^= value;
  for (uint8_t bit = 0; bit < 8U; ++bit) {
    crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0UL);
  }
  return crc;
}

static bool programStm32FromFile() {
  File image = SPIFFS.open(STM32_IMAGE_PATH, FILE_READ);
  if (!image || image.size() < 8U || image.size() > STM32_MAX_IMAGE_SIZE) {
    bleReply("ERROR staged-image");
    return false;
  }
  uint32_t crc = 0xFFFFFFFFUL;
  while (image.available()) crc = updateCrc32(crc, static_cast<uint8_t>(image.read()));
  crc ^= 0xFFFFFFFFUL;
  image.seek(0);

  while (STM32Serial.available() > 0) STM32Serial.read();
  STM32Serial.print("fwupdate\n");
  STM32Serial.flush();
  if (!waitForStm32Prefix("SPOTBOOT 1", 8000U)) {
    image.close();
    bleReply("ERROR bootloader-timeout");
    return false;
  }

  char header[48];
  snprintf(header, sizeof(header), "SPOTFW 1 %u %08lx\n",
           static_cast<unsigned>(image.size()), static_cast<unsigned long>(crc));
  STM32Serial.print(header);
  STM32Serial.flush();
  if (!waitForStm32Prefix("SPOTBOOT READY", 15000U)) {
    image.close();
    bleReply("ERROR erase");
    return false;
  }

  uint8_t buffer[STM32_FLASH_BLOCK];
  size_t sent = 0U;
  while (image.available()) {
    const size_t count = image.read(buffer, sizeof(buffer));
    if (count == 0U || STM32Serial.write(buffer, count) != count) {
      image.close();
      bleReply("ERROR uart-write");
      return false;
    }
    STM32Serial.flush();
    sent += count;
    String acknowledgement;
    if (!waitForStm32Prefix("SPOTBOOT ACK ", 10000U, &acknowledgement) ||
        acknowledgement.substring(13).toInt() != static_cast<long>(sent)) {
      image.close();
      bleReply("ERROR uart-ack");
      return false;
    }
    bleReply("FLASH " + String(sent));
  }
  image.close();
  if (!waitForStm32Prefix("SPOTBOOT OK", 10000U)) {
    bleReply("ERROR verify");
    return false;
  }
  bleReply("OK");
  return true;
}

void loop() {
  if (esp32RestartPending) {
    if (static_cast<uint32_t>(millis() - esp32RestartAt) >=
        ESP32_OTA_RESTART_DELAY_MS) {
      Serial.println("ESP32 OTA reboot");
      Serial.flush();
      esp_restart();
    }
    delay(1);
    return;
  }
  if (esp32UploadActive &&
      static_cast<uint32_t>(millis() - esp32LastChunkAt) >=
          ESP32_OTA_IDLE_TIMEOUT_MS) {
    abortEsp32Upload(true, "timeout");
  }
  if (esp32UploadActive) {
    // Keep STM32 console traffic out of the acknowledged OTA response stream.
    while (STM32Serial.available() > 0) STM32Serial.read();
    delay(1);
    return;
  }
  if (advertisingRestartPending &&
      static_cast<uint32_t>(millis() - disconnectedAt) >= 500U) {
    advertisingRestartPending = false;
    startBleAdvertising("disconnect");
  }
  if (!clientConnected && !advertisingActive && !advertisingRestartPending &&
      static_cast<uint32_t>(millis() - lastAdvertisingAttemptAt) >=
          BLE_ADVERTISING_RETRY_MS) {
    if (advertisingAttemptsSinceSuccess >= BLE_ADVERTISING_MAX_ATTEMPTS) {
      Serial.println("BLE advertising recovery failed; restarting ESP32");
      Serial.flush();
      delay(50);
      esp_restart();
    }
    startBleAdvertising("watchdog");
  }
  if (stm32FlashPending) {
    stm32FlashPending = false;
    programStm32FromFile();
  }
  if (!clientConnected || STM32Serial.available() <= 0) {
    delay(1);
    return;
  }
  uint8_t buffer[BLE_NOTIFY_CHUNK];
  size_t count = 0;
  while (count < sizeof(buffer) && STM32Serial.available() > 0) {
    const int value = STM32Serial.read();
    if (value >= 0) buffer[count++] = static_cast<uint8_t>(value);
  }
  if (count > 0) {
    portENTER_CRITICAL(&diagnosticMux);
    consoleRxBytes += count;
    consoleRxAt = millis();
    portEXIT_CRITICAL(&diagnosticMux);
    txCharacteristic->setValue(buffer, count);
    txCharacteristic->notify();
  }
  delay(1);
}
