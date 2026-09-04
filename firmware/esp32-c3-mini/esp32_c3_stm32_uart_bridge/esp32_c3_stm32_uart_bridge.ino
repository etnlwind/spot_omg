/* ESP32 BLE GATT UART <-> STM32 USART3 bridge. */
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

HardwareSerial STM32Serial(1);
static constexpr const char* DEVICE_NAME = "SpotOMG-Bridge";
static constexpr const char* SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char* RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr const char* TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
static constexpr uint32_t STM32_BAUD = 115200;
static constexpr int STM32_RX = 16;
static constexpr int STM32_TX = 17;
static constexpr size_t UART_RX_BUFFER_SIZE = 4096;
static constexpr size_t BLE_NOTIFY_CHUNK = 180;
static BLECharacteristic* txCharacteristic = nullptr;
static volatile bool clientConnected = false;

class ServerCallbacks final : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer*) override {
    clientConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer*) override {
    clientConnected = false;
    Serial.println("BLE client disconnected; advertising");
    BLEDevice::startAdvertising();
  }
};

class RxCallbacks final : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    const std::string value = characteristic->getValue();
    if (!value.empty()) {
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
  if (!STM32Serial.setRxBufferSize(UART_RX_BUFFER_SIZE)) {
    Serial.println("STM32 UART RX buffer allocation failed");
  }
  STM32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX, STM32_TX);
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(185);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService* service = server->createService(SERVICE_UUID);
  txCharacteristic = service->createCharacteristic(
      TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic* rxCharacteristic = service->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxCharacteristic->setCallbacks(new RxCallbacks());
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("SpotOMG BLE UART bridge ready");
}

void loop() {
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
    txCharacteristic->setValue(buffer, count);
    txCharacteristic->notify();
  }
  delay(1);
}
