/*
 * ESP32-C3 SuperMini Wi-Fi <-> STM32 UART Bridge
 *
 * Wiring:
 *   ESP32-C3 GPIO4 (TX)  -> STM32 PC11 (USART3_RX)
 *   ESP32-C3 GPIO5 (RX)  <- STM32 PC10 (USART3_TX)
 *   ESP32-C3 GND         -> STM32 GND
 *
 * STM32 USART3:
 *   TX = PC10
 *   RX = PC11
 *   115200 / 8-N-1
 *
 * Arduino IDE:
 *   Board: ESP32C3 Dev Module
 *   USB CDC On Boot: Enabled
 */

#include <WiFi.h>

HardwareSerial STM32Serial(1);

const char* WIFI_SSID     = "REMOVED";
const char* WIFI_PASSWORD = "REMOVED";

static constexpr uint16_t TCP_PORT = 3333;
WiFiServer server(TCP_PORT);
WiFiClient client;

static constexpr uint32_t STM32_BAUD = 115200;
static constexpr int STM32_RX = 5;  // GPIO5 <- STM32 PC10 TX
static constexpr int STM32_TX = 4;  // GPIO4 -> STM32 PC11 RX

static void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
}

static void printStatus()
{
  Serial.println();
  Serial.println("====================================");
  Serial.println("ESP32-C3 WiFi <-> STM32 UART Bridge");
  Serial.println("====================================");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("TCP port  : ");
  Serial.println(TCP_PORT);
  Serial.println("STM32 UART: USART3 115200 8N1");
  Serial.println("ESP32 TX  : GPIO4 -> STM32 PC11 RX");
  Serial.println("ESP32 RX  : GPIO5 <- STM32 PC10 TX");
  Serial.println("====================================");
}

void setup()
{
  Serial.begin(115200);

  const uint32_t started = millis();
  while (!Serial && (millis() - started) < 2000) {
    delay(10);
  }

  STM32Serial.begin(
      STM32_BAUD,
      SERIAL_8N1,
      STM32_RX,
      STM32_TX
  );

  delay(100);

  connectWiFi();

  server.begin();
  server.setNoDelay(true);

  printStatus();
  Serial.println("TCP server ready");
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
    if (client) {
      client.stop();
    }

    Serial.println("WiFi disconnected - reconnecting...");
    connectWiFi();
    printStatus();
  }

  if (!client || !client.connected()) {
    WiFiClient incoming = server.accept();

    if (incoming) {
      if (client) {
        client.stop();
      }

      client = incoming;
      client.setNoDelay(true);

      Serial.print("TCP client connected: ");
      Serial.println(client.remoteIP());

      client.println();
      client.println("ESP32-C3 -> STM32 bridge connected");
    }
  }

  // TCP -> STM32
  if (client && client.connected()) {
    while (client.available() > 0) {
      STM32Serial.write((uint8_t)client.read());
    }
  }

  // USB Serial Monitor -> STM32 (optional debug path)
  while (Serial.available() > 0) {
    STM32Serial.write((uint8_t)Serial.read());
  }

  // STM32 -> TCP + USB Serial Monitor
  while (STM32Serial.available() > 0) {
    const uint8_t b = (uint8_t)STM32Serial.read();

    Serial.write(b);

    if (client && client.connected()) {
      client.write(b);
    }
  }

  delay(1);
}
