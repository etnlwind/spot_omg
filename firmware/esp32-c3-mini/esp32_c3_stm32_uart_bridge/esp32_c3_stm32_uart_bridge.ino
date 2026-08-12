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
#include <cstring>

HardwareSerial STM32Serial(1);

struct WiFiCredential {
  const char* ssid;
  const char* password;
};

static const WiFiCredential WIFI_NETWORKS[] = {
  {"TP-Link_A9CF", "a@0128a@0128"},
  // {"SSID2", "PASSWORD2"},
  // {"SSID3", "PASSWORD3"},
};

static constexpr size_t MAX_WIFI_NETWORKS = 3;
static constexpr size_t WIFI_NETWORK_COUNT =
    sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);
static_assert(WIFI_NETWORK_COUNT > 0, "Register at least one Wi-Fi network");
static_assert(WIFI_NETWORK_COUNT <= MAX_WIFI_NETWORKS,
              "Up to three Wi-Fi networks can be registered");

static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static constexpr uint32_t WIFI_RETRY_DELAY_MS = 3000;

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
  WiFi.setAutoReconnect(false);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println("Scanning WiFi...");

    WiFi.disconnect();
    WiFi.scanDelete();
    const int networkCount = WiFi.scanNetworks();

    if (networkCount < 0) {
      Serial.printf("WiFi scan failed (error=%d)\n", networkCount);
    } else {
      const WiFiCredential* selectedNetwork = nullptr;
      int selectedScanIndex = -1;
      int32_t selectedRssi = INT32_MIN;

      Serial.println("Found:");

      for (int scanIndex = 0; scanIndex < networkCount; ++scanIndex) {
        for (size_t credentialIndex = 0;
             credentialIndex < WIFI_NETWORK_COUNT;
             ++credentialIndex) {
          const WiFiCredential& credential = WIFI_NETWORKS[credentialIndex];

          if (WiFi.SSID(scanIndex) != credential.ssid) {
            continue;
          }

          const int32_t rssi = WiFi.RSSI(scanIndex);
          Serial.printf("  %-24s RSSI=%ld dBm\n",
                        credential.ssid,
                        static_cast<long>(rssi));

          if (selectedNetwork == nullptr || rssi > selectedRssi) {
            selectedNetwork = &credential;
            selectedScanIndex = scanIndex;
            selectedRssi = rssi;
          }
          break;
        }
      }

      if (selectedNetwork != nullptr) {
        uint8_t selectedBssid[6];
        std::memcpy(selectedBssid, WiFi.BSSID(selectedScanIndex),
                    sizeof(selectedBssid));
        const int32_t selectedChannel = WiFi.channel(selectedScanIndex);

        Serial.println();
        Serial.println("Selected:");
        Serial.printf("  %s\n", selectedNetwork->ssid);
        Serial.println();
        Serial.print("Connecting");

        WiFi.begin(selectedNetwork->ssid,
                   selectedNetwork->password,
                   selectedChannel,
                   selectedBssid);

        const uint32_t started = millis();
        while (WiFi.status() != WL_CONNECTED &&
               (millis() - started) < WIFI_CONNECT_TIMEOUT_MS) {
          delay(500);
          Serial.print(".");
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
          WiFi.scanDelete();
          Serial.println("WiFi connected");
          Serial.printf("SSID : %s\n", WiFi.SSID().c_str());
          Serial.printf("RSSI : %ld dBm\n", static_cast<long>(WiFi.RSSI()));
          Serial.print("IP   : ");
          Serial.println(WiFi.localIP());
          return;
        }

        Serial.printf("Connection failed (status=%d)\n", WiFi.status());
      } else {
        Serial.println("  No registered networks are in range");
      }
    }

    WiFi.disconnect();
    WiFi.scanDelete();
    Serial.printf("Retrying in %lu seconds...\n",
                  static_cast<unsigned long>(WIFI_RETRY_DELAY_MS / 1000));
    delay(WIFI_RETRY_DELAY_MS);
  }
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
