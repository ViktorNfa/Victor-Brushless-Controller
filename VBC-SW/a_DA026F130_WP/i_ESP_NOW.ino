//###########################################
//      ESP-NOW FUNCTIONS DEFINITIONS
//###########################################
#ifdef ESP_NOW

#include <esp_now.h>
#include <WiFi.h>
#include <esp_private/wifi.h>   // for esp_wifi_internal_set_fix_rate
#include <esp_wifi.h>
#include <string.h>             // memcmp
#include <math.h>               // roundf, fabsf

// --- Config (match your sender) ---
#define CHANNEL 11
#define DATARATE WIFI_PHY_RATE_1M_L

// Broadcast peer (sender transmits to broadcast)
static const uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// *** Set to your sender's STA MAC (you said it ends with :58) ***
static const uint8_t master_mac[6] = { 0xD4, 0x8C, 0x49, 0xFC, 0x5C, 0x58 };

// --- These are defined in b_GLOBAL.ino; just extern them here ---
extern esp_now_peer_info_t peerInfo;
extern struct input_message  inputData;
extern struct output_message outputData;

// ---- Version-safe callback prototypes ----
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
static void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t *incomingData, int len);
static void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
#else
static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
#endif

// ---- Init ESPNOW (receiver) ----
void espNowInit() {
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  Serial.print("Controller MAC address (STA): ");
  Serial.println(WiFi.macAddress());

  // Safe bring-up sequence
  if (esp_wifi_stop() != ESP_OK) Serial.println("esp_wifi_stop: not started (ok)");
  esp_wifi_deinit();  // ignore error if not initialized
  wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
  my_config.ampdu_tx_enable = 0;
  ESP_ERROR_CHECK(esp_wifi_init(&my_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_wifi_internal_set_fix_rate(WIFI_IF_AP, true, DATARATE));
  ESP_ERROR_CHECK(esp_now_init());

  // Register callbacks (IDF v5/v4 safe)
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // Register a broadcast peer (so we can also send telemetry if needed)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;
  ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
}

// ---- RX callback: build Commander string and run it ----
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
static void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t *incomingData, int len) {
  if (!info) return;
  const uint8_t* mac = info->src_addr;
#else
static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  // Only accept from our sender
  if (memcmp(mac, master_mac, 6) != 0) return;
  if (len < (int)sizeof(input_message)) return;  // sanity

  memcpy(&inputData, incomingData, sizeof(inputData));
  if (inputData.act_id != ACT_ID) return;

  // Build command like "MC2" or "AP5.0" from up to 3 letters + number
  String extInput;
  if (inputData.act_commander1 != '0') extInput += inputData.act_commander1;
  if (inputData.act_commander2 != '0') extInput += inputData.act_commander2;
  if (inputData.act_commander3 != '0') extInput += inputData.act_commander3;
  if (extInput.length() == 0) return;   // nothing to run

  // Append numeric part — integer when effectively whole (so "MC2", not "MC2.000")
  float v   = inputData.act_target_value;
  float iv  = roundf(v);
  bool isInt = fabsf(v - iv) < 0.0005f;
  if (isInt) extInput += String((int)iv);
  else       extInput += String(v, 3);

  char ext_command[24];
  extInput.toCharArray(ext_command, sizeof(ext_command));
  commandExt.run(ext_command);
}

// ---- TX callback (optional debug) ----
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
static void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ESP-NOW TX OK" : "ESP-NOW TX FAIL");
}
#else
static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ESP-NOW TX OK" : "ESP-NOW TX FAIL");
}
#endif

// ---- Example telemetry sender (unused now) ----
void sendData() {
  // outputData.act_id = ACT_ID;
  // outputData.act_position = motor.shaft_angle;
  // esp_now_send(broadcastAddress, (uint8_t*)&outputData, sizeof(outputData));
}

#endif // ESP_NOW