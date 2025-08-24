#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#if __has_include(<esp_mac.h>)
  #include <esp_mac.h>
#else
  #include <esp_system.h>
#endif
#include <esp_bt.h>
#include <ctype.h>  // toupper

#define ACT_ID 100
#define CHANNEL 11
static const uint8_t kBroadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

typedef struct input_message {
  int   act_id;
  char  act_commander1;
  char  act_commander2;
  char  act_commander3;
  float act_target_value;
} input_message;

esp_now_peer_info_t peer{};

// ---- TX stats + helpers ----
static volatile uint32_t tx_ok = 0, tx_fail = 0;

static String cmdToString(char c1, char c2, char c3) {
  String s;
  if (c1 != '0') s += c1;
  if (c2 != '0') s += c2;
  if (c3 != '0') s += c3;
  return s.length() ? s : "(none)";
}

// ---- Send callback (IDF v5 vs v4) ----
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
void onEspNowSend(const wifi_tx_info_t*, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) tx_ok++; else tx_fail++;
  Serial.printf("[ACK] %s  (ok=%u fail=%u)\n",
                status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL",
                tx_ok, tx_fail);
}
#else
void onEspNowSend(const uint8_t*, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) tx_ok++; else tx_fail++;
  Serial.printf("[ACK] %s  (ok=%u fail=%u)\n",
                status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL",
                tx_ok, tx_fail);
}
#endif

// ---- Serial wait ----
static void waitForSerial(uint32_t ms=3000){
  uint32_t t0 = millis();
  while (!Serial && (millis()-t0) < ms) { delay(10); }
  delay(150);
}

// ---- Print MACs without needing Wi-Fi ----
static void printEfuseMacs() {
  uint8_t mac_sta[6], mac_ap[6];
#if __has_include(<esp_mac.h>)
  esp_read_mac(mac_sta, ESP_MAC_WIFI_STA);
  esp_read_mac(mac_ap,  ESP_MAC_WIFI_SOFTAP);
#else
  esp_efuse_mac_get_default(mac_sta);
  memcpy(mac_ap, mac_sta, 6); mac_ap[5] = mac_sta[5] + 1;
#endif
  auto p=[&](const char* tag, uint8_t m[6]){
    Serial.printf("%s %02X:%02X:%02X:%02X:%02X:%02X\n", tag, m[0],m[1],m[2],m[3],m[4],m[5]);
  };
  Serial.println("EFUSE MACs (no Wi-Fi needed):");
  p("  STA:", mac_sta);
  p("   AP:", mac_ap);
  Serial.println("Use one of these in master_mac[] on the receiver (try STA first).");
}

// ---- Conservative radio bring-up ----
void safeRadioBringup() {
  Serial.println("Bringing radio up safely…");

  // 1) Disable BT to free coex memory
  btStop(); delay(50);

  // 2) Hard reset Wi-Fi to OFF first, then enable
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(200);

  // 3) AP+STA is often stable on some WROOM batches
  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP_STA);
  delay(200);

  Serial.print("STA MAC (API): "); Serial.println(WiFi.macAddress());
  Serial.print(" AP MAC (API): "); Serial.println(WiFi.softAPmacAddress());

  // 4) Init ESPNOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED");
    while (true) delay(1000);
  }
  delay(20);

  // 5) Lock to channel (optional)
  esp_err_t ch = esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("Set channel %d -> %s\n", CHANNEL, ch==ESP_OK?"OK":"ERR");

  // 6) Add broadcast peer
  memcpy(peer.peer_addr, kBroadcast, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer FAILED");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onEspNowSend);

  Serial.println("ESP-NOW ready. Type:  M1.0   E1   AP5.0   MC2");
}

// ---- Parse "CMD[CMD][CMD]<float>" robustly ----
static bool parseLine(const String& line, input_message& out) {
  String s = line; s.trim();
  if (!s.length()) return false;

  char c1='0', c2='0', c3='0';
  uint8_t n = 0;
  int i = 0;

  // capture up to 3 leading letters (A-Z or a-z) and uppercase them
  while (i < s.length() && n < 3) {
    char ch = s[i];
    bool isLetter = ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z'));
    if (!isLetter) break;
    ch = toupper((unsigned char)ch);
    if      (n == 0) c1 = ch;
    else if (n == 1) c2 = ch;
    else             c3 = ch;
    n++; i++;
  }

  // parse the rest as a number (empty → 0.0)
  float v = 0.0f;
  if (i < s.length()) v = s.substring(i).toFloat();

  out = {ACT_ID, c1, c2, c3, v};
  return true;
}

// ---- Arduino ----
void setup() {
  Serial.begin(115200);
  waitForSerial();
  Serial.println("\nBoot OK");

  printEfuseMacs();
  safeRadioBringup();
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch=='\n' || ch=='\r') {
      if (buf.length()) {
        // Echo raw line
        Serial.printf("> %s\n", buf.c_str());

        input_message m;
        if (parseLine(buf, m)) {
          String cmd = cmdToString(m.act_commander1, m.act_commander2, m.act_commander3);
          Serial.printf("[TX] ID=%d  CMD=%s  VAL=%.4f\n", m.act_id, cmd.c_str(), m.act_target_value);

          esp_err_t e = esp_now_send(kBroadcast, (uint8_t*)&m, sizeof(m));
          if (e != ESP_OK) {
            Serial.printf("[TX] esp_now_send error: %d\n", (int)e);
          }
        } else {
          Serial.println("[PARSE] ignored (empty or invalid)");
        }
        buf = "";
      }
    } else {
      buf += ch;
    }
  }
}