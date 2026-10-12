// Pico W probe: does the WiFi chip work? Shows its address, scans the networks around, then starts an access
// point "NavPlotter-probe" (password navplotter). If the start-up does not complete (a board that is not a
// real Pico W), the second core puts the board back in loader mode.
#include <Arduino.h>
#include <WiFi.h>

volatile bool startupDone = false;
void loop1() {
  if(!startupDone && millis() > 40000) rp2040.rebootToBootloader();
  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println("probe: scanning"); Serial.flush();
  int n = WiFi.scanNetworks();
  Serial.printf("probe: scan found %d networks\n", n);
  for(int i = 0; i < n && i < 12; i++) Serial.printf("probe:   ch %d rssi %d\n", (int)WiFi.channel(i), (int)WiFi.RSSI(i));
  uint8_t mac[6]; WiFi.macAddress(mac);
  Serial.printf("probe: mac %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]); Serial.flush();
  WiFi.mode(WIFI_AP);
  int r = WiFi.beginAP("NavPlotter-probe", "navplotter", 6);
  Serial.printf("probe: beginAP %d\n", r); Serial.flush();
  pinMode(LED_BUILTIN, OUTPUT);
  startupDone = true;
}

void loop() {
  static unsigned long last = 0;
  if(millis() - last < 3000) return;
  last = millis();
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  Serial.printf("probe: alive %lu s, stations %d\n", millis() / 1000, WiFi.softAPgetStationNum());
}
