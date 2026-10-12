// Pico W probe: WiFi access point only. If the start-up does not complete, the second core puts the board
// back in loader mode.
#include <Arduino.h>
#include <WiFi.h>

volatile bool startupDone = false;
void loop1() {
  if(!startupDone && millis() > 30000) rp2040.rebootToBootloader();
  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println("probe: start"); Serial.flush();
  WiFi.mode(WIFI_AP);
  Serial.println("probe: mode set"); Serial.flush();
  bool ok = WiFi.softAP("NavPlotter-probe", "navplotter");
  Serial.printf("probe: softAP %d\n", ok); Serial.flush();
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("probe: ip %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]); Serial.flush();
  pinMode(LED_BUILTIN, OUTPUT);
  startupDone = true;
}

void loop() {
  static unsigned long last = 0;
  if(Serial.available() && Serial.read() == 'b') rp2040.rebootToBootloader();
  if(millis() - last < 2000) return;
  last = millis();
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  Serial.printf("probe: alive %lu s, stations %d, heap %d\n", millis() / 1000, WiFi.softAPgetStationNum(), rp2040.getFreeHeap());
}
