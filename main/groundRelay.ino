#include <WiFi.h>
#include <esp_now.h>


// ESP-NOW
// command options: 
String command;
String data;

esp_now_peer_info_t groundInfo;
uint8_t MAC[] = {
  0x20, 0x9B, 0xA9, 0x68, 0xEB, 0xD4
};

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);

void setup() {
  Serial.begin(112700); 

  // start Wifi
  WiFi.mode(WIFI_STA);  
  while(!WiFi.STA.started()){ Serial.println("WIFI NOT STARTED"); }
  // init ESP NOW
  if (esp_now_init() != ESP_OK) { Serial.println("ESPNOW NOT STARTING"); }

  // register and add ground
  // memcpy(groundInfo.peer_addr, MAC, 6);
  // groundInfo.channel = 0;
  // groundInfo.encrypt = false;
  // if (esp_now_add_peer(&groundInfo) != ESP_OK) { Serial.println("CANSAT NOT FOUND"); }
  // // register callback function
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
}

void loop() {
  while(Serial.available()) {
    command = Serial.readString();
    send();
    Serial.println(command);
  }
}

void send() {
  // if (esp_now_send(MAC, (uint8_t *) &command, sizeof(command)) == ESP_OK) {
  //   // if sent correctly
  //   packetCount++;
  // }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));
  Serial.println(data);
}