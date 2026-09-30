#include <WiFi.h>
#include <esp_now.h>

esp_now_peer_info_t groundInfo;
// Ground MAC
uint8_t MAC[] = {
  0x20, 0x9B, 0xA9, 0x68, 0xEB, 0xD4
};