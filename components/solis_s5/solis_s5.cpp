#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
//#include "esphome/components/text_sensor/text_sensor.h"
//#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"
#include "solis_s5.h"

namespace esphome {
namespace solis_s5 {

static const char *TAG = "solis_s5";

void SolisS5Component::setup() {
  // nothing to do here
}

void SolisS5Component::loop() {

  if (this->sensorupdateprogress > 0) {
    switch (this->sensorupdateprogress) {
      case 1:
        if (this->pactotalsensor != nullptr) {
          uint16_t v = this->messagedata[13] * 16777216 + this->messagedata[14] * 65536 + this->messagedata[15] * 256 + this->messagedata[16];
          this->pactotalsensor->publish_state((float)v);
        }
	
	if (this->etotalsensor != nullptr) {
          uint16_t v = this->messagedata[21] * 16777216 + this->messagedata[22] * 65536 + this->messagedata[23] * 256 + this->messagedata[24];
          this->etotalsensor->publish_state((float)v);
        }
	
	if (this->emonthsensor != nullptr) {
          uint16_t v = this->messagedata[25] * 256 + this->messagedata[27];
          this->emonthsensor->publish_state((float)v);
        }

	if (this->edaysensor != nullptr) {
          uint16_t v = this->messagedata[33] * 256 + this->messagedata[38];
          this->edaysensor->publish_state((float)v*0.1f);
        }

	if (this->vdc1sensor != nullptr) {
          uint16_t v = this->messagedata[47] * 256 + this->messagedata[48];
          this->vdc1sensor->publish_state((float)v * 0.1f);
        }

	if (this->idc1sensor != nullptr) {
          uint16_t v = this->messagedata[49] * 256 + this->messagedata[50];
          this->idc1sensor->publish_state((float)v * 0.1f);
        }
	
	if (this->vdc2sensor != nullptr) {
          uint16_t v = this->messagedata[51] * 256 + this->messagedata[52];
          this->vdc2sensor->publish_state((float)v * 0.1f);
        }

	if (this->idc2sensor != nullptr) {
          uint16_t v = this->messagedata[53] * 256 + this->messagedata[54];
          this->idc2sensor->publish_state((float)v * 0.1f);
        }
	
	if (this->pdc1sensor != nullptr) {
          uint16_t v1 = this->messagedata[47] * 256 + this->messagedata[48];
          uint16_t i1 = this->messagedata[49] * 256 + this->messagedata[50];
          this->pdc1sensor->publish_state((float)v1 * (float)i1 * 0.01f);
        }

	if (this->pdc2sensor != nullptr) {
          uint16_t v2 = this->messagedata[51] * 256 + this->messagedata[52];
          uint16_t i2 = this->messagedata[53] * 256 + this->messagedata[54];
          this->pdc2sensor->publish_state((float)v2 * (float)i2 * 0.01f);
        }
        break;

      case 2:
        if (this->vacusensor != nullptr) {
          uint16_t v = this->messagedata[21] * 256 + this->messagedata[22];
          this->vacusensor->publish_state((float)v * 0.1f);
        }

	if (this->vacvsensor != nullptr) {
          uint16_t v = this->messagedata[23] * 256 + this->messagedata[24];
          this->vacvsensor->publish_state((float)v * 0.1f);
	      }

	if (this->vacwsensor != nullptr) {
          uint16_t v = this->messagedata[25] * 256 + this->messagedata[26];
          this->vacwsensor->publish_state((float)v * 0.1f);
        }

	if (this->iacusensor != nullptr) {
          uint16_t v = this->messagedata[27] * 256 + this->messagedata[28];
          this->iacusensor->publish_state((float)v * 0.1f);
        }

	if (this->iacvsensor != nullptr) {
          uint16_t v = this->messagedata[29] * 256 + this->messagedata[30];
          this->iacvsensor->publish_state((float)v * 0.1f);
        }

	if (this->iacwsensor != nullptr) {
          uint16_t v = this->messagedata[31] + 256 + this->messagedata[32];
          this->iacwsensor->publish_state((float)v * 0.1f);
        }

        if (this->tigbtsensor != nullptr) {
          uint16_t v = this->messagedata[37] * 256 + this->messagedata[38];
          this->tigbtsensor->publish_state((float)v*0.1f);
        }
	break;
      case 4:
	if (this->vaactotalsensor != nullptr) {
          uint16_t v = this->messagedata[17] * 16777216 + this->messagedata[18] * 65536 + this->messagedata[19] * 256 + this->messagedata[20];
          this->vaactotalsensor->publish_state((float)v);
        }
	break;

    }
    this->sensorupdateprogress--;
  }

  static char buffer[SOLIS_S5_SERIAL_BUFFER_LEN] = {0};
  static uint8_t index = 0;
  static uint8_t loopwait = 0;

  while (available()) {
    buffer[index] = read();
    index++;
    index%=SOLIS_S5_SERIAL_BUFFER_LEN;
    loopwait = 0;
  }
  if (index > 0) {
    loopwait++;
  }
  
  if (loopwait > SOLIS_S5_LOOP_WAIT) { // some time has passed without receiving another character. this should be the end of a message.
    ESP_LOGV(TAG, "message recieved len=%d", index);
    if ((buffer[0] == 1) && (buffer[1] == 4)) { // message starts with the right preamble
      if ((buffer[2] == 11) && (buffer[3] == 183) && (buffer[4] == 0)) && (buffer[5] == 28)){
	Decoderselect = 1;
	ESP_LOGD(TAG, "Stick request 1 received");
      }	
      if ((buffer[2] == 11) && (buffer[3] == 208) && (buffer[4] == 0)) && (buffer[5] == 26)){
	Decoderselect = 1;
	ESP_LOGD(TAG, "Stick request 2 received");
      }	
      if ((buffer[2] == 11) && (buffer[3] == 234) && (buffer[4] == 0)) && (buffer[5] == 28)){
	Decoderselect = 1;
	ESP_LOGD(TAG, "Stick request 4 received");
      }	
      
      uint8_t msglen = buffer[3];
      if (index == msglen + 5) { // messasge has correct length
        uint8_t csum = 0;
        for (uint8_t i = 1; i < msglen+4; i++) { // csum after preamble, before tx csum
          csum += buffer[i];
        }
        if (csum == buffer[msglen+4]) { // checksum ok
          if ((buffer[2] == 161) && (msglen == 80)) { // inverter response
            memcpy(this->messagedata,buffer,SOLIS_S5_SERIAL_BUFFER_LEN); // copy message for processing on next update cycle
            this->messagelength = index; // length > 0 indicates the message data has been updated / ready for parsing
            ESP_LOGD(TAG, "inverter data received");
          } else if ((buffer[2] == 193) && (msglen == 40)) { // inverter config response
            ESP_LOGD(TAG, "inverter config response received");
          }
        } else {
          ESP_LOGV(TAG, "message checksum fail; discarding. csum = 0x%02X, check = 0x%02X", buffer[msglen+4], csum);
        }
      } else if ((msglen == 0) && (index == 55)) { // wifi stick command
        ESP_LOGD(TAG, "wifi stick command received; ignoring");
      } else {
        ESP_LOGV(TAG, "message insufficient length (requested: %d, received: %d); discarding", msglen+5, index);
      }
    } else {
      ESP_LOGV(TAG, "message received, invalid start character");
    }
    // reset message, ready for next one
    loopwait = 0;
    index = 0;
  }

}

void SolisS5Component::update() {
  if (this->messagelength > 0) { // if there's a message pending
    ESP_LOGV(TAG, "processing latest message");
    this->messagelength = 0; // indicate message is parsed   
    this->sensorupdateprogress = SOLIS_S5_SENSOR_COUNT; // start to update sensors
  } else {
    ESP_LOGV(TAG, "no data received");
  }
}

void SolisS5Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Solis S5 Component");
}

} // namespace dekacontroller
} // namespace esphome
