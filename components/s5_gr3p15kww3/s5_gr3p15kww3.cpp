#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"
#include "s5_gr3p15kww3.h"

namespace esphome {
namespace s5_gr3p15kww3 {

static const char *TAG = "s5_gr3p15kww3";

uint8_t Decoderselect = 0;
uint8_t Decodeddone = 0;
uint8_t CopyNext = 0;
uint8_t sensorupdateprogres = 0;

uint8_t reader = 0;
uint32_t pactotaldDummy;
uint32_t vaactotalDummy;
double Waitdummy = 0;

char buffer[SOLIS_S5_SERIAL_BUFFER_LEN] = {0};
uint8_t counter;
uint8_t index = 0;
uint8_t loopwait = 0;

void s5_gr3p15kww3Component::setup() {
  // nothing To do here
}



void s5_gr3p15kww3Component::loop() {

  if (sensorupdateprogres > 0) {
    switch (sensorupdateprogres) {
      case 1:
        if (this->pactotalsensor != nullptr) {
          uint32_t v = this->messagedata[13] * 16777216 + this->messagedata[14] * 65536 + this->messagedata[15] * 256 + this->messagedata[16];
          pactotaldDummy = this->messagedata[13] * 16777216 + this->messagedata[14] * 65536 + this->messagedata[15] * 256 + this->messagedata[16];
	  if (v > 30000){ 
              v = 1;
          }
          if (pactotaldDummy > 30000){
              pactotaldDummy = 1;
          }	
          this->pactotalsensor->publish_state((float)v);
        }
	
	if (this->etotalsensor != nullptr) {
          uint32_t v = this->messagedata[21] * 16777216 + this->messagedata[22] * 65536 + this->messagedata[23] * 256 + this->messagedata[24];
          this->etotalsensor->publish_state((float)v);
        }
	
	    if (this->emonthsensor != nullptr) {
          uint32_t v = this->messagedata[25] * 16777216 + this->messagedata[26] * 65536 + this->messagedata[27] * 256 + this->messagedata[28];
          this->emonthsensor->publish_state((float)v);
        }

	if (this->edaysensor != nullptr) {
          uint16_t v = this->messagedata[33] * 256 + this->messagedata[34];
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
        Decodeddone = Decodeddone + 1;
        sensorupdateprogres = 0;
        Decoderselect = 0;
        counter = 0;
        while(counter < 61){
            buffer[counter] = 0;
            counter = counter + 1;
        }
        ESP_LOGD(TAG, "Message 1 decoded");
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
	  if(v >1000){
            v = 1;
          }
          this->iacusensor->publish_state((float)v * 0.1f);
        }

	if (this->iacvsensor != nullptr) {
          uint16_t v = this->messagedata[29] * 256 + this->messagedata[30];
	  if(v >1000){
              v = 1;
          }	
          this->iacvsensor->publish_state((float)v * 0.1f);
        }

	if (this->iacwsensor != nullptr) {
          uint16_t v = this->messagedata[31] * 256 + this->messagedata[32];
	  if(v >1000){
              v = 1;
          }
          this->iacwsensor->publish_state((float)v * 0.1f);
        }

        if (this->tinvsensor != nullptr) {
          uint16_t v = this->messagedata[37] * 256 + this->messagedata[38];
	  if(v > 2000){
              v = 1;
          }
          this->tinvsensor->publish_state((float)v*0.1f);
        }
        Decodeddone = Decodeddone + 1;
        sensorupdateprogres = 0;
        Decoderselect = 0;
        counter = 0;
        while(counter < 61){
            buffer[counter] = 0;
            counter = counter + 1;
        }
        ESP_LOGD(TAG, "Message 2 decoded");
	    break;
      case 4:
        if (this->vaactotalsensor != nullptr) {
          uint32_t v = this->messagedata[17] * 16777216 + this->messagedata[18] * 65536 + this->messagedata[19] * 256 + this->messagedata[20];
          vaactotalDummy = this->messagedata[17] * 16777216 + this->messagedata[18] * 65536 + this->messagedata[19] * 256 + this->messagedata[20];
          this->vaactotalsensor->publish_state((float)v);
        }
        Decodeddone = Decodeddone + 1;
        sensorupdateprogres = 0;
        Decoderselect = 0;
        counter = 0;
        while(counter < 61){
            buffer[counter] = 0;
            counter = counter + 1;
        }
        ESP_LOGD(TAG, "Message 4 decoded");
        break;

    }
    
    this->sensorupdateprogress--;
  }

  if (Decodeddone == 3) {
      Decodeddone = 0;
      if ((vaactotalDummy / pactotaldDummy) > 1.0){
          float dummy = 1.0;
          this->pfacsensor->publish_state((float)dummy);
      }
      else if ((vaactotalDummy / pactotaldDummy) <= 1.0) {
          this->pfacsensor->publish_state((float)vaactotalDummy / pactotaldDummy);
      }
  }


  if (available() > 7 ) { // at least 7 bytes to start decoding
    if (CopyNext == 0) {
        reader = read();
        if (reader == 1){ //correct inverter adressed
            buffer[0] = reader;
            ESP_LOGD(TAG, "1 detected");
            while (available() == 0){
                Waitdummy = 6321 * 6151;
            }
            reader = read();
            if (reader == 4){
                buffer[1] = reader;
                ESP_LOGD(TAG, "4 detected");
            }
            else if (reader == 1){
                buffer[0] = reader;
                while (available() == 0){
                Waitdummy = 6321 * 6151;
                }
                reader = read();
                if (reader == 4){
                buffer[1] = reader;
                }
                
            }
            
            
            if ((buffer[0] == 1) && (buffer[1] == 4)){
            
                while (available() == 0){
                Waitdummy = 6321 * 6151;
                }
                reader = read();
                if (reader == 11){
                    buffer[2] = reader;
                    ESP_LOGD(TAG, "11 detected");
                    while (available() < 4){
                        Waitdummy = 6321 * 6151;
                    }
                        
                    counter = 3;
                    while (counter < 7){
                        while (available() == 0){
                            Waitdummy = 6321 * 6151;
                        }
                        buffer[counter] = read();
                        counter = counter +1;
                    }
                    
                    ESP_LOGD(TAG, "Message from stick to converter or converter to stick received");
                    
                    if ((buffer[2] == 11) && (buffer[3] == 183) && (buffer[4] == 0) && (buffer[5] == 28)){
	                    Decoderselect = 1;
      	                ESP_LOGD(TAG, "Stick request 1 received");
      	                CopyNext = 1;
                    }	
                    else if ((buffer[2] == 11) && (buffer[3] == 208) && (buffer[4] == 0) && (buffer[5] == 26)){
	                    Decoderselect = 2;
	                    ESP_LOGD(TAG, "Stick request 2 received");
	                    CopyNext = 1;
                    }	
                    else if ((buffer[2] == 11) && (buffer[3] == 234) && (buffer[4] == 0) && (buffer[5] == 28)){
	                    Decoderselect = 4;
	                    ESP_LOGD(TAG, "Stick request 4 received");
	                    CopyNext = 1;
                    }	
                }
                    
            }
                
        }
        
      
    }
    
    else if (CopyNext == 1){
        reader = read();
        if (reader == 1){
            buffer[0] = reader;
            while (available() == 0){
                Waitdummy = 6321 * 6151;
            }
            reader = read();
            if (reader == 4){
                buffer[1] = reader;
                if ((buffer[0] == 1) && (buffer[1] == 4)) {
                    if (Decoderselect == 1){
                        while (available() < 57){
                            Waitdummy = 6321 * 6151;
                        }
                        counter = 2;
                        while (counter < 57){
                            while (available() == 0){
                                Waitdummy = 6321 * 6151;
                            }
                            buffer[counter] = read();
                            counter = counter +1;
                        }
                        memcpy(this->messagedata,buffer,59); //Copy message 1 for processing
                        ESP_LOGD(TAG, "Message 1 copied for processing");
                        sensorupdateprogres = 1;
                    }
                    else if (Decoderselect == 2) {
                        while (available() < 55){
                            Waitdummy = 6321 * 6151;
                        }
                        counter = 2;
                        while (counter < 55){
                            while (available() == 0){
                                Waitdummy = 6321 * 6151;
                            }
                            buffer[counter] = read();
                            counter = counter +1;
                        }
                        memcpy(this->messagedata,buffer,55); //Copy message 2 for processing
                        ESP_LOGD(TAG, "Message 2 copied for processing");
                        sensorupdateprogres = 2;
                    }
                    else if (Decoderselect == 4) {
                        while (available() < 57){
                            Waitdummy = 6321 * 6151;
                        }
                        counter = 2;
                        while (counter < 57){
                            while (available() == 0){
                                Waitdummy = 6321 * 6151;
                            }
                            buffer[counter] = read();
                            counter = counter +1;
                        }
                        memcpy(this->messagedata,buffer,59); //Copy message 4 for processing
                        ESP_LOGD(TAG, "Message 4 copied for processing");
                        sensorupdateprogres = 4;
                    }
                    CopyNext = 0;
                }
            }
        }
    }
  }
}




void s5_gr3p15kww3Component::update() {
    //nothing to do here
}

void s5_gr3p15kww3Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Solis S5 Component");
}

} // namespace dekacontroller
} // namespace esphome
