//V5.1 
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
uint32_t pactotalDummy;
uint32_t vaactotalDummy;
double Waitdummy = 0;
uint32_t pactotalsensorSave = 1;
uint16_t vacusensorSave = 1;
uint16_t vacvsensorSave = 1;
uint16_t vacwsensorSave = 1;
uint16_t iacusensorSave = 1;
uint16_t iacvsensorSave = 1;
uint16_t iacwsensorSave = 1;
uint16_t tinvsensorSave = 1;
uint32_t vaactotalsensorSave = 1;

uint8_t buffer[SOLIS_S5_SERIAL_BUFFER_LEN] = {0};
uint8_t counter;
uint8_t index = 0;
uint8_t loopwait = 0;


uint16_t result;
uint8_t *point_to;
uint8_t result1;
uint8_t result2;


uint16_t CRC16 (const uint8_t *nData, uint16_t wLength)
{
static const uint16_t wCRCTable[] = {
0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

uint8_t nTemp;
uint16_t wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;

}


void s5_gr3p15kww3Component::setup() {
  // nothing To do here
}



void s5_gr3p15kww3Component::loop() {

  if (sensorupdateprogres > 0) {
    switch (sensorupdateprogres) {
      case 1:
        if (this->pactotalsensor != nullptr) {
          uint32_t v = this->messagedata[13] * 16777216 + this->messagedata[14] * 65536 + this->messagedata[15] * 256 + this->messagedata[16];
          pactotalDummy = v;
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
          this->iacusensor->publish_state((float)v * 0.1f);
        }

	    if (this->iacvsensor != nullptr) {
          uint16_t v = this->messagedata[29] * 256 + this->messagedata[30];
          this->iacvsensor->publish_state((float)v * 0.1f);
        }

	    if (this->iacwsensor != nullptr) {
          uint16_t v = this->messagedata[31] * 256 + this->messagedata[32];
          this->iacwsensor->publish_state((float)v * 0.1f);
        }

        if (this->tinvsensor != nullptr) {
          uint16_t v = this->messagedata[37] * 256 + this->messagedata[38];
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
          vaactotalDummy = v;
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
      if ((pactotalDummy / vaactotalDummy) > 1.0){
          float dummy = 1.0;
          this->pfacsensor->publish_state((float)dummy);
      }
      else if ((pactotalDummy / vaactotalDummy) <= 1.0) {
          this->pfacsensor->publish_state((float)pactotalDummy / vaactotalDummy);
      }
  }


  if (available() > 7 ) { // at least 7 bytes to start decoding
    if (CopyNext == 0) {
        reader = read();
        if (reader == 1){ //correct inverter adressed
            buffer[0] = reader;
            while (available() == 0){
                Waitdummy = 6321 * 6151;
            }
            reader = read();
            if (reader == 4){
                buffer[1] = reader;
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
                    while (available() < 4){
                        Waitdummy = 6321 * 6151;
                    }
                        
                    counter = 3;
                    while (counter < 8){
                        while (available() == 0){
                            Waitdummy = 6321 * 6151;
                        }
                        buffer[counter] = read();
                        counter = counter +1;
                    }
                    
                    point_to = &buffer[0];
                    result = crc16(point_to, 6);
                    result1 = result;
                    result2 = result >> 8;
                    if ((buffer[6] == result1) && (buffer[7] == result2)){
                        ESP_LOGD(TAG, "Message from stick to converter or converter to stick received , CRC = OK");
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
                reader = read();
                if ((reader == 52) || (reader == 56)) {
                    buffer[2] = reader;
                    if (Decoderselect == 1){
                        while (available() < 59){
                            Waitdummy = 6321 * 6151;
                        }
                        counter = 3;
                        while (counter < 61){
                            while (available() == 0){
                                Waitdummy = 6321 * 6151;
                            }
                            buffer[counter] = read();
                            counter = counter +1;
                        }
                        point_to = &buffer[0];
                        result = crc16(point_to, 59);
                        result1 = result;
                        result2 = result >> 8;
                        if ((buffer[59] == result1) && (buffer[60] == result2)){
                            memcpy(this->messagedata,buffer,61); //Copy message 1 for processing
                            ESP_LOGD(TAG, "Message 1 copied for processing , CRC OK");
                            sensorupdateprogres = 1;
                        }
                        else{
                            ESP_LOGD(TAG, "Message 1 CRC FAIL");
                            Decoderselect = 0;
                            CopyNext = 0;
                        }
                    }
                    else if (Decoderselect == 2) {
                        while (available() < 55){
                            Waitdummy = 6321 * 6151;
                        }
                        counter = 3;
                        while (counter < 57){
                            while (available() == 0){
                                Waitdummy = 6321 * 6151;
                            }
                            buffer[counter] = read();
                            counter = counter +1;
                        }
                        point_to = &buffer[0];
                        result = crc16(point_to, 55);
                        result1 = result;
                        result2 = result >> 8;
                        if ((buffer[55] == result1) && (buffer[56] == result2)){
                            memcpy(this->messagedata,buffer,61); //Copy message 1 for processing
                            ESP_LOGD(TAG, "Message 2 copied for processing , CRC OK");
                            sensorupdateprogres = 2;
                        }
                        else{
                            ESP_LOGD(TAG, "Message 2 CRC FAIL");
                            Decoderselect = 0;
                            CopyNext = 0;
                        }
                    }
                    else if (Decoderselect == 4) {
                        while (available() < 59){
                            Waitdummy = 6321 * 6151;
                        }
                        counter = 3;
                        while (counter < 61){
                            while (available() == 0){
                                Waitdummy = 6321 * 6151;
                            }
                            buffer[counter] = read();
                            counter = counter +1;
                        }
                        point_to = &buffer[0];
                        result = crc16(point_to, 59);
                        result1 = result;
                        result2 = result >> 8;
                        if ((buffer[59] == result1) && (buffer[60] == result2)){
                            memcpy(this->messagedata,buffer,61); //Copy message 1 for processing
                            ESP_LOGD(TAG, "Message 4 copied for processing , CRC OK");
                            sensorupdateprogres = 4;
                        }
                        else{
                            ESP_LOGD(TAG, "Message 4 CRC FAIL");
                            Decoderselect = 0;
                            CopyNext = 0;
                        }
                    }
                    CopyNext = 0;
                }
                else{
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
