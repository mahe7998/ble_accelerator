#ifndef __bma250_h
#define __bma250_h

// Bosch BM250 definition
#define BMA250_I2CADDR 0x18
#define BMA250_update_time_64ms 0x08
#define BMA250_update_time_32ms 0x09
#define BMA250_update_time_16ms 0x0A
#define BMA250_update_time_8ms 0x0B
#define BMA250_update_time_4ms 0x0C
#define BMA250_update_time_2ms 0x0D
#define BMA250_update_time_1ms 0x0E
#define BMA250_update_time_05ms 0xF
#define BMA250_range_2g 0x03
#define BMA250_range_4g 0x05
#define BMA250_range_8g 0x08
#define BMA250_range_16g 0x0C

class BMA250 {
  public:
     BMA250() {
          bLow = true;
          i16value = 0;
     };
     void begin(uint8_t range, uint8_t bw) {
           Wire.beginTransmission(BMA250_I2CADDR);
           Wire.write(0x0F); 
           Wire.write(range);
           Wire.endTransmission();
           Wire.beginTransmission(BMA250_I2CADDR);
           Wire.write(0x10);
           Wire.write(bw);
           Wire.endTransmission();
     }
     void start_read() {
           Wire.beginTransmission(BMA250_I2CADDR);
           Wire.write(0x02);
           Wire.endTransmission();
           // Request six data bytes
           Wire.requestFrom(BMA250_I2CADDR, 6);
     }
     inline uint8_t read_next_val() { 
           if (bLow) {
                 i16value = ((int16_t)(((int16_t) Wire.read()) | (((int16_t) Wire.read()) << 8))) >> 6; // Only 10 bit significant
                 bLow = false;
                 return (uint8_t) i16value;
           }
           else {
                 bLow = true;
                 return (uint8_t) (i16value >> 8);
           }
           
     }
     bool bLow;
     int16_t i16value;
};

#endif
