#include "lps.h"
#include "driver/i2c.h"
#include <math.h>
#include <stdbool.h>

#define SA0_LOW_ADDRESS 0b1011100
#define SA0_HIGH_ADDRESS 0b1011101

#define LPS331AP_WHO_ID 0xBB
#define LPS25H_WHO_ID 0xBD
#define LPS22DF_WHO_ID 0xB4

bool LPS_init(LPS *sensor){
  sensor->_device = device_auto;
  sensor->address = SA0_HIGH_ADDRESS;
  sensor->sa0 = sa0_auto;

  if(!detectDeviceAndAddress(sensor, device_auto, sa0_auto)) return false;
    
  switch(sensor->_device){
    case device_25H:
      sensor->translated_regs[-REF_P_L]       = LPS25H_REF_P_L; 
      sensor->translated_regs[-REF_P_H]       = LPS25H_REF_P_H;
      sensor->translated_regs[-CTRL_REG1]     = LPS25H_CTRL_REG1;
      sensor->translated_regs[-CTRL_REG2]     = LPS25H_CTRL_REG2;
      sensor->translated_regs[-CTRL_REG3]     = LPS25H_CTRL_REG3;
      sensor->translated_regs[-CTRL_REG4]     = LPS25H_CTRL_REG4;
      sensor->translated_regs[-INTERRUPT_CFG] = LPS25H_INTERRUPT_CFG;
      sensor->translated_regs[-INT_SOURCE]    = LPS25H_INT_SOURCE;
      sensor->translated_regs[-FIFO_CTRL]     = LPS25H_FIFO_CTRL;
      sensor->translated_regs[-THS_P_L]       = LPS25H_THS_P_L;
      sensor->translated_regs[-THS_P_H]       = LPS25H_THS_P_H;
      sensor->translated_regs[-RPDS_L]        = LPS25H_RPDS_L;
      sensor->translated_regs[-RPDS_H]        = LPS25H_RPDS_H;
      return true;
    
    case device_331AP:
      sensor->translated_regs[-REF_P_L]       = LPS331AP_REF_P_L; 
      sensor->translated_regs[-REF_P_H]       = LPS331AP_REF_P_H;
      sensor->translated_regs[-CTRL_REG1]     = LPS331AP_CTRL_REG1;
      sensor->translated_regs[-CTRL_REG2]     = LPS331AP_CTRL_REG2;
      sensor->translated_regs[-CTRL_REG3]     = LPS331AP_CTRL_REG3;
      sensor->translated_regs[-INTERRUPT_CFG] = LPS331AP_INTERRUPT_CFG;
      sensor->translated_regs[-INT_SOURCE]    = LPS331AP_INT_SOURCE;
      sensor->translated_regs[-THS_P_L]       = LPS331AP_THS_P_L;
      sensor->translated_regs[-THS_P_H]       = LPS331AP_THS_P_H;
      return true;

    case device_22DF:
      sensor->translated_regs[-REF_P_L]       = LPS22DF_REF_P_L; 
      sensor->translated_regs[-REF_P_H]       = LPS22DF_REF_P_H;
      sensor->translated_regs[-CTRL_REG1]     = LPS22DF_CTRL_REG1;
      sensor->translated_regs[-CTRL_REG2]     = LPS22DF_CTRL_REG2;
      sensor->translated_regs[-CTRL_REG3]     = LPS22DF_CTRL_REG3;
      sensor->translated_regs[-CTRL_REG4]     = LPS22DF_CTRL_REG4;
      sensor->translated_regs[-INTERRUPT_CFG] = LPS22DF_INTERRUPT_CFG;
      sensor->translated_regs[-INT_SOURCE]    = LPS22DF_INT_SOURCE;
      sensor->translated_regs[-FIFO_CTRL]     = LPS22DF_FIFO_CTRL;
      sensor->translated_regs[-THS_P_L]       = LPS22DF_THS_P_L;
      sensor->translated_regs[-THS_P_H]       = LPS22DF_THS_P_H;
      sensor->translated_regs[-RPDS_L]        = LPS22DF_RPDS_L;
      sensor->translated_regs[-RPDS_H]        = LPS22DF_RPDS_H;
      return true;
    
    default:
      return false;
  }
}

void LPS_enable(LPS sensor){
  if(sensor._device == device_22DF){

    writeReg(sensor, CTRL_REG1, 0x18);
    
    writeReg(sensor, CTRL_REG3, 0x01);

  }else if(sensor._device == device_25H){

    writeReg(sensor, CTRL_REG1, 0xB0);

  }else if(sensor._device == device_331AP){

    writeReg(sensor, CTRL_REG1, 0xE0);

  }
}

void writeReg(LPS sensor, int reg, uint8_t value){
  if( reg < 0){
    reg = sensor.translated_regs[-reg];
  }
  
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (sensor.address << 1) | I2C_MASTER_WRITE, 1);

  i2c_master_write_byte(cmd, reg, true);
  
  i2c_master_write_byte(cmd, value, true); 
  
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
  if(ret != ESP_OK){
    printf("writeReg failed with error: %s", esp_err_to_name(ret));
  }

  i2c_cmd_link_delete(cmd);
}

uint8_t readReg(LPS sensor, int reg){
  uint8_t value;

  if(reg < 0){
    reg = sensor.translated_regs[-reg];
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (sensor.address << 1) | I2C_MASTER_WRITE, true);

  i2c_master_write_byte(cmd, reg, true);

  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (sensor.address << 1) | I2C_MASTER_READ, true);

  i2c_master_read_byte(cmd, &value, I2C_MASTER_NACK);

  i2c_master_stop(cmd);
  
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);

  if(ret != ESP_OK){
    printf("readReg failed with error: %s\n", esp_err_to_name(ret));
    return 0xFF;
  }

  i2c_cmd_link_delete(cmd);

  return value;
}

float readPressureMillibars(LPS sensor){
  return (float)readPressureRaw(sensor) / 4096;
}

float readPressureInchesHg(LPS sensor){
  return (float)readPressureRaw(sensor) / 138706.5;
}

int32_t readPressureRaw(LPS sensor){
  uint8_t pxl, pl, ph;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (sensor.address << 1) | I2C_MASTER_WRITE, true);
  
  if(sensor._device == device_25H || sensor._device == device_331AP){
    i2c_master_write_byte(cmd, (PRESS_OUT_XL | (1 << 7)), true);
  }else {
    i2c_master_write_byte(cmd, PRESS_OUT_XL, true);
  }
  
  i2c_master_stop(cmd);
  
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  
  i2c_cmd_link_delete(cmd);
  
  if(ret != ESP_OK){
    printf("readPressureRaw [write] failed with error: %s\n", esp_err_to_name(ret));
    return 0xFF;
  }
  
  cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (sensor.address << 1) | I2C_MASTER_READ, true);
  
  i2c_master_read_byte(cmd, &pxl, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &pl, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &ph, I2C_MASTER_NACK);

  i2c_master_stop(cmd);

  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  
  i2c_cmd_link_delete(cmd);

  if(ret != ESP_OK){
    printf("readPressureRaw [read] failed with error: %s\n", esp_err_to_name(ret));
    return 0xFF;
  }
  
  return (int32_t)(int8_t)ph << 16 | (uint16_t)pl << 8 | pxl;
}

float readTemperatureC(LPS sensor){
  if(sensor._device == device_22DF){
    return (float)readTemperatureRaw(sensor) / 100;
  }else if(sensor._device == device_25H || sensor._device == device_331AP){
    return 42.5 + (float)readTemperatureRaw(sensor) / 480;
  }
  return 0.0;
}

float readTemperatureF(LPS sensor){
  if(sensor._device == device_22DF){
    return 32 + (float)readTemperatureRaw(sensor) / 100 * 1.8;
  }else if(sensor._device == device_25H || sensor._device == device_331AP){
    return 108.5 + (float)readTemperatureRaw(sensor) / 480 * 1.8;
  }
  return 0.0;
}

int16_t readTemperatureRaw(LPS sensor){
  uint8_t tl, th;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (sensor.address << 1) | I2C_MASTER_WRITE, true);

  if(sensor._device == device_25H || sensor._device == device_331AP){
    i2c_master_write_byte(cmd, TEMP_OUT_L | (1 << 7), true);
  }else {
    i2c_master_write_byte(cmd, TEMP_OUT_L, true);
  }

  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  
  i2c_cmd_link_delete(cmd);

  if(ret != ESP_OK){
    printf("readTemperatureRaw [write] failed with error: %s\n", esp_err_to_name(ret));
    return 0xFF;
  }

  cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (sensor.address << 1) | I2C_MASTER_READ, true);
  
  i2c_master_read_byte(cmd, &tl, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &th, I2C_MASTER_NACK);

  i2c_master_stop(cmd);

  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);

  if(ret != ESP_OK){
    printf("readTemperatureRaw [read] failed with error: %s\n", esp_err_to_name(ret));
    return 0xFF;
  }

  i2c_cmd_link_delete(cmd);

  return (int16_t)(th << 8 | tl);
}

float pressureToAltitudeMeters(float pressure_mbar){
  return (1 - pow(pressure_mbar / 1013.25, 0.190263)) * 44330.8;
}

float pressureToAltitudeFeet(float pressure_inHg){
  return (1 - pow(pressure_inHg / 29.9213, 0.190263)) * 145442;
}

bool detectDeviceAndAddress(LPS *sensor, deviceType device, sa0State sa0){
  if(sa0 == sa0_auto || sa0 == sa0_high){
    sensor->address = SA0_HIGH_ADDRESS;
    if(detectDevice(sensor, device)) return true;
  }
  if(sa0 == sa0_auto || sa0 == sa0_low){
    sensor->address = SA0_LOW_ADDRESS;
    if(detectDevice(sensor, device)) return true;
  }

  return false;
}

bool detectDevice(LPS *sensor, deviceType device){
  int id = testWhoAmI(sensor->address);

  if((device == device_auto || device == device_22DF) && id == LPS22DF_WHO_ID){
    sensor->_device = device_22DF;
    return true;
  }
  if((device == device_auto || device == device_25H) && id == LPS25H_WHO_ID){
    sensor->_device = device_25H;
    return true;
  }
  if((device == device_auto || device == device_331AP) && id == LPS331AP_WHO_ID){
    sensor->_device = device_331AP;
    return true;
  }
  return false;
}

int testWhoAmI(uint8_t address){
  uint8_t reg = WHO_AM_I;
  uint8_t value; 

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);

  i2c_master_write_byte(cmd, reg, true);

  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);

  i2c_master_read_byte(cmd, &value, I2C_MASTER_NACK);

  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  
  i2c_cmd_link_delete(cmd);

  if(ret != ESP_OK){
    printf("testWhoAmI failed with error: %s\n", esp_err_to_name(ret));
    return 0xFF;
  }

  return value;
}

deviceType getDeviceType(LPS sensor){
  return sensor._device;
}

uint8_t getAddress(LPS sensor){
  return sensor.address;
}


