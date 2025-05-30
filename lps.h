#ifndef LPS_H 
#define LPS_H

#include <stdbool.h>
#include <stdint.h>

#define dummy_reg_count 13

typedef enum {
  device_331AP, 
  device_25H,
  device_22DF,
  device_auto
} deviceType;

typedef enum { 
  sa0_low,
  sa0_high,
  sa0_auto
} sa0State;

typedef enum {
      REF_P_XL                = 0x08,
      IF_CTRL                 = 0x0E,
      WHO_AM_I                = 0x0F,
      RES_CONF                = 0x10,
      FIFO_WTM                = 0x15, // 22DF
      I3C_IF_CTRL_ADD         = 0x19, // 22DF
      FIFO_STATUS1            = 0x25, // 22DF
      FIFO_STATUS2            = 0x26, // 22DF
      STATUS_REG              = 0x27,
      PRESS_OUT_XL            = 0x28,
      PRESS_OUT_L             = 0x29,
      PRESS_OUT_H             = 0x2A,
      TEMP_OUT_L              = 0x2B,
      TEMP_OUT_H              = 0x2C,
      FIFO_STATUS             = 0x2F, // 25H
      AMP_CTRL                = 0x30, // 331AP
      DELTA_PRESS_XL          = 0x3C, // 331AP
      DELTA_PRESS_L           = 0x3D, // 331AP
      DELTA_PRESS_H           = 0x3E, // 331AP
      FIFO_DATA_OUT_PRESS_XL  = 0x78, 
      FIFO_DATA_OUT_PRESS_L   = 0x79, 
      FIFO_DATA_OUT_PRESS_H   = 0x7A, 

      REF_P_L          = -1,
      REF_P_H          = -2,
      CTRL_REG1        = -3,
      CTRL_REG2        = -4,
      CTRL_REG3        = -5,
      CTRL_REG4        = -6,
      INTERRUPT_CFG    = -7,
      INT_SOURCE       = -8,
      FIFO_CTRL        = -9,
      THS_P_L          = -10,
      THS_P_H          = -11,
      RPDS_L           = -12,
      RPDS_H           = -13,
      // update dummy_reg_count if registers are added here!

      // device-specific register addresses

      LPS331AP_REF_P_L        = 0x09,
      LPS331AP_REF_P_H        = 0x0A,
      LPS331AP_CTRL_REG1      = 0x20,
      LPS331AP_CTRL_REG2      = 0x21,
      LPS331AP_CTRL_REG3      = 0x22,
      LPS331AP_INTERRUPT_CFG  = 0x23,
      LPS331AP_INT_SOURCE     = 0x24,
      LPS331AP_THS_P_L        = 0x25,
      LPS331AP_THS_P_H        = 0x26,

      LPS25H_REF_P_L          = 0x09,
      LPS25H_REF_P_H          = 0x0A,
      LPS25H_CTRL_REG1        = 0x20,
      LPS25H_CTRL_REG2        = 0x21,
      LPS25H_CTRL_REG3        = 0x22,
      LPS25H_CTRL_REG4        = 0x23,
      LPS25H_INTERRUPT_CFG    = 0x24,
      LPS25H_INT_SOURCE       = 0x25,
      LPS25H_FIFO_CTRL        = 0x2E,
      LPS25H_THS_P_L          = 0x30,
      LPS25H_THS_P_H          = 0x31,
      LPS25H_RPDS_L           = 0x39,
      LPS25H_RPDS_H           = 0x3A,

      LPS22DF_INTERRUPT_CFG   = 0x0B,
      LPS22DF_THS_P_L         = 0x0C,
      LPS22DF_THS_P_H         = 0x0D,
      LPS22DF_CTRL_REG1       = 0x10,
      LPS22DF_CTRL_REG2       = 0x11,
      LPS22DF_CTRL_REG3       = 0x12,
      LPS22DF_CTRL_REG4       = 0x13,
      LPS22DF_FIFO_CTRL       = 0x14,
      LPS22DF_REF_P_L         = 0x16,
      LPS22DF_REF_P_H         = 0x17,
      LPS22DF_RPDS_L          = 0x1A,
      LPS22DF_RPDS_H          = 0x1B,
      LPS22DF_INT_SOURCE      = 0x24,
} regAddr;

typedef struct {
  deviceType _device;
  sa0State sa0;
  uint8_t address;
  regAddr translated_regs[dummy_reg_count + 1];
} LPS;

bool LPS_init(LPS *sensor);
void LPS_enable(LPS sensor);
void writeReg(LPS sensor, int reg, uint8_t value);

uint8_t readReg(LPS sensor, int reg);
float readPressureMillibars(LPS sensor);

float readPressureInchesHg(LPS sensor);

int32_t readPressureRaw(LPS sensor);
float readTemperatureC(LPS sensor);

float readTemperatureF(LPS sensor);
int16_t readTemperatureRaw(LPS sensor);
float pressureToAltitudeMeters(float pressure_mbar);
float pressureToAltitudeFeet(float pressure_inHg);

bool detectDeviceAndAddress(LPS *sensor, deviceType device, sa0State sa0);
bool detectDevice(LPS *sensor, deviceType device);
int testWhoAmI(uint8_t address);
uint8_t getAddress(LPS sensor);

#endif
