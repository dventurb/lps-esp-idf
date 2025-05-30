#include <stdbool.h>
#include "lps.h"
#include "driver/i2c.h"

#define SCL_IO 22 
#define SDA_IO 21

// I2C
void i2c_master_init(void);

// LPS 
LPS sensor;

void app_main(){
 i2c_master_init();
  
 if(!LPS_init(&sensor)){
    printf("LPS_init failed to detetect.\n");
  }
  LPS_enable(sensor);

  while(1){
    
    float pressure, altitude, temperature; 
    
    pressure = readPressureInchesHg(sensor);
    altitude = pressureToAltitudeFeet(pressure);
    temperature = readTemperatureF(sensor);
    
    printf("Pressure: %f\n", pressure);
    printf("Altitude: %f\n", altitude);
    printf("Temperature: %f\n\n", temperature);
    
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

/* ------------------------------ I2C ------------------------------ */

void i2c_master_init(){
  int i2c_master_port = I2C_NUM_0;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000;
  conf.clk_flags = 0;

  i2c_param_config(i2c_master_port, &conf);

  i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

