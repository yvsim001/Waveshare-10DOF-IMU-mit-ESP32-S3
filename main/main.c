#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdio.h>
#include "esp_timer.h"

#define I2C_MASTER_NUM              I2C_NUM_0   // ou I2C_NUM_1
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           46
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define SENSOR_ADDR                 0x68 // ← Remplace par l'adresse de ton appareil

static const char *TAG = "i2c_example";

void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true); // Adresse + Write
    i2c_master_write_byte(cmd, reg_addr, true);                              // Registre cible
    i2c_master_write_byte(cmd, data, true);                                  // Donnée à écrire
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true); // Adresse + Write
    i2c_master_write_byte(cmd, reg_addr, true);                              // Registre à lire
    i2c_master_start(cmd);                                                   // Restart
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);  // Adresse + Read
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);                   // Lire 1 octet
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}


void app_main()
{
    i2c_master_init();
    ESP_LOGI(TAG, "I2C Initialized");
	
	/// who i am
	i2c_write_byte(0x68, 0x7f, 0x00);  //Set BANK 0
	
    uint8_t data;
    esp_err_t ret = i2c_read_byte(0x68, 0x00, &data); //data = 0xea
	
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Data from sensor: 0x%02X", data);
        printf("Lecture OK: 0x%02X\n", data);
    } else {
        ESP_LOGE(TAG, "Failed to read from sensor: %s", esp_err_to_name(ret));
    }
    
    ///// RESET
    
    //i2c_write_byte(0x68, 0x7f, 0x00); //select bank 0
    //i2c_write_byte(0x68, 0x06, 0x80); //REset
    
    
    
	  
	    
    while(1)
    {
		  
		// Wake up + Autoclock
    
    	i2c_write_byte(0x68, 0x06, 0x01);
        
	    // Gyro + Accel config
	    //i2c_write_byte(0x68, 0x7f, 0x20); //select bank 2
	    
	    //i2c_write_byte(0x68, 0x01, 0x00); // set gyro scale to +/- 250dps
	    
	    //i2c_write_byte(0x68, 0x14, 0x00); // set accel scale to +/- 2g
	    
	    /// Read Data 
	    //i2c_write_byte(0x68, 0x7f, 0x00); //select bank 0
		uint8_t accel_XOUTH , accel_XOUTL, accel_YOUTH , accel_YOUTL, accel_ZOUTH , accel_ZOUTL, gyro_XOUTH , gyro_XOUTL, gyro_YOUTH , gyro_YOUTL, gyro_ZOUTH , gyro_ZOUTL;
	    
	    i2c_read_byte(0x68, 0x2D,  &accel_XOUTH); 
	    i2c_read_byte(0x68, 0x2E,  &accel_XOUTL);
	    i2c_read_byte(0x68, 0x2F,  &accel_YOUTH); 
	    i2c_read_byte(0x68, 0x30,  &accel_YOUTL);
	    i2c_read_byte(0x68, 0x31,  &accel_ZOUTH); 
	    i2c_read_byte(0x68, 0x32, &accel_ZOUTL);
	  	// Combine bytes (signed 16-bit)
		uint16_t accel_X = (accel_XOUTH << 8) | accel_XOUTL;
		uint16_t accel_Y = (accel_YOUTH << 8) | accel_YOUTL;
		uint16_t accel_Z = (accel_ZOUTH << 8) | accel_ZOUTL;
		
		//printf("Acceleration Value: X = %d, Y = %d, Z = %d\n", accel_X , accel_Y, accel_Z);
	    i2c_read_byte(0x68, 0x33, &gyro_XOUTH);
		i2c_read_byte(0x68, 0x34, &gyro_XOUTL);
		i2c_read_byte(0x68, 0x35, &gyro_YOUTH);
		i2c_read_byte(0x68, 0x36, &gyro_YOUTL);
		i2c_read_byte(0x68, 0x37, &gyro_ZOUTH);
		i2c_read_byte(0x68, 0x38, &gyro_ZOUTL);
		
		// Combinaison des valeurs hautes et basses
		uint16_t gyro_X = (gyro_XOUTH << 8) | gyro_XOUTL;
		uint16_t gyro_Y = (gyro_YOUTH << 8) | gyro_YOUTL;
		uint16_t gyro_Z = (gyro_ZOUTH << 8) | gyro_ZOUTL;
		
		//printf("Gyroscope Value: X = %d, Y = %d, Z = %d\n", gyro_X, gyro_Y, gyro_Z);
		uint64_t timestamp_ms = esp_timer_get_time() / 1000;
		printf("%llu %d %d %d %d %d %d\n", timestamp_ms, accel_X, accel_Y, accel_Z, gyro_X, gyro_Y, gyro_Z);
		vTaskDelay(pdMS_TO_TICKS(100));
		
	}
	    
}