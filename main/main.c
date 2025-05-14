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

#define SENSOR_ADDR                 0x68

#define REG_ADD_WIA 0x00
#define REG_VAL_WIA 0xEA
#define REG_ADD_USER_CTRL 0x03
#define REG_VAL_BIT_DMP_EN 0x80
#define REG_VAL_BIT_FIFO_EN 0x40
#define REG_VAL_BIT_I2C_MST_EN 0x20
#define REG_VAL_BIT_I2C_IF_DIS 0x10
#define REG_VAL_BIT_DMP_RST 0x08
#define REG_VAL_BIT_DIAMOND_DMP_RST 0x04
#define REG_ADD_PWR_MIGMT_1 0x06
#define REG_VAL_ALL_RGE_RESET 0x80
#define REG_VAL_RUN_MODE 0x01 //Non low-power mode
#define REG_ADD_LP_CONFIG 0x05
#define REG_ADD_PWR_MGMT_1 0x06
#define REG_ADD_PWR_MGMT_2 0x07
#define REG_ADD_ACCEL_XOUT_H 0x2D
#define REG_ADD_ACCEL_XOUT_L 0x2E
#define REG_ADD_ACCEL_YOUT_H 0x2F
#define REG_ADD_ACCEL_YOUT_L 0x30
#define REG_ADD_ACCEL_ZOUT_H 0x31
#define REG_ADD_ACCEL_ZOUT_L 0x32
#define REG_ADD_GYRO_XOUT_H 0x33
#define REG_ADD_GYRO_XOUT_L 0x34
#define REG_ADD_GYRO_YOUT_H 0x35
#define REG_ADD_GYRO_YOUT_L 0x36
#define REG_ADD_GYRO_ZOUT_H 0x37
#define REG_ADD_GYRO_ZOUT_L 0x38
#define REG_ADD_EXT_SENS_DATA_00 0x3B
#define REG_ADD_REG_BANK_SEL 0x7F
#define REG_VAL_REG_BANK_0 0x00
#define REG_VAL_REG_BANK_1 0x10
#define REG_VAL_REG_BANK_2 0x20
#define REG_VAL_REG_BANK_3 0x30

/* user bank 1 register */
/* user bank 2 register */
#define REG_ADD_GYRO_SMPLRT_DIV 0x00
#define REG_ADD_GYRO_CONFIG_1 0x01
#define REG_VAL_BIT_GYRO_DLPCFG_2 0x10   /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_4 0x20   /* bit[5:3] */
#define REG_VAL_BIT_GYRO_DLPCFG_6 0x30   /* bit[5:3] */
#define REG_VAL_BIT_GYRO_FS_250DPS 0x00  /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_500DPS 0x02  /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_1000DPS 0x04 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_FS_2000DPS 0x06 /* bit[2:1] */
#define REG_VAL_BIT_GYRO_DLPF 0x01       /* bit[0]   */
#define REG_ADD_ACCEL_SMPLRT_DIV_2 0x11
#define REG_ADD_ACCEL_CONFIG 0x14
#define REG_VAL_BIT_ACCEL_DLPCFG_2 0x10 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_4 0x20 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_DLPCFG_6 0x30 /* bit[5:3] */
#define REG_VAL_BIT_ACCEL_FS_2g 0x00    /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_4g 0x02    /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_8g 0x04    /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_FS_16g 0x06   /* bit[2:1] */
#define REG_VAL_BIT_ACCEL_DLPF 0x01     /* bit[0]   */

/* user bank 3 register */
#define REG_ADD_I2C_SLV0_ADDR 0x03
#define REG_ADD_I2C_SLV0_REG 0x04
#define REG_ADD_I2C_SLV0_CTRL 0x05
#define REG_VAL_BIT_SLV0_EN 0x80
#define REG_VAL_BIT_MASK_LEN 0x07
#define REG_ADD_I2C_SLV0_DO 0x06
#define REG_ADD_I2C_SLV1_ADDR 0x07
#define REG_ADD_I2C_SLV1_REG 0x08
#define REG_ADD_I2C_SLV1_CTRL 0x09
#define REG_ADD_I2C_SLV1_DO 0x0A

/* define ICM-20948 Register  end */

/* define ICM-20948 MAG Register  */
#define REG_ADD_MAG_WIA1 0x00
#define REG_VAL_MAG_WIA1 0x48
#define REG_ADD_MAG_WIA2 0x01
#define REG_VAL_MAG_WIA2 0x09
#define REG_ADD_MAG_ST2 0x10
#define REG_ADD_MAG_DATA 0x11
#define REG_ADD_MAG_CNTL2 0x31
#define REG_VAL_MAG_MODE_PD 0x00
#define REG_VAL_MAG_MODE_SM 0x01
#define REG_VAL_MAG_MODE_10HZ 0x02
#define REG_VAL_MAG_MODE_20HZ 0x04
#define REG_VAL_MAG_MODE_50HZ 0x05
#define REG_VAL_MAG_MODE_100HZ 0x08
#define REG_VAL_MAG_MODE_ST 0x10
/* define ICM-20948 MAG Register  end */


#define FIFO_EN_1         			0x66
#define FIFO_EN_2         			0x67
#define FIFO_COUNTH       			0x70
#define FIFO_COUNTL       			0x71
#define FIFO_R_W          			0x72

#define ACCEL_GYRO_FIFO_FRAME_SIZE 12 // 6 value × 2 octets (accel + gyro)

int16_t accel[3], gyro[3];
int16_t gstGyros16X, gstGyros16Y, gstGyros16Z;

static const char *TAG = "i2c_example";

typedef struct icm20948_st_avg_data_tag
	{
	  uint8_t u8Index;
		int16_t s16AvgBuffer[8];
	}ICM20948_ST_AVG_DATA;
	
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

void select_bank(uint8_t bank) {
    i2c_write_byte(SENSOR_ADDR, REG_ADD_REG_BANK_SEL, bank);
}

void enable_fifo() {
    select_bank(REG_VAL_REG_BANK_0);
	
    // Reset FIFO + Enable FIFO
    i2c_write_byte(SENSOR_ADDR, REG_ADD_USER_CTRL, 0x44); // FIFO_EN | FIFO_RST
    vTaskDelay(pdMS_TO_TICKS(10)); // delay after reset
    //FiFO Mode
    i2c_write_byte(SENSOR_ADDR, 0x69, 0x00); //Mode Stream
    
	//Slave FIFO Enable
	//i2c_write_byte(SENSOR_ADDR, FIFO_EN_1, 0b1111);
    // Activate accel and gyro (XYZ) in FIFO
    i2c_write_byte(SENSOR_ADDR, FIFO_EN_2, 0x1E); // ACCEL_FIFO_EN + GYRO_X/Y/Z_EN --> Write accel and Gyro to FIFO

}
void icm20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{ 
  uint8_t i;
  
  *(pAvgBuffer + ((*pIndex) ++)) = InVal;
    *pIndex &= 0x07;
    
    *pOutVal = 0;
  for(i = 0; i < 8; i ++) 
    {
      *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
}

bool read_fifo_sample(int16_t *accel_xyz, int16_t *gyro_xyz) {
    uint8_t count_h, count_l;
    i2c_read_byte(SENSOR_ADDR, FIFO_COUNTH, &count_h);
    i2c_read_byte(SENSOR_ADDR, FIFO_COUNTL, &count_l);
    uint16_t fifo_count = (count_h << 8) | count_l;

    if (fifo_count < ACCEL_GYRO_FIFO_FRAME_SIZE) return false;

    // Burst read all FIFO data at once
    uint8_t raw_data[ACCEL_GYRO_FIFO_FRAME_SIZE];
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, FIFO_R_W, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, ACCEL_GYRO_FIFO_FRAME_SIZE, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    // Parse data (accel X/Y/Z → gyro X/Y/Z)
    for (int i = 0; i < 3; i++) {
        accel_xyz[i] = (raw_data[i*2] << 8) | raw_data[i*2 + 1];
        gyro_xyz[i]  = (raw_data[6 + i*2] << 8) | raw_data[6 + i*2 + 1];
    }
	
    int32_t s32OutBuf[3] = {0};
	int32_t s32OutBuf1[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
	static ICM20948_ST_AVG_DATA sstAvgBuf1[3];

	for(uint8_t i = 0; i < 3; i ++) 
	    {
	        icm20948CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, gyro_xyz[i], s32OutBuf + i);
	    }
	gyro_xyz[0]= s32OutBuf[0] -gstGyros16X;
	gyro_xyz[1]= s32OutBuf[1] -gstGyros16Y;
	gyro_xyz[2]= s32OutBuf[2] -gstGyros16X;
	
	for(uint8_t i = 0; i < 3; i ++) 
	    {
	        icm20948CalAvgValue(&sstAvgBuf1[i].u8Index, sstAvgBuf1[i].s16AvgBuffer, accel_xyz[i], s32OutBuf1 + i);
	    }
		
	accel_xyz[0] = s32OutBuf1[0];
	accel_xyz[1] = s32OutBuf1[1];
	accel_xyz[2] = s32OutBuf1[2];
    return true;
}

void icm20948GyroOffset(void)
{
  uint8_t i;
  int16_t accel_0[3], gyro_0[3];
  int32_t s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
  for(i = 0; i < 32; i ++)
  {
    read_fifo_sample(accel_0, gyro_0);
    s32TempGx += gyro[0];
    s32TempGy += gyro[1];
    s32TempGz += gyro[2];
	vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  gstGyros16X = s32TempGx >> 5;
  gstGyros16Y = s32TempGy >> 5;
  gstGyros16Z = s32TempGz >> 5;
  return;
}



void app_main()
{
    i2c_master_init();
    ESP_LOGI(TAG, "I2C Initialized");
	
	/// who i am
	i2c_write_byte(SENSOR_ADDR, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);  //Set BANK 0
	
    uint8_t data;
    esp_err_t ret = i2c_read_byte(SENSOR_ADDR, REG_ADD_WIA, &data); //data = 0xea
	
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Data from sensor: 0x%02X", data);
        printf("Lecture OK: 0x%02X\n", data);
    } else {
        ESP_LOGE(TAG, "Failed to read from sensor: %s", esp_err_to_name(ret));
    }
    
    ///// RESET
    
    // Wake up + Autoclock
    i2c_write_byte(SENSOR_ADDR, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0); //select bank 0
    i2c_write_byte(SENSOR_ADDR, REG_ADD_PWR_MGMT_1, REG_VAL_ALL_RGE_RESET);
	vTaskDelay(pdMS_TO_TICKS(100));
	i2c_write_byte(SENSOR_ADDR, REG_ADD_PWR_MGMT_1, REG_VAL_RUN_MODE);

    
    //Gyro + Accel config
    select_bank(REG_VAL_REG_BANK_2);
	i2c_write_byte(SENSOR_ADDR, REG_ADD_GYRO_SMPLRT_DIV, 0x07);
	i2c_write_byte(SENSOR_ADDR, REG_ADD_GYRO_CONFIG_1, 	REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_1000DPS | REG_VAL_BIT_GYRO_DLPF);
	
	i2c_write_byte(SENSOR_ADDR, REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07);
	i2c_write_byte(SENSOR_ADDR, REG_ADD_ACCEL_CONFIG, REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);
    select_bank(REG_VAL_REG_BANK_0);
    
	vTaskDelay(pdMS_TO_TICKS(100));
	
    enable_fifo();
	  
	    
    while(1)
    {
		  
		
    	uint64_t timestamp_ms = esp_timer_get_time() / 1000;
    	if (read_fifo_sample(accel, gyro)) {
    		printf("%llu %d %d %d %d %d %d\n", timestamp_ms, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
    	    }
    	vTaskDelay(pdMS_TO_TICKS(100));

	}

}

