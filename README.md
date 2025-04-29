# Waveshare 10DOF IMU mit ESP32-S3

## Projektbeschreibung
Dieses Projekt beschreibt die Integration des Waveshare 10DOF IMU-Moduls mit einem ESP32-S3-Mikrocontroller. Ziel ist es, Sensordaten von Beschleunigung, Gyroskop, Magnetometer und Barometer auszulesen und zu analysieren.

## Hardware
# ESP32-S3 ICM-20948 Motion Tracking

![ICM-20948 Block Diagram](https://invensense.tdk.com/wp-content/uploads/2021/01/ICM-20948-Block-Diagram.png)

A minimal ESP-IDF project for interfacing with the ICM-20948 9-axis IMU (accelerometer, gyroscope, magnetometer) using I²C on ESP32-S3.

## Features
- ✅ 9-axis motion tracking (3x accelerometer, 3x gyroscope, 3x magnetometer)
- ✅ Digital Motion Processor (DMP) for sensor fusion
- ✅ I²C interface (400kHz Fast Mode)
- ✅ Configurable Full-Scale Ranges (FSR)
- ✅ Low power operation (down to 1.71V)
- ✅ ESP-IDF v5.x compatible

## Hardware Connections

| ICM-20948 Pin | ESP32-S3 Pin | Note                     |
|---------------|-------------|--------------------------|
| VDD           | 3.3V        | Power supply             |
| GND           | GND         | Ground                   |
| SCL           | GPIO 9      | I²C Clock                |
| SDA           | GPIO 46     | I²C Data                 |

**Important Notes:**
- Requires 4.7kΩ pull-up resistors on SDA/SCL lines
- VDDIO must be 1.71V-1.95V (use level shifter if ESP32 runs at 3.3V)
- For SPI mode, additional pins would be needed

## Software Setup

### Prerequisites
- ESP-IDF v5.x
- ICM-20948 driver (included in components/)

### Build and Flash
```bash
git clone https://github.com/yvsim001/10DOF_PICO.git
cd esp32-icm20948
idf.py set-target esp32s3
idf.py build flash monitor
```

## Usage
```c
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
}
```
# Output

```c
uint64_t timestamp_ms = esp_timer_get_time() / 1000;
		printf("%llu %d %d %d %d %d %d\n", timestamp_ms, accel_X, accel_Y, accel_Z, gyro_X, gyro_Y, gyro_Z);
		vTaskDelay(pdMS_TO_TICKS(100));
```



### ESP-IDF Components library

[Read This](icm20948/README.md)

### ICM20948 Library

[ICM20948](https://github.com/listout/icm20948_driver/tree/master/components/icm20948)

[ICM20948 (2)](https://github.com/ashkorehennessy/icm20948_esp/tree/main/components/icm20948)
