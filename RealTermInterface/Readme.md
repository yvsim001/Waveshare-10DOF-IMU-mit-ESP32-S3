# Orientation3Dcube

This MATLAB project visualizes the orientation of a 3D cube in real-time, using data from an external IMU sensor (via a serial COM port). The orientation is calculated from the accelerometer readings and is displayed both as a rotating cube and in text (Roll, Pitch, Yaw values).

## 💡 Features

- Real-time communication with a serial IMU sensor (UART).
- Live 3D cube representing the orientation of the sensor.
- Estimation of roll and pitch from accelerometer data.
- Quaternion-based rotation of the 3D cube.
- Optional yaw (set to zero here, as it requires a magnetometer or gyro integration).
- Stop button and clean shutdown when figure is closed.

## 🔧 How It Works

1. **Serial Connection**  
   The script connects to a specified COM port (e.g., COM3) and listens for incoming data in the format:

+ timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z

2. **Orientation Estimation**  
Only accelerometer data is used to compute:
- **Roll** = atan2(ay, az)
- **Pitch** = atan2(-ax, sqrt(ay² + az²))
- **Yaw** = 0 (placeholder)

3. **Rotation Using Quaternions**  
The computed angles are converted to a quaternion and used to rotate a virtual cube that represents the sensor’s orientation in 3D space.

4. **3D Visualization**  
A cube is drawn using MATLAB’s `patch` and updated at each iteration with the rotated vertices.

## 📦 Requirements

- MATLAB with 3D plotting support
- Serial device (IMU) sending 7 values per line over COM port
- `OpenCOMport` and `GetSerialData` helper functions
- Optional: Custom fonts, accelerometer calibration

## 🖥️ How to Run

1. Connect your IMU device to your PC via USB.
2. Modify the script to match the correct `ComPortNumber`.
3. Run `Orientation3Dcube` in MATLAB.
4. The cube should move according to your sensor’s orientation.

## 🛑 Stopping the Program

- Press the **Stop** button in the figure window, or
- Simply close the figure.

## 🔄 Next Steps / Suggestions

- Add yaw estimation using gyroscope and/or magnetometer.
- Implement a complementary filter or Kalman filter for better accuracy.
- Add sensor calibration step.

---
