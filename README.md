# IMU_GPS_fusion_for_position_estimation
This project is designed to read data from the Parker 3DM-CV7 IMU and u-blox NEO-7M GPS module. Using this data, it estimates the device's position (latitude, longitude, altitude) and orientation (pitch, roll, yaw) of device.

Platform: Arduino MEGA

Used modules: Parker 3DM-CV7 Inertial measurement unit
              
              u-blox NEO-7M GPS module
              
Wiring:

Libraries:

readMessage();
- In this function serial2 is being checked. Algorithm is looking for SYNC1 and SYNC2 bytes of IMU message. When succesfull, rest of the message is saved in message array.
  
checksumCheck();
- When IMU message is succesfully catched and stored in message array, checksum function is being performed based on Fletcher16 checksum algorithm. Since SNC1 and SYNC2 bytes are not stored in message array, but the value is always the same, are their values
  brougth into checksum automatically. They every byte of message is added into calculated coeficients based on recently mention Fletcher 16 algorithm. When all bytes (except two checksum bytes) are added, modulo 255 is performed for both coeficients. Results of     these two coeficients are compared with IMU checksum bytes. If values are the same, message was received correctly.
  
floatValues();
- Every sensor has 3 axes in which it measures. Data are being sent in 3 x 4 bytes (4 bytes for x-axis, 4 bytes for y-axis and 4 bytes for z-axis). Here it is necessary to used SensorConnect application and check for order in which data are sent. This code expect    data in accelerometer (x, y,z), gyroscope (x, y, z) and magnetometer (x, y, z). IMU has option to change this order manually in their app. If sensor data are transmitted in different order, calculated values won´t make any sense, so check this for your IMU         and if neccesaray, change order to corespond with earlier mentioned. This function takes 4 bytes stored in massaged and convert them into float.
  
GPSread();

positionEstimation();
kalman();
imuLocationEstimation();
valuesOut();
Notes:
