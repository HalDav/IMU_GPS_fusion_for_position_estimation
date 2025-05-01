# IMU_GPS_fusion_for_position_estimation
This project is designed to read data from the Parker 3DM-CV7 IMU and u-blox NEO-7M GPS module. Using this data, it estimates the device's position (latitude, longitude, altitude) and orientation (pitch, roll, yaw) of device.

Platform: 
- Arduino MEGA

Used modules: 
- Parker 3DM-CV7 Inertial measurement unit connected to +5V, GND, Rx2, Tx2 of Arduino MEGA
- u-blox NEO-7M GPS module connected to +3.3V, GND, Rx3, Tx3 of Arduino MEGA

Libraries:
- TinyGPS.h
- Kalman.h

Function descriptions:
readMessage();
- In this function serial2 is being checked. Algorithm is looking for SYNC1 and SYNC2 bytes of IMU message. When succesfull, rest of the message is saved in message array.
  
checksumCheck();
- When IMU message is succesfully catched and stored in message array, checksum function is being performed based on Fletcher16 checksum algorithm. Since SNC1 and SYNC2 bytes are not stored in message array, but the value is always the same, are their values
brougth into checksum automatically. They every byte of message is added into calculated coeficients based on recently mention Fletcher 16 algorithm. When all bytes (except two checksum bytes) are added, modulo 255 is performed for both coeficients. Results of     these two coeficients are compared with IMU checksum bytes. If values are the same, message was received correctly.
  
floatValues();
- Every sensor has 3 axes in which it measures. Data are being sent in 3 x 4 bytes (4 bytes for x-axis, 4 bytes for y-axis and 4 bytes for z-axis). Here it is necessary to used SensorConnect application and check for order in which data are sent. This code expect data in accelerometer (x, y, z), gyroscope (x, y, z) and magnetometer (x, y, z). IMU has option to change this order manually in their app. If sensor data are transmitted in different order, calculated values won´t make any sense, so check this for your IMU and change order to corespond with earlier mentioned if neccesaray. This function takes 4 bytes stored in massaged and convert them into float.
  
GPSread();
- This function uses TinyGPS library to receive and store data into relevant variables.
  
positionEstimation();
- Here pitch, roll and yaw are calculated. At first program decides if gyro and accelerometer errors are calculated. If not, error calculation cycle is done. Program takes first 50 IMU measurements and calculates average value of sensors measurements while device is not moving. These error values are later on used in calculations. For every sensor program calculates it´s eastimation on its own. For pitch accelerometer and gyroscope is used as well for roll. For yaw acclerometer cannot be used, thats why magnetometer is used instead. Also gyroscope is unable to start estimate on it´s own, that why it gets initial value from accelerometer or magnetometer. Another problem with gyro calculations is when next estimations exceeds 0-360 degree interval. When this happens estimated values is 360 is subtracted from estimated value. On the other hand, when last estimated vlaue is near 0, sometimes next estimation can be in negative values - here 360 is added to estimation.
  
kalman();
imuLocationEstimation();
valuesOut();
Notes:
