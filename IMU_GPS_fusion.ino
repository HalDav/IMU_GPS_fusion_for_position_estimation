/**********************************************************************************************************************************
* Project:    IMU and GPS position (pitch, roll, yaw) and location estimation SW
*
* Author:     HalDav
* Platform:   Arduino MEGA
*
*
* Used modules: 
*   - Parker 3DM-CV7 IMU (connected to +5V, GND, Tx2, Rx2 of Arduino)
*   - u-blox NEO-7M (connected to +3.3V, GND, Tx3, Rx3 of Arduino)
*
*
* Tasks:
*   - Receiving and decoding data from Parker IMU - main focus
*   - Developing algorithm for position and location estimation based on IMU data - main focus
*   - Receiving data from GPS - library used
*
*
* Notes:
*   This was my first more complex project in Arduino. It was mostly focused on receiving data from IMU and working with them. Since there is no Arduino library for working
*   with Parker IMU (at least not in time of developing this program), it was necessary to develop an algorithm based on manufacturer documentation to acquire data. In position estimation own algorithms were created, 
*   based on rotational matrix, to calculate orientation in space of device. The same goes for location estimation. 
*/


//Libraries
#include <TinyGPS.h>
TinyGPS gps; // create gps object
#include <Kalman.h>
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

//Message variables, IMU variables
int dtime = 6;    // Lowest delay values 6 ms. Anything below has big error in data tranmission 
const unsigned int MAX_MESSAGE_LENGTH = 60;
static unsigned message[MAX_MESSAGE_LENGTH];

long c1_old, c1_new, c2_old, c2_new, c1_chcksm, c2_chcksm; //checksum calculation coeficients (Fletcher-16)
int mess_c1, mess_c2; //for reading checksum bytes from message
int j, k;

bool newData; 
bool checksumOK; 
bool error;
bool sensorErrorCalculated;
bool pitchRoll180 = true;  //Here you can decide if you want final values in 0 - 360 or +-180 range. Heading is always 0 - 360. You can change this variable to false, if you want 0 - 360 degrees.
bool pitchNumber;
bool imuLocationSetUp;

//GPS variables
float lat,lon, gps_speed, gps_course, gps_altitude, gps_distance, gps_totalDistanceTraveled;
unsigned long time, date;

//Sensor error variables
float gyrox_error;
float gyroy_error;
float gyroz_error;
float accx_error;
float accy_error;
float accz_error;


//IMU position estimation variables
float Pi = 3.141592653589;
float g = 9.80665;
float gyrox, gyroy,gyroz, accx, accy, accz, magx, magy, magz;
float dt;
float oldMillis, newMillis;
float pitch, roll, yaw;
float accPitch, accRoll, gyroPitch_inc, gyroRoll_inc, gyroYaw_inc, gyroPitch, gyroRoll, gyroYaw, gyroPitch_old, gyroRoll_old, gyroYaw_old, magPitch, magRoll, magYaw;
float ax, ay, mz, gx, gy, gz;
double kRoll, kPitch, kYaw;
//float imu_yaw_correction = -265;

//IMU location variables
float imu_speed, imu_distance, imu_totalDistanceTraveled, pitch360;
float x_distance, y_distance, z_distance;
float imu_lat, imu_long, imu_alt;

//IMU and GPS fusion variables

union value {       // 4 bytes to float conversion
  float f;
  unsigned long l;
  byte b[4];
} value;


//**********************************************************************************************************************************************************************************
void setup() {
  Serial3.begin(9600);
  Serial2.begin(115200);
  Serial.begin(115200);

  kalmanX.setAngle(0); // Setting initial roll angle to zero
  kalmanY.setAngle(0); // Setting initial pitch angle to zero
  kalmanZ.setAngle(0); // Setting initial yaw angle to zero

  oldMillis = 0;          //Setting up initial values
  gyroPitch_old = 999;    
  gyroRoll_old = 999;
  gyroYaw_old = 999; 
  sensorErrorCalculated = false; 
  imuLocationSetUp = false;
  k = 0;      
}
void loop() {
  newData = false;
  checksumOK = false;
  error = false;

  readMessage();
  checksumCheck();
  floatValues();
  GPSread();
  positionEstimation();
  complementaryFiltration();
  kalman();
  imuLocationEstimation();
  imu_gps_Fusion();
  valuesOut();

}
//****************************************************MESSAGE**READ*****************************************************************************************************************
void readMessage() {
  while (Serial2.available() > 0){ 
                 
    unsigned data = Serial2.read();
    if(data == 117){   //Searching for the first SYNC byte
        
      unsigned data = Serial2.read();
      if(data == 101){    //Searching for the second SYNC byte    
        for(j = 0; j <= (MAX_MESSAGE_LENGTH); j++){  //Saving full IMU message
        unsigned data = Serial2.read();
        message[j] = data;
        }
                
      }
    }
  }
  newData = true;
}
//************************CHECKSUM*********************************************************************************************************************************************
void checksumCheck(){
  if(newData == true){ 
    c1_old = 218; //Algorithm for calculating FLETCHER16 checksum
    c2_old = 335;
    for(j = 0; j <= (MAX_MESSAGE_LENGTH-3); j++){  
      c1_new = c1_old + message[j];
      c2_new = c2_old + c1_new;
      c1_old = c1_new;
      c2_old = c2_new; 
    }
    c1_chcksm = (c1_old % 256);
    c2_chcksm = (c2_old % 256);
    mess_c1 = message[58];
    mess_c2 = message[59];
    if(c1_chcksm == mess_c1 && c2_chcksm == mess_c2){
      checksumOK = true;
      error = false;
    }
    else{
      error = true;
      checksumOK = false;
    }
  }
  delay(dtime);
}
//***********************PRINTING**VALUES*******************************************************************************************************************************************************
void floatValues(){
  
  if(checksumOK == true  && error == false){
  
    //Accelerometer 04-----------------------------------------------------------------------------------------------------------------------------------------------------
    value.b[0] = message[35];   //Using UNION to get float
    value.b[1] = message[34];
    value.b[2] = message[33];
    value.b[3] = message[32];
    accx = (value.f)*10;

    value.b[0] = message[39];   
    value.b[1] = message[38];
    value.b[2] = message[37];
    value.b[3] = message[36];
    accy = (value.f)*10;
  
    value.b[0] = message[43];   
    value.b[1] = message[42];
    value.b[2] = message[41];
    value.b[3] = message[40];
    accz = (value.f)*10;
                            
    //Gyroscope 05----------------------------------------------------------------------------------------------------------------------------------------------------------
    value.b[0] = message[49];   //Using UNION to get float
    value.b[1] = message[48];
    value.b[2] = message[47];
    value.b[3] = message[46];
    gyrox = value.f;
        
    value.b[0] = message[53];   
    value.b[1] = message[52];
    value.b[2] = message[51];
    value.b[3] = message[50];
    gyroy = value.f;
        
    value.b[0] = message[57];   
    value.b[1] = message[56];
    value.b[2] = message[55];
    value.b[3] = message[54];
    gyroz = value.f;
    

      //Gaussmeter------------------------------------------------------------------------------------------------------------------------------------------------------------
    value.b[0] = message[21];   //Using UNION to get float
    value.b[1] = message[20];
    value.b[2] = message[19];
    value.b[3] = message[18];
    magx = value.f;
          
    value.b[0] = message[25];   
    value.b[1] = message[24];
    value.b[2] = message[23];
    value.b[3] = message[22];
    magy = value.f;
        
    value.b[0] = message[29];   
    value.b[1] = message[28];
    value.b[2] = message[27];
    value.b[3] = message[26];
    magz = value.f;               
  } 
  if(error == true && checksumOK == false){
    
  }        
}
void GPSread(){
  while(Serial3.available()){ // Checks for gps data
    if(gps.encode(Serial3.read()))
    
    gps.f_get_position(&lat,&lon); // Gets latitude and longitude
    gps.get_datetime(&date, &time);
    gps_speed = gps.f_speed_kmph();
    gps_course = gps.f_course();
    gps_altitude = gps.f_altitude();
  }
  gps_distance = ((gps_speed/3.6) * dt)/1000; 
  gps_totalDistanceTraveled = gps_totalDistanceTraveled + abs(gps_distance);
}
void positionEstimation(){
  if(checksumOK == true  && error == false){
    newMillis = millis();
    dt = (newMillis - oldMillis)/1000;  //Measuring time between two position estimations used in gyro integration
    oldMillis = millis();
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Calculating gyro and accelerometer errors.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(sensorErrorCalculated == false){                                     //These loops calculate error based on the first 50 measured values. When error is calculated, it is used for 
      gyrox_error = gyrox_error + gyrox;                                   //position estimation corrections. Without these corrections gyro has tendency to increase its value even when object is not moving at all.
      gyroy_error = gyroy_error + gyroy;
      gyroz_error = gyroz_error + gyroz;

      accx_error = accx_error + accx;                                   
      accy_error = accy_error + accy;
      accz_error = accz_error + accz;
      Serial.println("Calculating error, please don't move with your device.");
      k = k+1;

      if(k == 50){
        gyrox_error = gyrox_error / 50;
        gyroy_error = gyroy_error / 50;
        gyroz_error = gyroz_error / 50;

        accx_error = accx_error / 50;
        accy_error = accy_error / 50;
        accz_error = (accz_error / 50) + g;  //+g because gravity influences this value, so it is not 0 when calibrating
        sensorErrorCalculated = true;
      }
    }
    
    if(sensorErrorCalculated == true){
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Angle calculations
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Pitch
      
      //Accelerometer
      float accx_2 = accx - accx_error;   // Sometimes (accx - accx_error)/g is out of the interval -1;1. This and if conditions below solve that

      if(accx_2  > g){
          accx_2 = g;
      }
      if(accx_2 < (-g)){
          accx_2 = -g;
      }

      accPitch = asinf((accx_2) / g) * 180 / Pi;      

      if(accz < 0 && accPitch > 0){       // Recalculating every accelerometer value to get 0 - 360 degrees
        accPitch = accPitch;              // 0 - 90 degrees
      }   

      if(accz > 0 && accPitch > 0){       // 90 - 180 degrees
        accPitch = 180 - accPitch;
      }   

      if(accz > 0 && accPitch < 0){       // 180 - 270 degrees
        accPitch = 180 + (-accPitch);
      }   
      if(accz < 0 && accPitch < 0){       // 270 - 360 degrees
        accPitch = 360 + accPitch;
      }   
      
      //Gyroscope
      if(gyroPitch_old == 999){             //Because gyro does not have its own reference value, we need to get it from another sensor - accelerometer
        gyroPitch_old = accPitch;           
      }

      gyroPitch_inc = ((gyroy-gyroy_error)*dt) * 180 / Pi;                           
      gyroPitch = gyroPitch_old + gyroPitch_inc;
      if(gyroPitch > 360){                        //This solves pitch values over 360 (usually when last value is near 360, and the it crosses 0)
        gyroPitch = gyroPitch - 360;
      }       
      if(gyroPitch < 0){                      //This solves any negative values in angles. We get only 0 - 360 degrees.
        gyroPitch = gyroPitch + 360;                    
      }               
      pitchNumber = isDigit(gyroPitch);   //Gyroscope sometimes gets also NaN. Here ve detect, if output is a number, if not, we simply take value from accelerometer.
      if(pitchNumber == false){
        gyroPitch = accPitch;
      }
      
      
     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //Roll
     
     //Accelerometer
      accRoll = (atanf((accy-accy_error) / (accz-accz_error))) * 180 / Pi;

      if(accy < 0 && accz  < 0){        // Recalculating every accelerometer value to get 0 - 360 degrees
        accRoll = accRoll;              // 0 - 90 degrees
      }
      if(accy < 0 && accz > 0){         // 90 - 180 degrees
        accRoll = 180 + accRoll;
      }
      if(accy > 0 && accz > 0){         // 180 - 270 degrees
        accRoll = 180 + accRoll;
      }
      if(accy > 0 && accz < 0){          // 270 - 360 degrees
        accRoll = 360 + accRoll;
      }


      //Gyroscope
      if(gyroRoll_old == 999){              //The same goes for the first measurement of roll angle
        gyroRoll_old = accRoll;
      }
      
      gyroRoll_inc = ((gyrox-gyrox_error)*dt) * 180 / Pi;                            
      gyroRoll = gyroRoll_old + gyroRoll_inc;                         //Computing new increment and adding it to the last value, the same goes for yaw and roll. Then actual value save as "old"
      if(gyroRoll > 360){                                           //This solves pitch values over 360 (usually when last value is near 360, and the it crosses 0)
        gyroRoll = gyroRoll - 360;
      }
      if(gyroRoll < 0){                                               //This solves any negative values in angles. We get only 0 - 360 degrees.
        gyroRoll = gyroRoll + 360;                    
      }
      
      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //Yaw

      //Gaussmeter
      magYaw = atan2(magy, magx) * 180 / Pi;
      
      if(magYaw > 0 && magYaw < 180){
        magYaw = 180 - magYaw;
      }
      if(magYaw < 0 && magYaw > -180){                //This helps to recount Yaw value into 0 - 360 degree values.
        magYaw = -(magYaw) + 180;                    //Without this condition, we get 0 - 180 and 0 - (-180) degrees.
      }

      if(gyroYaw_old == 999){                 //And the last time -  uploading reference value for gyroscope.
        gyroYaw_old = magYaw;
      }

      //Gyroscope
      gyroYaw_inc = ((gyroz-gyroz_error)*dt) * 180 / Pi;                            
      gyroYaw = gyroYaw_old + gyroYaw_inc;
        if(gyroYaw > 360){                          //This solves pitch values over 360 (usually when last value is near 360, and the it crosses 0)
            gyroYaw = gyroYaw - 360;                
          }
        if(gyroYaw < 0){                      //This solves any negative values in angles. We get only 0 - 360 degrees.
            gyroYaw = gyroYaw + 360;                    
        }
    }
  }   
}
void complementaryFiltration(){ 
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
  //Data fusion - Complementary filter for final angle calculation
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PITCH
  if(gyroPitch < 90 && accPitch > 270){                         //This part of code is explained lower in code.
    accPitch = accPitch - 360;
    pitch = 0.95 * gyroPitch + 0.05 * accPitch;
  }
  if(accPitch < 90 && gyroPitch > 270){
    gyroPitch = gyroPitch - 360;
    pitch = 0.95 * gyroPitch + 0.05 * accPitch;
  }
  else{
    pitch = 0.95 * gyroPitch + 0.05 * accPitch;
  }
  if(pitch < 0){                                          //When final pitch after filtration comes up negative, it is automatically recalculated to positive number
    pitch = pitch + 360;
  }

  //ROLL
  if(gyroRoll < 90 && accRoll > 270){                          //In final estimation we have usually 3 different cases. In first case we have two values with similar number. 
      accRoll = accRoll - 360;                                 //Then we have no problem with calculation. However problem starts when one sensor is little bit higher number than 0
      roll = 0.95 * gyroRoll + 0.05 * accRoll;                  //and second one is lower than 360. Usually it would end up with final value around 180, which is incorrect. 
    }                                                          //Thats why we number <360 change in number lower than 0 (negative degrees), calculate solution, and if final solution is lower than 0
  if(accRoll < 90 && gyroRoll > 270){                       //we easily change number to number 0-360 equivalent to make negative degrees back to positive.
    gyroRoll = gyroRoll - 360;
    roll = 0.95 * gyroRoll + 0.05 * accRoll;
    }
  else{
    roll = 0.95 * gyroRoll + 0.05 * accRoll;
  }
  if(roll < 0){
    roll = roll + 360;                                  //When final roll after filtration comes up negative, it is automatically recalculated to positive number
  }

  //YAW
  magYaw = magYaw;
  if(gyroYaw < 90 && magYaw > 270){                       //This part of code is explained higher in code.
    magYaw = magYaw - 360;
    yaw = 0.1 * gyroYaw + 0.9 * magYaw;
  }
  if(magYaw < 90 && gyroYaw > 270){
    gyroYaw = gyroYaw - 360;
    yaw = 0.1 * gyroYaw + 0.9 * magYaw;
  }
  else{
    yaw = 0.1 * gyroYaw + 0.9 * magYaw;
  }
  if(yaw < 0){
    yaw = yaw + 360;                                    //When final yaw after filtration comes up negative, it is automatically recalculated to positive number
  }
      

  gyroPitch_old = pitch;
  gyroRoll_old = roll;
  gyroYaw_old = yaw;
  pitch360 = pitch;                               //In location estimation we calculate with 0 - 360 degrees, not +-180
  if(pitchRoll180 == true){
    if(pitch > 180){
      pitch = pitch - 360;
    }
    if(roll > 180){
      roll = roll - 360;
    }
  }
}
void kalman(){
  ax = accx;
  ay = accy;
  mz = magz;
  gx = gyrox;
  gy = gyroy;
  gz = gyroz;

  // Kalman filtration 
  kRoll = kalmanX.getAngle(ax, gx, 0.01);
  kPitch = kalmanY.getAngle(ay, gy, 0.01);
  kYaw = kalmanZ.getAngle(mz, gz, 0.01);
}
void imuLocationEstimation(){
  if(sensorErrorCalculated == true){        //Without error calibration we get wrong results
    
    //speed, distance calculation
    if( -0.07 < (accx - accx_error) && (accx - accx_error) < 0.07 ){ /accelerometer noise cancelling
      accx = 0;
      imu_speed = imu_speed + (accx  * dt );  
    }
    
    else{
      if( pitch > 0){
        imu_speed = imu_speed + ((accx - accx_error - abs(  (g * sin(pitch)))) * dt);  //Speed increment is acceleration - sensor error - accx made by rotating in space; for higher angles will need corrections, not done yet
      }
      if(pitch < 0){
        imu_speed = imu_speed + ((accx - accx_error + abs( (g * sin(pitch)))) * dt  );  //Speed increment is acceleration - sensor error - accx made by rotating in space
      }
    }
    
    imu_distance = imu_speed * dt ;
    imu_totalDistanceTraveled = imu_totalDistanceTraveled + abs(imu_distance);
    
    //Transformation into Earth coordinate system
    x_distance = (imu_distance * sin(yaw) * cos(pitch360)) / 1000;      //Vector of motion is divided into three axes by these formulas
    if((yaw >= 0 && yaw <= 179.99999) && x_distance < 0){                   
      x_distance = x_distance * (-1);
    }
    if(yaw >= 180 && yaw <= 359.99999 && x_distance > 0){
      x_distance = x_distance * (-1);
    }

    y_distance = (imu_distance * cos(yaw) * cos(pitch360)) / 1000;      
    if(yaw >= 270 || yaw <= 89.99999 && y_distance < 0){                   
      y_distance = y_distance * (-1);
    }
    if(yaw >= 90 && yaw <= 269.99999 && y_distance > 0){
      y_distance = y_distance * (-1);
    }

    z_distance = imu_distance * sin(pitch);       //Here we calculate in meters   

    

    if(imuLocationSetUp == false){    //First values we get from gps, then IMU goes on its own
      imu_lat = lat;              
      imu_long = lon;
      imu_alt = gps_altitude;
      imuLocationSetUp = true;
    }
    //Altitude
    imu_alt = imu_alt + z_distance;      
    //Latitude
    imu_lat = imu_lat + ((y_distance)/(6378.135 + imu_alt) * (180 / Pi));
    //Longitude
    imu_long = imu_long + ((x_distance)/(6378.135 + imu_alt) * (180 / Pi));
    
    
  }
}
void imu_gps_Fusion(){
} 
void valuesOut(){ 
  if(sensorErrorCalculated == true){
    pitch = pitch + kPitch;
    roll = roll + kRoll;
    yaw = yaw + kYaw + imu_yaw_correction;
      
    if(yaw >= 360){
      yaw = yaw - 360;
    }
    if(yaw < 0){
      yaw = yaw + 360;
    }
      
    //IMU data
    Serial.print(" IMU_measurements: ");
    Serial.print("  IMU_Latitude ");
    Serial.print(imu_lat, 10);
    Serial.print(" IMU_Longitude ");
    Serial.print(imu_long, 10);
    Serial.print(" IMU_Altitude ");
    Serial.print(imu_alt, 10);
    Serial.print(" IMU_PITCH: ");
    Serial.print(pitch, 10);
    Serial.print(" IMU_ROLL: ");
    Serial.print(roll, 10);
    Serial.print(" IMU_YAW: ");
    Serial.print(yaw, 10);
    Serial.print(" IMU_speed: ");
    Serial.print(imu_speed, 10);
    Serial.print(" IMU_distance: ");
    Serial.print(imu_distance, 10);
    Serial.print(" IMU_total_distance: ");
    Serial.print(imu_totalDistanceTraveled, 10);
      
    //GPS data
    Serial.print(" GPS_measurements: ");
    Serial.print(" GPS_Latitude: ");
    Serial.print(lat, 10);
    Serial.print(" GPS_Longitude: ");
    Serial.print(lon, 10); 
    Serial.print(" GPS_Altitude: ");
    Serial.print(gps_altitude, 10);
    Serial.print(" GPS_Speed: ");
    Serial.print(gps_speed, 10);
    Serial.print(" GPS_Distance: ");
    Serial.print(gps_distance, 10);
    Serial.print(" GPS_total_distance: ");
    Serial.print(gps_totalDistanceTraveled, 10);
    Serial.print(" GPS_Date_(ddmmyy):");
    Serial.print(date);
    Serial.print(" GPS_Time_(hhmmss): ");
    Serial.println(time);
    Serial.print(" ");
    }
    
  if(error == true && checksumOK == false){
      
  }        
}
  
    





