//Libraries
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//Globals
LSM9DS1 imu;

#define LSM9DS1_M 0x1E 
#define LSM9DS1_AG 0x6B 
#define PRINT_CALCULATED
//#define PRINT_RAW

#define DECLINATION 1.16 


static unsigned long lastPrint = 0; // Keep track of print time
float prevTime = 0; //used to hold last time a calculation was done

float timeAGM[10]; //[time, acc, gyro, magneto]

float pos[4];
float vel[3];
float acc[3];

float resetVariableX = 0;
float resetVariableY = 0;
float resetVariableZ = 0;

float acceleroMarge = 0.1;
float resetMarge = 10;

#define PRINT_SPEED 1            // the calculation will be done 1000/print_speed times
float timesToSkipThePrint = 100; //Skips *100* times a print statement
float timesPrintSkipped = 0;     //keeps track of the times the print is skipped

void setup() 
{
  
  Serial.begin(115200);
  
  // initialization
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
 
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

void accelToPos(float currTime)
{
  float deltaTime = (currTime - prevTime)/1000; //calculates the time between the current and previous calculation
  prevTime = currTime;
  
  float accX = imu.calcAccel(imu.ax);
  float accY = imu.calcAccel(imu.ay);
  float accZ=  imu.calcAccel(imu.az)-1;

  //---->AANPASSEN NAAR HET STOKSTELSEL
  
  //get the previous position and velocity
  float posX = pos[1];
  float posY = pos[2];
  float posZ = pos[3];
  
  float velX = vel[0];
  float velY = vel[1];
  float velZ = vel[2];  
  
  //calculate the new position and store them together with the time 
  float newPosX = posX + velX * deltaTime;
  float newPosY = posY + velY * deltaTime;
  float newPosZ = posZ + velZ * deltaTime;
  
  pos[0] = currTime;
  pos[1] = newPosX;
  pos[2] = newPosY;
  pos[3] = newPosZ;
 
 String stringToSend;
 stringToSend +=pos[0];
 stringToSend +=pos[1];
 stringToSend +=pos[2];
 stringToSend +=pos[3];
 char arrayToSend[stringToSend.length()+1];
 stringToSend.toCharArray(arrayToSend,stringToSend.length()+1);

//arrayToSend is an array with the time, xPos,yPos,zPos which can be send
String test;
test += accX;
test += accY;
test += accZ;
char testtest[test.length()+1];
  Serial.println(test);
 // Serial.write(testtest);

  ///------>positie opslaan in een lijst of in het geheugen
  
  //calculating new velocities  
  if (abs(accX) > acceleroMarge)                          //stationaire waardes vermijden
  { 
   float newVelX = velX + accX*deltaTime*9.81;  // a staat in g, dus maal 9.81  
   vel[0] = newVelX;
   resetVariableX = 0;
  }
  else{
    resetVariableX +=1;
  }
   
  if (abs(accY) > acceleroMarge){
   float newVelY = velY + accY*deltaTime*9.81;
   vel[1] = newVelY;
   resetVariableY = 0;}
   else{
    resetVariableY +=1;
   }

   
   if (abs(accZ) > acceleroMarge){
   float newVelZ = velZ + (accZ)*deltaTime*9.81;
   vel[2] = newVelZ;
   resetVariableZ = 0;}
   else{
    resetVariableZ +=1;}

//When the acceleration doesn't update, the velocity is set on 0. 
  if (resetVariableX > resetMarge){
    vel[0] = 0;
    resetVariableX = 0;
    }
    if (resetVariableY > resetMarge){
    vel[1] = 0;
    resetVariableY = 0;
    }
    if (resetVariableZ > resetMarge){
    vel[2] = 0;
    resetVariableZ = 0;
    }

    //printing on certain times
if (timesPrintSkipped == timesToSkipThePrint){
 //printGyro();
  printMag();
  //printAccel();
  //printPosition();
  //printVelocity();
  //printAttitude(imu.ax, imu.ay, imu.az,-imu.my, -imu.mx, imu.mz);
  Serial.println("");
  timesPrintSkipped = 0;
  
  }
  
  timesPrintSkipped +=1;
}

void printPosition(){
   Serial.println("------>POSITIE: ");
  int i=0;
  for (i=1;i<4;i++){
    Serial.print(pos[i]); Serial.println("   ,   ");
    
  }
}


void printVelocity(){
  Serial.println("----> VELOCITY");
  int v=0;
  for (v=0;v<3;v++){
    Serial.print(vel[v]); Serial.println("   ,   ");
    } 
}


void loop()
{
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
  
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    
                 
  
    
    lastPrint = millis(); // Update lastPrint time
    //calculate the new position and velocity
    
    accelToPos(lastPrint);
    
    timeAGM[0] = lastPrint;
 
  }
}

void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
  
  //update the list
  timeAGM[4] = imu.calcGyro(imu.gx);
  timeAGM[5] = imu.calcGyro(imu.gy);
  timeAGM[6] = imu.calcGyro(imu.gz);
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
 //update the list
  timeAGM[1] = imu.calcAccel(imu.ax);
  timeAGM[2] = imu.calcAccel(imu.ay);
  timeAGM[3]=  imu.calcAccel(imu.az);
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
  //update the list
  timeAGM[7] = imu.calcMag(imu.mx);
  timeAGM[8] = imu.calcMag(imu.my);
  timeAGM[9] = imu.calcMag(imu.mz);
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{

 
  float roll = atan2(ay, az);
  
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

 
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  
  pitch *= 180.0 / PI;
  
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
  
  Serial.println("");
  
}



