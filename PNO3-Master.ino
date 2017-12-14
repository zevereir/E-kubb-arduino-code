///Libraries///
#include <SparkFunLSM9DS1.h>
//#include <VirtualWire.h>
#include <CapacitiveSensor.h>
#include <SD.h>

///Globals///
LSM9DS1 imu;
CapacitiveSensor   capacitive = CapacitiveSensor(4,2);

#define LSM9DS1_M 0x1E 
#define LSM9DS1_AG 0x6B 
#define PRINT_CALCULATED

//calculation speed
static unsigned long lastPrint = 0; 
#define PRINT_SPEED 1            

//vectors
float pos[3];
float vel[3];
float xAxis[3]={1,0,0};
float yAxis[3]={0,1,0};
float zAxis[3]={0,0,1};
float oldxAxis[3] = {1,0,0};
float orientationCorrection[3] = {1,0,0};
float newAcc[3];
float sdVector[7];

//counters and marges
float resetVariables[3] = {0,0,0};
float gyroMarges[3] = {3,2.8,0.7};
float prevTime = 0; //used to hold last time a calculation was done
float acceleroMarge = 0.1;
float resetMarge = 10;
float timesResultIsFalse = 0;
float timesUntilResultIsFalse = 3;
float timesResultCheckIsSkipped = 0;
float timesToSkipResultCheck = 10;
float sortOfResult = 3;
float result=2;
float checkCounter =0;
float checkAmount = 0;
float checkSum =0;
float RFCounter=0;
float alphaBetaTime;
float betaGammaTime;
float gammaATime;
float resetMillis;
float crashCounter =0;
int Phase; 
long capacitiveThresh = 10000;

//pins
int transmit_pin = 7;
int receive_pin = 4;
int transmit_en_pin = 3;
int buttonPin = 5;
int SDPin = 10;

//SD

File myFile;
char sdChar;
String sdValue;
unsigned long filePosition;

void setup() 
{
  
  Serial.begin(115200);
  
  // initialization IMU
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG; 
  if (!imu.begin())
  {   
  }
 
  // initialization capacitive sensor and button
  capacitive.set_CS_AutocaL_Millis(0xFFFFFFFF);
  pinMode(buttonPin, INPUT);
   
 /* // initialization VirtualWire
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(4000);   // Bits per sec
 */
  //sd
  if (!SD.begin(4)) {
    return;
  }
  SD.remove("stok.txt");
  myFile = SD.open("stok.txt", FILE_WRITE);
}

void loop()
{
  // Update the sensor values whenever new data is available
  if ((Phase == 0) || (Phase == 1) || (Phase == 2) )
  {
    if ( imu.gyroAvailable() )
    {
      imu.readGyro();
    }
    if ( imu.accelAvailable() )
    {
      imu.readAccel();
    }
   if ( imu.magAvailable() )
    {
     imu.readMag();
    }
  }
  //checking for reset
   if (buttonPressed() && !capacitivePressed()){
    reset(); 
   }
   
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    
   if (Phase == 0){alpha();}
    else if (Phase == 1){beta();}
    else if (Phase == 2) {gamma();} 
    else if (Phase == 3){a();}
    else if (Phase == 4){b();}
    else if (Phase == 5){c();}
    else{d();}
  }
}


void alpha()
{
  SDWrite();
  if (buttonPressed() && capacitivePressed()){
     Phase = 1;
     alphaBetaTime = millis()-resetMillis;
    }
  }
  
void beta(){
  SDWrite();
  if (!capacitivePressed()){
     Phase = 2;
     betaGammaTime = millis()-resetMillis;
    }}

void gamma(){
  SDWrite();
  if (buttonPressed() && capacitivePressed()){
     Phase = 3;
     gammaATime = millis()-resetMillis;
     myFile.close();
    }}

    
void a(){
  myFile = SD.open("stok.txt");
  SDRead();
  if (sdVector[0] <= alphaBetaTime){
    accelToPos(sdVector[0],sdVector[1],sdVector[2],sdVector[3],sdVector[4],sdVector[5],sdVector[6]);
  }
  else{
    Phase = 4;
  }
}
  
void b(){
  SDRead();
  if (sdVector[0] <= betaGammaTime){
    accelToPos(sdVector[0],sdVector[1],sdVector[2],sdVector[3],sdVector[4],sdVector[5],sdVector[6]);
    throwCheck();
  }
  else{
    if ((checkSum/checkAmount)>0){
      result =0;
      sortOfResult = 0;
    }
    Phase = 5;
  }
}
void c(){
  SDRead();
  if (!crash(sdVector[1],sdVector[2],sdVector[3])){
    accelToPos(sdVector[0],sdVector[1],sdVector[2],sdVector[3],sdVector[4],sdVector[5],sdVector[6]);
    /*if (RFCounter == 20){
      sendRFData(false);
      RFCounter =0;
      }
    RFCounter +=1;*/
    spinCheck();
   }
  
  else{
    /*sendRFData(true);*/
    Phase = 6;
    myFile.close();
  }   
}

void d(){}

void SDWrite(){
 String sdWriteString;
 sdWriteString += millis()-resetMillis;
 sdWriteString += ";";
 sdWriteString += imu.calcAccel(imu.ax);
 sdWriteString += ";";
 sdWriteString += imu.calcAccel(imu.ay);
 sdWriteString += ";";
 sdWriteString += imu.calcAccel(imu.az);
 sdWriteString += ";";
 sdWriteString += imu.calcGyro(imu.gx);
 sdWriteString += ";";
 sdWriteString += imu.calcGyro(imu.gy);
 sdWriteString += ";";
 sdWriteString += imu.calcGyro(imu.gz);
 sdWriteString += ";";
 if (myFile) {
     myFile.println(sdWriteString);    
 }
}

void SDRead()
{
  if (myFile) {
    int indexOfArray = 0;
    if (myFile.available()) 
    {
      myFile.seek(filePosition);
      sdChar = myFile.read();
      while (sdChar != '\n'){
        if (sdChar != ';'){
          sdValue = sdValue + sdChar;
          }else{
        sdVector[indexOfArray] = sdValue.toFloat();
        sdValue = "";
        indexOfArray = indexOfArray + 1;
        } 
        sdChar = myFile.read();       
        } 
        filePosition = myFile.position();
    }
  
  }
}

bool crash(float aX, float aY, float aZ){
  if (crashCounter=100){
  return (sqrt(sq(aX)+sq(aY)+sq(aZ))>1);}
  else
 {crashCounter +=1;
  return false;}
  }

void throwCheck( ){
  if (checkCounter == 10){
    checkSum += xAxis[0];
    checkAmount += 1;
    checkCounter =0; 
  }
  else{
    checkCounter+=1;
  }
}

bool capacitivePressed(){
 return (capacitive.capacitiveSensor(30) > capacitiveThresh);
}
bool buttonPressed(){
 return (digitalRead(buttonPin) == HIGH);
}

void accelToPos(float currTime, float accX, float accY, float accZ, float gX, float gY, float gZ)
{
  float deltaTime = (currTime - prevTime)/1000; //calculates the time between the current and previous calculation
  prevTime = currTime;
 
  updateAxes(deltaTime, gX, gY,  gZ);
  
  newAcc[0] = accX*xAxis[0] + accY*yAxis[0] + accZ*zAxis[0];
  newAcc[1] = accX*xAxis[1] + accY*yAxis[1] + accZ*zAxis[1];
  newAcc[2] = accX*xAxis[2] + accY*yAxis[2] + accZ*zAxis[2];

  float accLen = (sq(newAcc[0]) + sq(newAcc[1]) + sq(newAcc[2]));
  if ((Phase != 5) && (absolute(accLen-1)< 0.005 )){
    orientationCorrection[0] = newAcc[0]/accLen;
    orientationCorrection[1] = newAcc[1]/accLen;
    orientationCorrection[2] = newAcc[2]/accLen;
  }
  
  accX = newAcc[0]-orientationCorrection[0];
  accY = newAcc[1]-orientationCorrection[1];
  accZ = newAcc[2]-orientationCorrection[2];
  
  //get the previous position and velocity
  float posX = pos[0];
  float posY = pos[1];
  float posZ = pos[2];
  
  float velX = vel[0];
  float velY = vel[1];
  float velZ = vel[2];  
  
  //calculate the new position and store them together with the time 
  float newPosX = posX + velX * deltaTime;
  float newPosY = posY + velY * deltaTime;
  float newPosZ = posZ + velZ * deltaTime;
  
  
  pos[0] = newPosX;
  pos[0] = newPosY;
  pos[1] = newPosZ;
  //-----> orientatie via eenheidsvector
  // [0 of 1] -> 0 fout, 1 goe, 2 onbekend . Wordt doorgestuurd op het einde. 
 
  //calculating new velocities  
  if (absolute(accX) > acceleroMarge)                          //stationaire waardes vermijden
  { 
   float newVelX = velX + accX*deltaTime*9.81;  // a staat in g, dus maal 9.81  
   vel[0] = newVelX;
   resetVariables[0] = 0;
  }
  else{
    resetVariables[0] +=1;
  }
   
  if (absolute(accY) > acceleroMarge){
   float newVelY = velY + accY*deltaTime*9.81;
   vel[1] = newVelY;
   resetVariables[1] = 0;}
   else{
    resetVariables[1] +=1;
   }

   
   if (absolute(accZ) > acceleroMarge){
   float newVelZ = velZ + (accZ)*deltaTime*9.81;
   vel[2] = newVelZ;
   resetVariables[2] = 0;}
   else{
    resetVariables[2] +=1;}

//When the acceleration doesn't update, the velocity is set on 0. 
  if (resetVariables[0] > resetMarge){
    vel[0] = 0;
    resetVariables[0] = 0;
    }
    if (resetVariables[1] > resetMarge){
    vel[1] = 0;
    resetVariables[1] = 0;
    }
    if (resetVariables[2] > resetMarge){
    vel[2] = 0;
    resetVariables[2] = 0;
    }
}
/*
void sendRFData(bool resultSend){
 String stringToSend1;
 String stringToSend2;
 stringToSend1 += "(";
 if (!resultSend){
 
 stringToSend1 +=pos[0];
 stringToSend1 += " ";
 stringToSend1 +=pos[1];
 stringToSend1 += " ";
 stringToSend1 +=pos[2];

 stringToSend2 +=xAxis[0];
 stringToSend2 += " ";
 stringToSend2 +=xAxis[1];
 stringToSend2 += " ";
 stringToSend2 +=xAxis[2]; 
 }
 else{
  stringToSend1 += "(";
  stringToSend1 += result;
  stringToSend2 += sortOfResult;
 }
 stringToSend2 += ")";
  
 char arrayToSend1[stringToSend1.length()+1];
 stringToSend1.toCharArray(arrayToSend1,stringToSend1.length()+1);

  vw_send((uint8_t *)arrayToSend1, strlen(arrayToSend1));
  vw_wait_tx(); // Wait until the whole message is gone
  
  Serial.write(arrayToSend1);

char arrayToSend2[stringToSend2.length()+1];
 stringToSend2.toCharArray(arrayToSend2,stringToSend2.length()+1);
 
vw_send((uint8_t *)arrayToSend2, strlen(arrayToSend2));
  vw_wait_tx(); // Wait until the whole message is gone
  
  Serial.write(arrayToSend2);
}

*/
void updateAxes(float deltaTime, float gX, float gY, float gZ){

  float rotVelx = gX*(PI/180);
  float rotVely = gY*(PI/180);
  float rotVelz = gZ*(PI/180);

  // Driftwaardes voorkomen door marge in te bouwen
  if (absolute(rotVelx) < gyroMarges[0]*(PI/180)){
   rotVelx = 0;
   }

  if (absolute(rotVely) < gyroMarges[1]*(PI/180)){
   rotVely = 0;
   }

  if (absolute(rotVelz) < gyroMarges[2]*(PI/180)){
   rotVelz = 0;
   }

  float angx = rotVelx * deltaTime;
  float angy = rotVely * deltaTime;
  float angz = rotVelz * deltaTime;
 
  float currxAxis[3];
  float curryAxis[3];
  float currzAxis[3];

currxAxis[0] = xAxis[0];
currxAxis[1] = xAxis[1];
currxAxis[2] = xAxis[2];
curryAxis[0] = yAxis[0];
curryAxis[1] = yAxis[1];
curryAxis[2] = yAxis[2];
currzAxis[0] = zAxis[0];
currzAxis[1] = zAxis[1];
currzAxis[2] = zAxis[2];
  
  
  // Berekenen nieuwe x as van het stokstelsel in het wereldstelsel
  xAxis[0] = cos(angz)*cos(angy)*currxAxis[0] - sin(angz)*cos(angy)*curryAxis[0] + sin(angy)*currzAxis[0];
  xAxis[1] = cos(angz)*cos(angy)*currxAxis[1] - sin(angz)*cos(angy)*curryAxis[1] + sin(angy)*currzAxis[1];
  xAxis[2] = cos(angz)*cos(angy)*currxAxis[2] - sin(angz)*cos(angy)*curryAxis[2] + sin(angy)*currzAxis[2];

  // Berekenen nieuwe y as van het stokstelsel in het wereldstelsel
  yAxis[0] = sin(angz)*cos(angx)*currxAxis[0] + cos(angz)*cos(angx)*curryAxis[0] - sin(angx)*currzAxis[0];
  yAxis[1] = sin(angz)*cos(angx)*currxAxis[1] + cos(angz)*cos(angx)*curryAxis[1] - sin(angx)*currzAxis[1];
  yAxis[2] = sin(angz)*cos(angx)*currxAxis[2] + cos(angz)*cos(angx)*curryAxis[2] - sin(angx)*currzAxis[2];

  // Berekenen nieuwe y as van het stokstelsel in het wereldstelsel
  zAxis[0] = -sin(angy)*currxAxis[0] + sin(angx)*cos(angy)*curryAxis[0] + cos(angx)*cos(angy)*currzAxis[0];
  zAxis[1] = -sin(angy)*currxAxis[1] + sin(angx)*cos(angy)*curryAxis[1] + cos(angx)*cos(angy)*currzAxis[1];
  zAxis[2] = -sin(angy)*currxAxis[2] + sin(angx)*cos(angy)*curryAxis[2] + cos(angx)*cos(angy)*currzAxis[2];
}

void spinCheck()
{  
 float angleToCheck;
 float x1 = oldxAxis[0];
 float y1= oldxAxis[1];
 float z1= oldxAxis[2];
 float x2 = xAxis[0];
 float y2= xAxis[1];
 float z2= xAxis[2];
 oldxAxis[0] = x2;
 oldxAxis[1] = y2;
 oldxAxis[2] = z2;

 angleToCheck = absolute(acos((y1*z2-y2*z1)/sqrt(sq(x1*y2-x2*y1)+sq(x1*z2-x2*z1)+sq(y1*z2-y2*z1))));
 
 if (angleToCheck > PI/2){
  angleToCheck = PI - angleToCheck;
 }
 if (angleToCheck < PI/4){
  timesResultIsFalse +=1 ;
 }
 if (timesResultIsFalse == timesUntilResultIsFalse)
 {
  result = 0;
  if (sortOfResult == 3){
  sortOfResult = 1;
  }
  else{
  sortOfResult =1;}
 }
  
  }
  
float absolute(float value){
if (value<0){
  return -value;}
else{
  return value;
  }
}


void reset()
{
  Phase = 0;
  pos[0] =0;
  pos[1] = 0;
  pos[2] = 0;
  vel[0] = 0;
  vel[1]= 0;
  vel[2]= 0;
  xAxis[0]=1;
  xAxis[1]=0;
  xAxis[2]=0;
  yAxis[0]=0;
  yAxis[1]=1;
  yAxis[2]=0;
  zAxis[0]=0;
  zAxis[1]=0;
  zAxis[2]=1;
  resetMillis = millis();
  alphaBetaTime=0;
  betaGammaTime=0;
  gammaATime=0;
  RFCounter =0;
  result=2;
  sortOfResult=3;
  crashCounter =0;
  checkCounter =0;
  checkAmount = 0;
  checkSum =0;
  SD.remove("stok.txt");
}

