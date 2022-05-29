#include <EEPROM.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#define analogInPinA 22
#define analogInPinB 23
#define I2C_SLAVE 8

Servo pitchServo, yawServo;
int indexY = -1;
int indexP = -1;
int indexK = -1;
int indexI = -1;
int indexD = -1;
int indexM = -1;
int indexS = -1;
int indexV = -1;
int indexB = -1;
int opt = 0;
char a[21];
char b[21];
char c;
int rx = 0;
bool whoGoes = false;
bool request = false;
bool isReady = false;
String message= "N";
String receivedMessage = "";
int kpAddr = 0, kiAddr = 1, kdAddr = 2, offA = 3, offB = 4, modeAddr = 5, vertAddr = 6, deadBandAddr = 7;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0.1, Ki=0.5, Kd=0.7;
int kP, kI, kD;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

bool go = false;
bool received = false;

bool precPitchDirection = false;
bool precYawDirection = false;

bool firstCalibration = false;

bool yawDirection = false;
bool pitchDirection = false;

bool lockPitch = false;
bool lockYaw = true;

int diff = 0;

int posCounterYaw = 90;
int posCounterPitch = 95;

int stepPitch = 0;
int stepYaw = 0;

int trackerMode = 0;
int precTrackerMode = 0;
int isVerticalTracking = 1;

int pinAOffset = 0;
int pinBOffset = 0;

int countPitch = 0;
int countYaw = 0;
int countRead = 0;

int analA, rawAnalA;
int analB, rawAnalB;
double averageA = 0;
double averageB = 0;

int yawPos = 90;
int pitchPos = 95;

int precPitch = 0;
int precYaw = 0;

unsigned long timeToWaitYaw = 30;
unsigned long timeToWaitPitch = 30;

unsigned long timeToWaitMessage = 100;
unsigned long messageTimer = 0;

unsigned long pitchCountMax = timeToWaitPitch;
unsigned long yawCountMax = timeToWaitYaw;

unsigned long now;
unsigned long precRead = 0;

unsigned long pitchTimer = 0;
unsigned long yawTimer = 0;

int stepYawCount = 20;
int stepPitchCount = 20;

int maxRSSIYaw = 0;
int maxRSSIPitch = 0;
int precRSSIYaw = 0;
int precRSSIPitch = 0;

int maxRSSI = 0;

int offset = 2;
int deadBand = 10;


void setup() {
  analogReference(EXTERNAL);
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  Wire.begin(I2C_SLAVE);
  message.toCharArray(b, 21);
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receive);
  pitchServo.attach(2);
  yawServo.attach(3);
  pitchServo.write(95);
  yawServo.write(90);
  pinMode(analogInPinA, INPUT_PULLUP);
  pinMode(analogInPinB, INPUT_PULLUP);

  loadFromEEPROM();
  
  if(isVerticalTracking == 1)
    lockPitch = true;

  if(trackerMode < 3)
    lockYaw = true;
    
  Setpoint = 100;
  myPID.SetOutputLimits(5, 100);
  myPID.SetSampleTime(5);
  myPID.SetTunings(Kp, Ki, Kd);  

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  delay(2000);
}
void(* reset) (void) = 0;

void loop() {
  readRSSI();
  moveYaw();
  movePitch();
  manageReceive();
  manageSend();
}

void loadFromEEPROM(){
  kpAddr *= sizeof(int);
  kiAddr *= sizeof(int);
  kdAddr *= sizeof(int);
  offA *= sizeof(int);  
  offB *= sizeof(int);  
  modeAddr *= sizeof(int);
  vertAddr *= sizeof(int);
  deadBandAddr *= sizeof(int); 

  EEPROM.get(kpAddr, kP);
  EEPROM.get(kiAddr, kI);
  EEPROM.get(kdAddr, kD);
  EEPROM.get(offA, pinAOffset);
  EEPROM.get(offB, pinBOffset);
  EEPROM.get(modeAddr, trackerMode);
  EEPROM.get(vertAddr, isVerticalTracking);
  EEPROM.get(deadBandAddr, deadBand);
  
  Kp = kP/1000.00;
  Ki = kI/1000.00;
  Kd = kD/1000.00;
}

void saveToEEPROM(){
  update(kpAddr, kP);
  update(kiAddr, kI);
  update(kdAddr, kD);
  update(offA, pinAOffset);
  update(offB, pinBOffset);
  update(modeAddr, trackerMode);
  update(vertAddr, isVerticalTracking);
  update(deadBandAddr, deadBand);
}

void update(int addr, int val){
  int c = 0;
  if(EEPROM.get(addr, c) != val)
    EEPROM.put(addr, val);
}

void manageSend(){
  if(!request){
    if(whoGoes){
      whoGoes = false;
      message = String(analA)+"A"+String(analB)+"B"+String(yawPos)+"Y"+String(pitchPos)+"P"+String(kP)+"K";
    }else{
      whoGoes = true;
      message = String(kI)+"I"+String(kD)+"D"+String(trackerMode)+"M"+String(isVerticalTracking)+"V"+String(deadBand)+"C";
    }
    request = true;
    isReady = false;
    rx = 0;
  } else {
    if(rx < message.length()){
      a[rx] = message.charAt(rx);
      rx++;
    } else
      isReady = true;
  }
}

void manageReceive(){
  if(received){
    received = false;
    request = false;
    
    indexY = receivedMessage.indexOf('Y');
    indexP = receivedMessage.indexOf('P');
    indexK = receivedMessage.indexOf('K');
    indexI = receivedMessage.indexOf('I');
    indexD = receivedMessage.indexOf('D');
    indexM = receivedMessage.indexOf('M');
    indexS = receivedMessage.indexOf('S');
    indexV = receivedMessage.indexOf('V');
    indexB = receivedMessage.indexOf('B');

    if(indexK>=0){
      kP = receivedMessage.substring(0, indexK).toInt(); 
      kI = receivedMessage.substring(indexK+1, indexI).toInt();
      kD = receivedMessage.substring(indexI+1, indexD).toInt();
      Kp = kP/1000.00;
      Ki = kI/1000.00;
      Kd = kD/1000.00;
      myPID.SetTunings(Kp, Ki, Kd);
    }

    if(indexM>=0){
      trackerMode = receivedMessage.substring(0, indexM).toInt();   

      if(trackerMode > 0)
        go = true;
    }

    if(indexV>=0)
      isVerticalTracking = receivedMessage.substring(0, indexV).toInt();   

    if(isVerticalTracking == 0){
      if(indexP>=0)
        pitchPos = receivedMessage.substring(indexY+1, indexP).toInt();
      stepPitch = 1;
    } else {
        lockPitch = true;
        stepPitch = 0;
        countPitch = 0;
        pitchCountMax = timeToWaitPitch;      
    }

    if(trackerMode == 3){
      if(indexY>=0)
        yawPos = receivedMessage.substring(0, indexY).toInt();
      stepYaw = 1; 
    } else {
      lockYaw = true;
      stepYaw = 0;
      countYaw = 0;
      yawCountMax = timeToWaitYaw; 
    }
      
    if(indexS>=0){
      opt = receivedMessage.substring(0, indexS).toInt();
      if(opt == 1)
      saveToEEPROM();
  
      if(opt == 2)
        reset();
    
      if(opt == 3)
        calibration();     
    }

    if(indexB>=0)
      deadBand = receivedMessage.substring(0, indexB).toInt();

    receivedMessage = "";
  }
}

void requestEvent() {
  if(isReady)
    Wire.write(a);
  else
    Wire.write(b);
    
  request = false;
}

void receive(int howMany) {
  while (Wire.available())   // slave may send less than requested
  {
    c = Wire.read();    // receive a byte as character
    receivedMessage += String(c);
  }

  received = true;
}

void readRSSI(){
  now = micros();

  if(now - precRead >= 133){
    precRead = now;

    averageA += analogRead(analogInPinA);
    averageB += analogRead(analogInPinB);  
    
    if(countRead>=15){
      countRead = 0;
      
      averageA /= 15;
      averageB /= 15;
      
      rawAnalA = (int) averageA;
      rawAnalB = (int) averageB;
    
      analA = rawAnalA - pinAOffset;
      analB = rawAnalB - pinBOffset;
  
      averageA = 0;
      averageB = 0;
    
      if(analA>analB){
        diff =(analA-analB);
        maxRSSI = analA;
        maxRSSIPitch = analB;
        if(trackerMode == 0)
          yawDirection = false;
      } else {
        diff =(analB-analA);
        maxRSSI = analB;
        maxRSSIPitch = analA;
        if(trackerMode == 0)
          yawDirection = true;
      }
    
      if(trackerMode == 0){
        Input = diff;
        myPID.Compute();
        timeToWaitYaw = (int) Output;
        if(diff < deadBand || maxRSSI<175){
          lockPitch = true;
          go = false;
        } else {
          go = true;
          lockPitch = false;
        }
      } else if(trackerMode < 3)
        timeToWaitYaw = timeToWaitPitch;
        else
        timeToWaitYaw = timeToWaitPitch;
      
      if(trackerMode == 1){
        maxRSSIYaw = analA;
        maxRSSIPitch = analA;
      }
    
      if(trackerMode == 2){
        maxRSSIYaw =  analB;
        maxRSSIPitch = analB;
      }
    } else
      countRead++;
  }
}

void movePitch(){
  now = millis();

  if(now-pitchTimer >= timeToWaitPitch){
    pitchTimer = now;

    if(isVerticalTracking)
      countPitch++;
    else
      countPitch = 0;

    if(stepPitch == 0 && lockPitch && countPitch >= pitchCountMax){

      countPitch = 0;
      lockYaw = false;
      stepPitch = 1;
      precRSSIPitch = maxRSSIPitch;
  
      if(pitchDirection){
        if(pitchPos>120)
          pitchPos =  posCounterPitch - 3;
        else
          pitchPos =  posCounterPitch - 2;
      }
      else
        pitchPos =  posCounterPitch + 2;
      
      if(pitchPos<=70)
        pitchPos = 70;
  
      if(pitchPos>=160)
        pitchPos = 160;

      if(precPitchDirection == pitchDirection)
        pitchCountMax -= timeToWaitPitch;

      if(pitchCountMax < 0)
        pitchCountMax = 0;

      precPitchDirection = pitchDirection;
    }
          

    if(stepPitch > 1 && stepPitch<stepPitchCount){
      stepPitch++;
    }

    if(stepPitch == 1){

      if(maxRSSIPitch > precRSSIPitch)
        precPitch = posCounterPitch;
      
      if(pitchPos > posCounterPitch)
        posCounterPitch++;
  
      if(pitchPos < posCounterPitch)
        posCounterPitch--;
  
      if(pitchPos == posCounterPitch)
        if(isVerticalTracking == 1)
          stepPitch = 2;
        else
          stepPitch = 1;
      
      if(posCounterPitch<=75)
        posCounterPitch = 75;
  
      if(posCounterPitch>=180)
        posCounterPitch = 180;
  
      pitchServo.write(posCounterPitch);
    }

    if(stepPitch >= stepPitchCount){

      if(posCounterPitch <= precPitch + offset && posCounterPitch >= precPitch - offset){
        pitchCountMax += timeToWaitPitch;
        pitchDirection != pitchDirection;
      } else {
        if(precPitch > posCounterPitch)
          pitchDirection = false;
      
        if(precPitch < posCounterPitch)
          pitchDirection = true;
      }

      if(posCounterPitch <= 75)
        pitchDirection = false;

      if (posCounterPitch>= 180)
        pitchDirection = true;
      
      stepPitch = 0;
      lockYaw = true;
    }
  }
}

void moveYaw(){
  now = millis();

  if(trackerMode == 0){
    if(now-yawTimer >= timeToWaitYaw && lockYaw && go){
      yawTimer = now;
      
      if(yawDirection && yawPos<180)
        yawPos++;
    
      if(!yawDirection && yawPos>0)
        yawPos--;

      posCounterYaw = yawPos;
    
      yawServo.write(yawPos);    
    }
  }

  if(trackerMode > 0){
    if(now-yawTimer >= timeToWaitYaw){
      yawTimer = now;

      if(trackerMode < 3)
        countYaw++;
      else
        countYaw = 0;
  
      if(stepYaw == 0 && lockYaw && countYaw >= yawCountMax){
        countYaw = 0;
        lockPitch = false;
        stepYaw = 1;
        precRSSIYaw = maxRSSIYaw;
    
        if(yawDirection)
          yawPos =  posCounterYaw - 3;
        else
          yawPos =  posCounterYaw + 3;
        
        if(yawPos<=0)
          yawPos = 0;
    
        if(yawPos>=180)
          yawPos = 180;
  
        if(precYawDirection == yawDirection)
          yawCountMax -= timeToWaitYaw;
  
        if(yawCountMax < 0)
          yawCountMax = 0;

        precYawDirection = yawDirection;
      }
            
  
      if(stepYaw > 1 && stepYaw<stepYawCount){
        stepYaw++;
      }
  
      if(stepYaw == 1){
  
        if(maxRSSIYaw > precRSSIYaw)
          precYaw = posCounterYaw;
        
        if(yawPos > posCounterYaw)
          posCounterYaw++;
    
        if(yawPos < posCounterYaw)
          posCounterYaw--;
    
        if(yawPos == posCounterYaw && trackerMode<3){
          stepYaw = 2;
        } else
          stepYaw = 1;
        
        if(posCounterYaw<=0)
          posCounterYaw = 0;
    
        if(posCounterYaw>=180)
          posCounterYaw = 180;
    
        yawServo.write(posCounterYaw);
      }
  
      if(stepYaw >= stepYawCount){
  
        if(posCounterYaw <= precYaw + offset && posCounterYaw >= precYaw - offset){
          yawCountMax += timeToWaitYaw;
          yawDirection != yawDirection;
        } else {
          if(precYaw > posCounterYaw)
            yawDirection = true;
        
          if(precYaw < posCounterYaw)
            yawDirection = false;
        }

      if(posCounterYaw <= 0)
        yawDirection = true;

      if (posCounterYaw>= 180)
        yawDirection = false;
        
        stepYaw = 0;
        lockPitch = true;
      }
    }
  }
}

void calibration(){
  if(rawAnalA>rawAnalB){
    pinAOffset =(rawAnalA-rawAnalB);
    pinBOffset = 0;
  }
   else {
    pinBOffset =(rawAnalB-rawAnalA);
    pinAOffset = 0;
 }
}
