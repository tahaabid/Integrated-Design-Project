//Main by Saad and Taha
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_SSD1331.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

//Mega2560 pin 20 (SDA), pin 21 (SCL)
//SPI pin
// MOSI 51
//MISO 50
//SCK 52
//SS 53

///////////////////////////////////////////////////////
//Color sensor
#define colorpin_out 26 //brown
#define colorpin_s2 27 //orange
#define colorpin_s3 28 //yellow
#define colorpin_s1 29 // purple
#define colorpin_s0 30 //blue
#define colorpin_LED 31 //LED

///////////////////////////////////////////////////////
//Black Line
//see black tape is 1, LED off
//see white is 0, LED on
#define blackLinePinAnalogTR A12   //Top Right
#define blackLinePinTR 42         //Top Right
#define blackLinePinAnalogTL A13   //Top Left
#define blackLinePinTL 43         //Top Left

//#define blackLinePinAnalogTR A12   //Top Right Right
#define blackLinePinTRR 40         //Top Right Right
//#define blackLinePinAnalogTL A13   //Top Left Left
#define blackLinePinTLL 41         //Top Left Left

#define blackLinePinAnalogFR A14  //Far Right (near the wheel)
#define blackLinePinFR 20         //Far Right (near the wheel)
#define blackLinePinAnalogFL A15  //Far Left (near the wheel)
#define blackLinePinFL 21         //Far Left (near the wheel)

bool prevBlackLineTR=0;
bool prevBlackLineTL=0;
bool blackLineTR = 0; //top right
bool blackLineTL = 0; // top left
bool blackLineTRR = 0; //top right right
bool blackLineTLL = 0; // top left left
bool blackLineFR = 0; //far right
bool blackLineFL = 0; //far left
int  blackLineAnalogTR = 0; //top right
int  blackLineAnalogTL = 0; //top left
int  blackLineAnalogFR = 0; //far right
int  blackLineAnalogFL = 0; //far left

///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// Photo Diode
#define photoPinAnalogR  A8//Photo Right
#define photoPinR 38  //Photo Right
#define photoPinAnalogL  A9//Photo Left
#define photoPinL  39  //Photo Left

bool photoR = 0; //right
bool photoL = 0; //left
int photoAnalogR = 0; //right
int photoAnalogL = 0; //left
///////////////////////////////////////////////////////
//Power detection
#define powerValuePin A7
int  powerValue = 0;

///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
//Ultra Sonic
#define trigPin 13
#define echoPin 12

unsigned long UltraSonicStartTime = 0;
int UltraSonicDone = 1;
int duration;
float distance;
int pwr;
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
//interrupt pin
int interruptL1 = 2;
int interruptL2 = 3;

int interruptR1 = 18;
int interruptR2 = 19;
///////////////////////////////////////////////////////

//////////////////////////////////////////////////////
//motor driver LM298 control pin
int pin1R = 8;
int pin2R = 9;
int pin1L = 10;
int pin2L = 11;
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
// encoder parameter
//encoderValue
long encoderValueR = 0;
long encoderValueL = 0;

//Reset encoderValue
int encoderValueR_Reset = 0;
int encoderValueL_Reset = 0;

//encoderValue Diff
long encoderValueR_Diff = 0;
long encoderValueL_Diff = 0;
//////////////////////////////////////////////////////

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//Mega2560 pin 20 (SDA), pin 21 (SCL)
//SPI pin
// MOSI 51
//MISO 50
//SCK 52
//SS 53


////////////////////////////////////////////////////
//Display

// Color definitions
#define  BLACK           0x0000
#define BLUE            0x0006
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF
#define BACKGROUND      0x0000


#define Sclk 52 //change to 52 from 13 from above--- connect this to the display module CLK pin (Serial Clock)
#define Mosi 51 //change to 51 from 11--- connect this to the display module DIN pin (Serial Data)
#define Rst  5 //change to 5 from 9--- connect this to the display module RES pin (Reset)
#define Dc   6 //change to 6 from 8--- connect this to the display module D/C  pin (Data or Command)
#define Cs   7 //change to 7 from 10--- connect this to the display module CS  pin (Chip Select)

Adafruit_SSD1331 display = Adafruit_SSD1331(Cs, Dc, Mosi, Sclk, Rst);
int x = 0;
int y = 0;


long int timer=millis();

int tr;
int tl;
int fr;
int fl;
int intRaised=0;
int cd=0;
//////////////////////////////////////////////////////

int GreenCheck = 0;
int RedCheck = 0;
int forwardStart = 0;
int rightStart = 0;
int leftStart = 0;
int targetValue = 0;
int ColourCh = 0; //green = 1, red = 2, no colour = 0
int direc = 1; //East = 1, North = 2, West = 3, South = 4
bool backOnLine=true;
//////////////////////////////////////////////////////








void setup()
{
  Serial.begin(9600);
  /////////////////////////////////////////
  
  //Color sensor setup
  pinMode(colorpin_LED, OUTPUT);
  pinMode(colorpin_out, INPUT);
  pinMode(colorpin_s0, OUTPUT);
  pinMode(colorpin_s1, OUTPUT);
  pinMode(colorpin_s2, OUTPUT);
  pinMode(colorpin_s3, OUTPUT);
  // turn on LED
  digitalWrite(colorpin_LED, 1);
  //100% freq
  digitalWrite(colorpin_s0, 1);
  digitalWrite(colorpin_s1, 1);
  //detect red
  digitalWrite(colorpin_s2, 1);
  digitalWrite(colorpin_s3, 0);
  /////////////////////////////////////////

  /////////////////////////////////////////
  //Black Line sensor
  pinMode(blackLinePinTR, INPUT);
  pinMode(blackLinePinTL, INPUT);
  pinMode(blackLinePinTRR, INPUT);
  pinMode(blackLinePinTLL, INPUT);
  pinMode(blackLinePinFR, INPUT);
  pinMode(blackLinePinFL, INPUT);
  /////////////////////////////////////////

  /////////////////////////////////////////
  //Photo sensor
  pinMode(photoPinR, INPUT);
  pinMode(photoPinL, INPUT);
  /////////////////////////////////////////

  /////////////////////////////////////////
  //Ultra Sonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  /////////////////////////////////////////

  /////////////////////////////////////////
  //Motor
  pinMode(pin1L, OUTPUT);
  pinMode(pin2L, OUTPUT);
  pinMode(pin1R, OUTPUT);
  pinMode(pin2R, OUTPUT);
  /////////////////////////////////////////
  
  /////////////////////////////////////////
  
  //Encoder
  //https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  pinMode(interruptL1, INPUT_PULLUP);
  pinMode(interruptL2, INPUT_PULLUP);
  pinMode(interruptR1, INPUT_PULLUP);
  pinMode(interruptR2, INPUT_PULLUP);
  //setup interrupt
  attachInterrupt(digitalPinToInterrupt(interruptL1), countL, FALLING);
  attachInterrupt(digitalPinToInterrupt(interruptR1), countR, FALLING);
  
  pinMode(colorpin_LED, OUTPUT);
  pinMode(colorpin_out, INPUT);
  pinMode(colorpin_s0, OUTPUT);
  pinMode(colorpin_s1, OUTPUT);
  pinMode(colorpin_s2, OUTPUT);
  pinMode(colorpin_s3, OUTPUT);

  // turn on LED
  digitalWrite(colorpin_LED, 1);

  //100% freq
  digitalWrite(colorpin_s0, 1);
  digitalWrite(colorpin_s1, 1);
  //detect red
  digitalWrite(colorpin_s2, 1);
  digitalWrite(colorpin_s3, 0);

  //////////////////////////////////////
  //Display 
  display.begin();
  display.fillScreen(BLACK);
  display.setTextColor(WHITE);  
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print("Ready to measure");

  attachInterrupt(digitalPinToInterrupt(blackLinePinFR), crossSection, RISING);
  attachInterrupt(digitalPinToInterrupt(blackLinePinFL), crossSection, RISING);
  
  
  delay(1000);
  informationdisplay();
}












void loop()
{
  Serial.print("Main Loop Running...");
  InputCapture();
  //turn90R();
  //move_car_forward();
  //turn90R();
  //coordinateControl();
  for (int x=0; x<1; x++){
    for (int x=0; x<4; x++){
      ColorInput();
    }
    ColorCheck();
  }
  MotorControl();
}











void informationdisplay(void) {
  display.fillScreen(WHITE);
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(2, 0);
  display.println("Coordinates: ");
  display.print("(");
  display.print(x);
  display.print(", ");
  display.print(y);
  display.println(")");
  display.print("Direction: ");
  display.println(direc);
}




//////////////////////////////////////////////////////
//Input routine
void InputCapture() {
  //black
  prevBlackLineTR=blackLineTR;
  prevBlackLineTL=blackLineTL;
  blackLineTR = digitalRead(blackLinePinTR);
  blackLineTL = digitalRead(blackLinePinTL);
  blackLineTRR = digitalRead(blackLinePinTRR);
  blackLineTLL = digitalRead(blackLinePinTLL);
  
  blackLineFR = digitalRead(blackLinePinFR);
  blackLineFL = digitalRead(blackLinePinFL);
  blackLineAnalogTR = analogRead(blackLinePinAnalogTR);
  blackLineAnalogTL = analogRead(blackLinePinAnalogTL);
  blackLineAnalogFR = analogRead(blackLinePinAnalogFR);
  blackLineAnalogFL = analogRead(blackLinePinAnalogFL);
  //photo diode
  photoR     = digitalRead(photoPinR);
  photoL     = digitalRead(photoPinL);
  photoAnalogR = analogRead(photoPinAnalogR);
  photoAnalogL = analogRead(photoPinAnalogL);
  //Power Sensor
  powerValue = analogRead(powerValuePin);
}
//////////////////////////////////////////////////////

//Coordinate control
void coordinateControl() {
  //delay(100);
  if ((x == 2) and (y==0) and (direc==1)) {
    backOnLine=false;
    turn90L(55, true);
    delay(100);
  }
  else if ((x == 2) and (y==4) and (direc==2)) {
    backOnLine=false;
    turn90R(55, true);
    delay(100);
  }
  else if ((x == 4) and (y==4) and (direc==1)) {
    backOnLine=false;
    turn90R(55, true);
    delay(100);
  }
  else if ((x == 4) and (y==1) and (direc==4)) {
    backOnLine=false;
    turn90R(55, true);
    delay(10000);
  }
  else{
    if (blackLineTL==1 and blackLineTR==0){
            //turn90L(1);
            stopCar();
            forwardL();
    }
    else if (blackLineTL==0 and blackLineTR==1){
            //turn90R(1);
            stopCar();
            forwardR();
    }
    else if (blackLineTL==1 and blackLineTR==1){
            stopCar();
            backOnLine=true;
            move_car_forward(5);
            //forwardCar();
    }
    else if (blackLineTL==0 and blackLineTR==0){
            if (blackLineTL==0 and blackLineTR==0){
              stopCar();
              move_car_forward(2);
              //forwardCar();
            }
            if (blackLineTLL==1){
              stopCar();
              //forwardL();
              turn90L(3, false);
            }
            else if (blackLineTRR==1){
              stopCar();
              //forwardR();
              turn90R(3, false);
            }
      
    }
  }
}
//////////////////////////////////////////////////////
//Motor control
void MotorControl() {
  //delay(100);
  if (ColourCh == 1) {
    //delay(100);
    Serial.print("green turn");
    turn90R(56, true);
    //delay(200);
  }
  else if (ColourCh == 2) {
    //delay(100);
    Serial.print("red turn");
    turn90L(56, true);
    //delay(200);
  }
  else if (ColourCh == 0) {
    if (blackLineTL==1 and blackLineTR==0){
            //turn90L(1);
            stopCar();
            forwardL();
    }
    else if (blackLineTL==0 and blackLineTR==1){
            //turn90R(1);
            stopCar();
            forwardR();
    }
    else if (blackLineTL==1 and blackLineTR==1){
            stopCar();
            backOnLine=true;
            move_car_forward(5);
            //forwardCar();
    }
    else if (blackLineTL==0 and blackLineTR==0){
            if (blackLineTL==0 and blackLineTR==0){
              stopCar();
              move_car_forward(2);
              //forwardCar();
            }
            if (blackLineTLL==1){
              stopCar();
              //forwardL();
              turn90L(3, false);
            }
            else if (blackLineTRR==1){
              stopCar();
              //forwardR();
              turn90R(3, false);
            }
      
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt subroutine
void countL() {
  if (digitalRead(interruptL2)) {
    encoderValueL--;
  }
  else {
    encoderValueL++;
  }
}


void countR() {
  if (digitalRead(interruptR2)) {
    encoderValueR++;
  }
  else {
    encoderValueR--;
  }
}


void crossSection(){
      if (backOnLine){
      if (millis() - timer > 2000) {
      //if (blackLinePinFR==1 or blackLinePinFL==1){ 
       Serial.println("interrupt entered");
       timer=millis();
       if (direc==1){
          x=x+1;
          Serial.print("x= ");
          Serial.println(x);
          informationdisplay();
       }
       if (direc==2){
          y=y+1;
          Serial.print("y= ");
          Serial.println(y);
          informationdisplay();
       }
       if (direc==3){
          x=x-1;
          Serial.print("x= ");
          Serial.println(x);
          informationdisplay();
       }
       if (direc==4){
          y=y-1;
          Serial.print("y= ");
          Serial.println(y);
          informationdisplay();
       }
       stopCar();
       delay(200);
       //move_car_forward(10);
      //}
    }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Motor subrountine
void forwardR() {
  analogWrite(pin1R, 0);
  analogWrite(pin2R, 250);
  analogWrite(pin1L, 0);
  analogWrite(pin2L, 40);
}
void backwardR() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
}
void stopR() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
}
void forwardL() {
  analogWrite(pin1L, 0);
  analogWrite(pin2L, 250);
  analogWrite(pin1R, 0);
  analogWrite(pin2R, 40);
}
void backwardL() {
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void stopL() {
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
void turnCarR() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void turnCarL() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
void turnCarL_Backward() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
void turnCarR_Backward() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void turnCarOnsiteR() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void turnCarOnsiteL() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void forwardCar() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void backwardCar() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void stopForwardCar() {
  backwardCar();
  delay(150);
  stopL();
  stopR();
}
void stopBackwardCar() {
  forwardCar();
  delay(100);
  stopL();
  stopR();
}
void stopTurnCarOnsiteR() {
  turnCarOnsiteR();
  delay(100);
  stopL();
  stopR();
}
void stopTurnCarOnsiteL() {
  turnCarOnsiteL();
  delay(100);
  stopL();
  stopR();
}
void stopCar() {
  stopL();
  stopR();
}
void forwardRightWheel() {
 Serial.print(" |fRW| ");
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
}
void forwardLeftWheel() {
 Serial.print(" |fLW| ");
 digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void backwardRightWheel() {
 Serial.print(" |bRW| ");
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
}
void backwardLeftWheel() {
 Serial.print(" |bLW| ");
  digitalWrite(pin1L, 1);
  digitalWrite(pin2L, 0);
}
void stopRightWheel() {
 Serial.print(" |sRW| ");
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
}
void stopLeftWheel() {
 Serial.print(" |sLW| ");
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turn90L(int tval, bool t) {
  
  //turn right 90 degree
  if (t){
   if(direc == 1) { direc = 2;}
    else if(direc == 2) { direc = 3;}
    else if(direc == 3) { direc = 4;}
    else if(direc == 4) { direc = 1;}
  }
  Serial.print("turning right");
  if (rightStart == 0) {
    stopCar();
    targetValue = tval;
    encoderValueL = 0;
    encoderValueR = 0;
    rightStart = 1;
  }
  while(rightStart!=0){
  if ((abs(encoderValueL) > targetValue) or (abs(encoderValueR) > targetValue)) {
    stopCar();
    ColourCh = 0;
    rightStart = 0;
    //move_car_forward(50);
  }
  else {
    if ((abs(encoderValueL)) < (abs(encoderValueR))) {
      forwardLeftWheel();
      stopRightWheel();
    }
    else if (abs(encoderValueL) > abs(encoderValueR)) {
      backwardRightWheel();
      stopLeftWheel();
    }
    else {
      turnCarOnsiteR();
    }
  }
  }
  //delay(100);
  
}

void move_car_forward(int tval) {
  //forward 100 click
  if (forwardStart == 0){
    stopCar();
    targetValue = tval;
    encoderValueL = 0;
    encoderValueR = 0;
    forwardStart = 1;
  }
  while(forwardStart!=0){
    Serial.print("Encoder Left:  ");
    Serial.println(encoderValueL);
    Serial.print("Encoder Right:  ");
    Serial.println(encoderValueR);
  if (encoderValueL > targetValue or encoderValueR > targetValue) {
    stopCar();
    forwardStart = 0;
  }
  else {
    if (encoderValueL > encoderValueR) {
      turnCarL();
    }
    else if (encoderValueR > encoderValueL) {
      turnCarR();
    }
    else {
      forwardCar();
    }
  }
  }
}

void turn90R(int tval, bool t) {

 //turn left 90 degree
 if (t){
    if(direc == 1) { direc = 4;}
    else if(direc == 2) { direc = 1;}
    else if(direc == 3) { direc = 2;}
    else if(direc == 4) { direc = 3;}
  }
 Serial.print("turning left");
  if (leftStart == 0){
 Serial.print("leftStart");
    stopCar();
    targetValue = tval;
    encoderValueL = 0;
    encoderValueR = 0;
    leftStart = 1;
  }
  while(leftStart!=0){
  if (abs(encoderValueR) > targetValue or abs(encoderValueL) > targetValue) {
    Serial.print("stopCar()");
    stopCar();
    ColourCh = 0;
    leftStart = 0;
    //move_car_forward(50);
  }
  else {
    if (abs(encoderValueR) < abs(encoderValueL)) {
      Serial.print("abs(encoderValueR) < abs(encoderValueL)");
      forwardRightWheel();
      stopLeftWheel();
    }
    else if (abs(encoderValueR) > abs(encoderValueL)) {
      Serial.print("abs(encoderValueR) > abs(encoderValueL)");
      backwardLeftWheel();
      stopRightWheel();
    }
    else {
      Serial.print("turnCarOnsiteL()");
      turnCarOnsiteL();
    }
  }
  }

  //delay(100);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//UltraSonic subroutine
void UltraSonic() {
  if (UltraSonicDone) {
    UltraSonicDone = 0;
    UltraSonicStartTime = millis();
    digitalWrite(trigPin, LOW);  // Added this line
  }
  if (millis() > UltraSonicStartTime + 2) {
    digitalWrite(trigPin, HIGH);
  }
  if (millis() > UltraSonicStartTime + 12) {
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 3000); //set 3000ns as timout
    distance = (duration / 2) / 29.1;
    UltraSonicDone = 1;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int colorValueC = 0;
int colorValueR = 0;
int colorValueG = 0;
int colorValueB = 0;
double CRratio = 0;
double CGratio = 0;
double CBratio = 0;
int colorCnt = 0; //each loop only detect one color value
int colorCheckCntR = 0; //check several times before detection
int colorCheckCntG = 0;
int colorCheckCntB = 0;
int colorCheckCnt = 1;//testing value

void ColorInput() {
  if (colorCnt == 0) {
    //read Clear value
    colorValueC = pulseIn(colorpin_out, LOW);
    //Set Red filter
    digitalWrite(colorpin_s2, 0);
    digitalWrite(colorpin_s3, 0);
    colorCnt++;
  }
  else if (colorCnt == 1) {
    //read Red value
    colorValueR = pulseIn(colorpin_out, LOW);
    //Set Blue filter
    digitalWrite(colorpin_s2, 0);
    digitalWrite(colorpin_s3, 1);
    colorCnt++;
  }
  else if (colorCnt == 2) {
    //read Blue value
    colorValueB = pulseIn(colorpin_out, LOW);
    //Set Green filter
    digitalWrite(colorpin_s2, 1);
    digitalWrite(colorpin_s3, 1);
    colorCnt++;
  }
  else {
    //read Green value
    colorValueG = pulseIn(colorpin_out, LOW);
    //Set Clear filter
    digitalWrite(colorpin_s2, 1);
    digitalWrite(colorpin_s3, 0);
    colorCnt = 0;
  }
  Serial.print(" colorCRBG= ");
  Serial.print(colorValueC);
  Serial.print(" ");
  Serial.print(colorValueR);
  Serial.print(" ");
  Serial.print(colorValueB);
  Serial.print(" ");
  Serial.print(colorValueG);
  Serial.print(" ");
}

void ColorCheck() {
  //Serial.println("Checking color"M);
  CRratio = (colorValueR*1.0) / colorValueC;
  CBratio = (colorValueB*1.0) / colorValueC;
  CGratio = (colorValueG*1.0) / colorValueC;
    
    //Check Red Color
  if ((150 < colorValueC && colorValueC < 390) &&
      (0.89 <= CRratio && CRratio <= 1.61) &&
      (1.96 <= CBratio && CBratio <= 3.73) &&
      (2.57 <= CGratio && CGratio <= 4.71)) {
    colorCheckCntR++;
  } else {
    colorCheckCntR = 0;
  }

  
  //Continous detection before notification
  if (colorCheckCntR > colorCheckCnt) {
    Serial.print(" Red is detected. ");
    ColourCh=2;
    colorCheckCntR = colorCheckCnt;
  }
  //Check Green Color
  if ((100 <= colorValueC && colorValueC <= 350) &&
      (1.36 <= CRratio && CRratio <= 2.82) &&
      (2.57 <= CBratio && CBratio <= 3.8) &&
      (1.84 <= CGratio && CGratio <= 4.71)) {
    colorCheckCntG++;
  } else {
    colorCheckCntG = 0;
  }
  //Continous detection before notification
  if (colorCheckCntG > colorCheckCnt) {
    Serial.print(" Green is detected. ");
    ColourCh=1;
    colorCheckCntG = colorCheckCnt;
  }

}
