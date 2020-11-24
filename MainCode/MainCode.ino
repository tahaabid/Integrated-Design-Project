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
#define blackLinePinAnalogFR A14  //Far Right (near the wheel)
#define blackLinePinFR 20         //Far Right (near the wheel)
#define blackLinePinAnalogFL A15  //Far Left (near the wheel)
#define blackLinePinFL 21         //Far Left (near the wheel)

bool blackLineTR = 0; //top right
bool blackLineTL = 0; // top left
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
  //ForwardDriveCar();
  driveCar();
  /***Serial.println("F");
  ForwardDriveCar();
  Serial.println("L");
  LeftRotateCar();
  Serial.println("R");
  RightRotateCar();*/

  /*
  stopCar();
  ForwardDriveCar();
  delay(1500);
  stopCar();
  RightRotateCar();
  ForwardDriveCar();
  delay(1500);
  stopCar();
  LeftRotateCar(); */  

  Serial.print(" encoderValueL= ");
  Serial.print(encoderValueL);
  Serial.print(" encoderValueR= ");
  Serial.print(encoderValueR);
  Serial.println("");
  
}

void crossSection(){
  //if ((fl==1) or (fr==1)){
      if (millis() - timer > 400) {
       Serial.println("interrupt entered");
       timer=millis();
       intRaised=1;
       //stopCar();
       if (cd==0){
          x=x+1;
          Serial.print("x= ");
          Serial.println(x);
          informationdisplay();
          //ColorInput();
          //ColorCheck();
       }
       if (cd==1){
          y=y+1;
          Serial.print("y= ");
          Serial.println(y);
          informationdisplay();
          //ColorInput();
          //ColorCheck();
       }
       //delay(1000);
       intRaised=0;
       //digitalWrite(pin1R, 0);
       //digitalWrite(pin2R, 1);
       //digitalWrite(pin1L, 0);
       //digitalWrite(pin2L, 1);
    }
  //}
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
}

void driveCar(){
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
  Serial.print("Starting...");
  
  int flag=0;
  while (true){
    tr=digitalRead(blackLinePinTR);
    tl=digitalRead(blackLinePinTL);
    fr=digitalRead(blackLinePinFR);
    fl=digitalRead(blackLinePinFL);
    //Serial.print("tr and tl: ");
    //Serial.print(tr);
    //Serial.print(" ");
    //Serial.println(tl);

    

    if (intRaised==0){
      if (flag==0){
        digitalWrite(pin1R, 0);
        digitalWrite(pin2R, 1);
        digitalWrite(pin1L, 0);
        digitalWrite(pin2L, 1);
        flag=1; 
      }
      if ((tl==1) and (tr==0)){
        digitalWrite(pin1R, 0);
        digitalWrite(pin2R, 0);
        digitalWrite(pin1L, 0);
        digitalWrite(pin2L, 1);
        Serial.println("turning left");
        while(!((tl==1) and (tr==1))){
          tr=digitalRead(blackLinePinTR);
          tl=digitalRead(blackLinePinTL);
          fr=digitalRead(blackLinePinFR);
          fl=digitalRead(blackLinePinFL);
        };
        digitalWrite(pin1R, 0);
        digitalWrite(pin2R, 1);
        digitalWrite(pin1L, 0);
        digitalWrite(pin2L, 1);
        Serial.println("forward");
      }
      if ((tl==0) and (tr==1)){
        digitalWrite(pin1R, 0);
        digitalWrite(pin2R, 1);
        digitalWrite(pin1L, 0);
        digitalWrite(pin2L, 0);
        Serial.println("turning right");
        while(!((tl==1) and (tr==1))){
          tr=digitalRead(blackLinePinTR);
          tl=digitalRead(blackLinePinTL);
          fr=digitalRead(blackLinePinFR);
          fl=digitalRead(blackLinePinFL);
        };
        digitalWrite(pin1R, 0);
        digitalWrite(pin2R, 1);
        digitalWrite(pin1L, 0);
        digitalWrite(pin2L, 1);
        Serial.println("forward");
      }
    }
    else{
      //stopCar();
      flag=0; 
    }
  }
}


//////////////////////////////////////////////////////
//Input routine
void InputCapture() {
  //black
  blackLineTR = digitalRead(blackLinePinTR);
  blackLineTL = digitalRead(blackLinePinTL);
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

//////////////////////////////////////////////////////
//Motor control
void MotorControl() {
}
void ForwardDriveCar(){
    long target=2;
    long ValueR = encoderValueR;
    long ValueL = encoderValueL;
    long rwheel=0;
    long lwheel=0;
    //inititate wheels to move
    digitalWrite(pin1R, 0);
    digitalWrite(pin2R, 1);
    digitalWrite(pin1L, 0);
    digitalWrite(pin2L, 1);
    //check color in every loop
    ColorInput();
    ColorCheck();
    while(lwheel<target or rwheel<target){
      if (rwheel>target){
        digitalWrite(pin2R, 0);
      }     
      if (lwheel>target){
        digitalWrite(pin2L, 0);
      }
      rwheel=abs(ValueR-encoderValueR);
      lwheel=abs(ValueL-encoderValueL);
      Serial.print("");
      /***Serial.print(" encoderValueL= ");
      Serial.print(lwheel);
      Serial.print(" encoderValueR= ");
      Serial.print(rwheel);
      Serial.println("");*/
    }
  }
void LeftRotateCar(){
    long target=70;
    long ValueR = encoderValueR;
    long ValueL = encoderValueL;
    long rwheel=0;
    long lwheel=0;

    digitalWrite(pin1R, 1);
    digitalWrite(pin2R, 0);
    digitalWrite(pin1L, 0);
    digitalWrite(pin2L, 1);
    
    while(rwheel<target or lwheel<target){
      //turnCarOnsiteL()
      if (rwheel>target){
        digitalWrite(pin1R, 0);
      }     
      if (lwheel>target){
        digitalWrite(pin2L, 0);
      }
      rwheel=abs(ValueR-encoderValueR);
      lwheel=abs(ValueL-encoderValueL);
      Serial.print("");
      /***Serial.print(" encoderValueL= ");
      Serial.print(lwheel);
      Serial.print(" encoderValueR= ");
      Serial.print(rwheel);
      Serial.println("");*/
    }
    stopCar();
  }
 void RightRotateCar(){
    long target = 70;
    long ValueR = encoderValueR;
    long ValueL = encoderValueL;
    long rwheel=0;
    long lwheel=0;

    digitalWrite(pin1R, 0);
    digitalWrite(pin2R, 1);
    digitalWrite(pin1L, 1);
    digitalWrite(pin2L, 0);
    
    while(rwheel<target or lwheel<target){
     //turnCarOnsiteR()
     if (lwheel>target){
        digitalWrite(pin1L, 0);
      }     
      if (rwheel>target){
        digitalWrite(pin2R, 0);
      }
      rwheel=abs(ValueR-encoderValueR);
      lwheel=abs(ValueL-encoderValueL);
      Serial.print("");
      ///Serial.print(" encoderValueL= ");
      /***Serial.print(lwheel);
      Serial.print(" encoderValueR= ");
      Serial.print(rwheel);
      Serial.println("");
      //delay(100);*/
    }
    stopCar();
  }
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
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
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
// Motor subrountine
void turnCarOnsiteL() {
  digitalWrite(pin1R, 1);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void turnCarOnsiteR() {
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
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 1);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 1);
}
void stopCar() {
  digitalWrite(pin1R, 0);
  digitalWrite(pin2R, 0);
  digitalWrite(pin1L, 0);
  digitalWrite(pin2L, 0);
}

//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
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
////////////////////////////////////////////////////


////////////////////////////////////////////////////

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
int colorCheckCnt = 4;//testing value






/////////////////////////////////////////////////
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
<<<<<<< Updated upstream
  //Check Red Color
  if ((50 <= colorValueC && colorValueC <= 170) &&
      (110 <= colorValueR && colorValueR <= 260) &&
      (250 <= colorValueB && colorValueB <= 420) &&
      (300 <= colorValueG && colorValueG <= 500)) {
=======
  Serial.println("Checking color");
  CRratio = colorValueR / colorValueC;
  CBratio = colorValueB / colorValueC;
  CGratio = colorValueG / colorValueC;
  
  //Check Red Color
  if ((101 < colorValueC && colorValueC < 1207) &&
      (1.30 < CRratio && CRratio < 11.84) &&
      (0.44 < CBratio && CBratio < 15.5) &&
      (0.32 < CGratio && CGratio < 5.99)) {
>>>>>>> Stashed changes
    colorCheckCntR++;
  } else {
    colorCheckCntR = 0;
  }
  //Continous detection before notification
  if (colorCheckCntR > colorCheckCnt) {
    //stop the moving car firstly
    stopCar();
    Serial.print(" Red is detected. ");
    Serial.println("\n Turning Left");
    LeftRotateCar();
    colorCheckCntR = colorCheckCnt;
    forwardCar();
    delay(5000);
  }
  //Check Green Color
<<<<<<< Updated upstream
  if ((50 <= colorValueC && colorValueC <= 150) &&
      (110 <= colorValueR && colorValueR <= 320) &&
      (140 <= colorValueB && colorValueB <= 360) &&
      (140 <= colorValueG && colorValueG <= 320)) {
=======
  if ((58 < colorValueC && colorValueC < 1207) &&
      (2.05 < CRratio && CRratio < 12.38) &&
      (0.45 < CBratio && CBratio < 16.45) &&
      (0.32 < CGratio && CGratio < 4.20)) {
>>>>>>> Stashed changes
    colorCheckCntG++;
  } else {
    colorCheckCntG = 0;
  }
  //Continous detection before notification
  if (colorCheckCntG > colorCheckCnt) {
    //Stop the moving car
    stopCar();
    Serial.print(" Green is detected. ");
    Serial.println("\n Turning Right");
    RightRotateCar();
    colorCheckCntG = colorCheckCnt;
    forwardCar();
    delay(5000);
  }
  //Check Blue Color
  if ((95 < colorValueC && colorValueC < 110) &&
      (235 < colorValueR && colorValueR < 270) &&
      (300 < colorValueB && colorValueB < 330) &&
      (205 < colorValueG && colorValueG < 230)) {
    colorCheckCntB++;
  } else {
    colorCheckCntB = 0;
  }
  //Continous detection before notification
  if (colorCheckCntB > colorCheckCnt) {
    Serial.print(" Blue is detected. ");
    colorCheckCntB = colorCheckCnt;
  }
}
