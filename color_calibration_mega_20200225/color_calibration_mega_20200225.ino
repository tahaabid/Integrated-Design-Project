//Color sensor
//Pin out
#define colorpin_out 26 //brown
#define colorpin_s2 27 //orange
#define colorpin_s3 28 //yellow
#define colorpin_s1 29 //purple
#define colorpin_s0 30 //blue
#define colorpin_LED 31 //green

//parameter
int colorValueC = 0;
int colorValueR = 0;
int colorValueG = 0;
int colorValueB = 0;
int colorCnt = 0; //each loop only detect one color value
int colorCheckCntR = 0; //check several times before detection
int colorCheckCntG = 0;
int colorCheckCntB = 0;
int colorCheckCnt = 2;


void setup()
{
  Serial.begin(115200);

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
}

void loop()
{
  for (int x=0; x<4; x++){
    ColorInput();
  }
  //Serial.print(" colorCRBG= ");
  Serial.print(colorValueC);
  Serial.print(",");
  Serial.print(colorValueR);
  Serial.print(",");
  Serial.print(colorValueB);
  Serial.print(",");
  Serial.print(colorValueG);
  //Serial.print("");
  //ColorCheck();
  Serial.println("");
  delay(200);
}

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
  
}
void ColorCheck() {
  //Check Red Color
  if ((4 < colorValueC && colorValueC < 9) &&
      (4 < colorValueR && colorValueR < 12) &&
      (15 < colorValueB && colorValueB < 24) &&
      (25 < colorValueG && colorValueG < 34)) {
    colorCheckCntR++;
  } else {
    colorCheckCntR = 0;
  }
  //Continous detection before notification
  if (colorCheckCntR > colorCheckCnt) {
    Serial.print(" Red is detected. ");
    colorCheckCntR = colorCheckCnt;
  }
  //Check Green Color
  if ((125 < colorValueC && colorValueC < 155) &&
      (280 < colorValueR && colorValueR < 350) &&
      (370 < colorValueB && colorValueB < 440) &&
      (260 < colorValueG && colorValueG < 300)) {
    colorCheckCntG++;
  } else {
    colorCheckCntG = 0;
  }
  //Continous detection before notification
  if (colorCheckCntG > colorCheckCnt) {
    Serial.print(" Green is detected. ");
    colorCheckCntG = colorCheckCnt;
  }
  //Check Blue Color
  if ((3 < colorValueC && colorValueC < 7) &&
      (17 < colorValueR && colorValueR < 26) &&
      (6 < colorValueB && colorValueB < 10) &&
      (8 < colorValueG && colorValueG < 18)) {
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
