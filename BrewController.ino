// SdFat - Version: Latest 
#include <BlockDriver.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <SysCall.h>
#include <PID_v1.h>
// SD - Version: Latest 
#include <SPI.h>
// MAX6675 library - Version: Latest 
#include <max6675.h>
// Adafruit MLX90614 Library - Version: Latest 
#include <Adafruit_MLX90614.h>
#include <DHT.h>
#include <DHT_U.h>
#include <LedControl.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Timer.h>
#include <Time.h>
#include <TimeLib.h>

//Define Variables we'll be connecting to
double PIDSetpoint, PIDInput, PIDOutput;
//Specify the links and initial tuning parameters
double Kp=1000, Ki=10, Kd=10;
//double Kp=2, Ki=0, Kd=0;
PID myPID(&PIDInput, &PIDOutput, &PIDSetpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 2000;
unsigned long windowStartTime,m;
int PIDout=0;

/* Arduino Pin to Display Pin
   DIN,  CLK, CS ,devices */
LedControl lc=LedControl(12,10,11,3);
LedControl lc1=LedControl(30,34,32,3);

// Data wire is plugged into port 8 on the Arduino
#define ONE_WIRE_BUS1 9
#define ONE_WIRE_BUS2 8

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS1);
OneWire oneWire2(ONE_WIRE_BUS2);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);  
DallasTemperature sensors2(&oneWire2);  
//Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// set up variables using the SD utility library functions:
//  * SD card attached to SPI bus as follows:
// ** MOSI - pin 51 on Arduino Mega
// ** MISO - pin 50 on Arduino Mega
// ** CLK - pin 52 on Arduino Mega
// ** CS - pin 53 on Arduino Mega
SdFat SD;
//Sd2Card card;
//SdVolume volume;
//SdFile root;
//const int chipSelect = 53;
#define SD_CS_PIN 53
File myFile;

// Define PINs
const int led = LED_BUILTIN; //13;
const int rims_pwr = 2;
const int rims_enable = 35;
const int rims_led = 27;
const int rims_pwr_led = 24;
const int rims_up = 46;
const int rims_down = 47;
const int pump_pwr = 5;
const int pump_enable = 38;
const int pump_led = 28;
const int urn_pwr = 3;
const int urn_enable = 37;
const int urn_led = 29;
const int urn_pwr_led = 26;
const int urn_up = 44;
const int urn_down = 45;
const int fridge1_pwr = 6;
const int fridge1_enable = 33;
const int fridge1_led = 25;
const int fridge1_up = 42;
const int fridge1_down = 43;
const int fridge2_pwr = 7;
const int fridge2_led = 23;
const int fridge2_enable = 31;
const int fridge2_up = 40;
const int fridge2_down = 41;

//define displays
const int fridge1_display = 0;
const int fridge2_display = 0;
const int urn_display = 1;
const int rims_display = 1;
const int mash_display = 2;
const int amb_display = 2;
   
Timer t;
boolean toggle=false;
boolean UrnOn = false;
boolean RIMSOn = false;
// DHT
#define DHTPIN 22     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);
// Current Temp & Humidity
float curt,curh;
const int Readings=24;
float Temp[Readings];
int TempY[Readings]; // Y grapgh coordinate
float Hum[Readings];
String sTime[Readings];
String sDate[Readings];
int Count=0;
const int WireReadings=17;
float UrnTemp=0;
float MashTemp=0;
float MashTemp1=0;
float RIMSTemp=0;
float WireFridgeTemp=0;
float WireFridge2Temp=0;
String sWireTime[WireReadings];
String sWireDate[WireReadings];
float UrnTemps[WireReadings];
int WireCount=0;
int UrnSetPoint=05;
int RIMSSetPoint=76;

// Fridge
const int FridgeReadings = 40;
float FridgeTemp[FridgeReadings];
int FridgeTempY[FridgeReadings]; // Y grapgh coordinate
String sFridgeTime[FridgeReadings];
String sFridgeDate[FridgeReadings];
int FridgeCount=0;
int FridgeSetPoint = 18;
int Fridge2SetPoint = 18;
int FermDay = 0;
int LagerProfile[] = {12,12,12,12,12,12,12,12,12,12,12,12,18,18,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,2};
int AleProfile[] = {18,18,18,18,18,18,18,18,18,18,22,22,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};
boolean Lager=false;
boolean FridgeOn = false;
boolean Fridge2On = false;
double anaReading,anaAverage,anaTemp;

void setup() {
  pinMode ( led, OUTPUT );  
  digitalWrite ( led, 0 );
  pinMode ( fridge1_pwr, OUTPUT );
  pinMode ( fridge1_led, OUTPUT );
  pinMode ( fridge1_enable, INPUT_PULLUP  );
  pinMode ( fridge1_up, INPUT_PULLUP );
  pinMode ( fridge1_down, INPUT_PULLUP );
  pinMode ( fridge2_pwr, OUTPUT );
  pinMode ( fridge2_led, OUTPUT );
  pinMode ( fridge2_enable, INPUT_PULLUP  );
  pinMode ( fridge2_up, INPUT_PULLUP );
  pinMode ( fridge2_down, INPUT_PULLUP );
  pinMode ( pump_pwr, OUTPUT );
  pinMode ( pump_led, OUTPUT );
  pinMode ( pump_enable, INPUT_PULLUP );
  pinMode ( urn_pwr, OUTPUT );
  pinMode ( urn_led, OUTPUT );
  pinMode ( urn_pwr_led, OUTPUT );
  pinMode ( urn_enable, INPUT_PULLUP );
  pinMode ( urn_up, INPUT_PULLUP );
  pinMode ( urn_down, INPUT_PULLUP );
  pinMode ( rims_pwr, OUTPUT );
  pinMode ( rims_led, OUTPUT );
  pinMode ( rims_pwr_led, OUTPUT );
  pinMode ( rims_enable, INPUT_PULLUP );
  pinMode ( rims_up, INPUT_PULLUP );
  pinMode ( rims_down, INPUT_PULLUP );
  analogReference(INTERNAL1V1);
  // Init outputs to off
  digitalWrite(rims_led,1);
  digitalWrite(rims_pwr,0);
  digitalWrite(rims_pwr_led,1);
  digitalWrite(urn_led,1);
  digitalWrite(urn_pwr,0);
  digitalWrite(urn_pwr_led,1);
  digitalWrite(fridge1_led,1);
  digitalWrite(fridge1_pwr,1);
  digitalWrite(fridge2_led,1);
  digitalWrite(fridge2_pwr,1);
  digitalWrite(pump_led,1);
  digitalWrite(pump_pwr,1);
  
  Serial.begin ( 9600);
  t.every(5000, doAdjustFridgeTemp);
  t.every(5000, doAdjustFridge2Temp);
  t.every(1000, doReadAnaTemp);
  t.every(300, doSetpointAdjust);
  t.every(500, doReadSwitches);
   
  SdCard();
  SdReadSetpoints();
  
  dht.begin();
  //wake up the MAX72XX from power-saving mode 
   lc.shutdown(0,false);
   lc.shutdown(1,false);
   lc.shutdown(2,false);
   lc1.shutdown(0,false);
   lc1.shutdown(1,false);
   lc1.shutdown(2,false);

   //set a medium brightness for the Leds
   lc.setIntensity(0,5);
   lc.setIntensity(1,8);
   lc.setIntensity(2,8);
   lc1.setIntensity(0,5);
   lc1.setIntensity(1,8);
   lc1.setIntensity(2,8);
   
  //PID setup
  windowStartTime = millis();
  //initialize the variables we're linked to
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
 // Init PID
 // t.every(10, doPIDwrite);
  
  t.every(10000, doReadEnv);
  doReadEnv();
  t.every(1000,doReadOneWire);
  doReadOneWire();
  t.every(1000,doReadOneWire2);
  doReadOneWire2();
  t.every(2500, doDisplay1);
  t.every(300000, doClearDisplay);

  doDisplay1();
}

void loop() {
  t.update();   
  
  // RIMS
  if(digitalRead(rims_enable) == 1 && digitalRead(pump_enable) == 1) // Pump must be running
  {
    myPID.Compute();
    m=millis();
    if (m- windowStartTime > WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if (PIDOutput < m- windowStartTime){
      digitalWrite(rims_pwr, LOW);
      digitalWrite(rims_pwr_led, HIGH);
      PIDout=0;
    } 
    else { //RIMS on
      digitalWrite(rims_pwr, HIGH);
      digitalWrite(rims_pwr_led, LOW);
 //   Serial.println("low");
      PIDout=1;
    }
  }
  else { // RIMS off
    digitalWrite(rims_pwr, LOW);
    digitalWrite(rims_pwr_led, HIGH);
    PIDout=0;   
  }  
  
  //URN
  if(digitalRead(urn_enable) == 1 ) // Only turn on URN if RIMS off
  {
    if((UrnTemp < UrnSetPoint) && (PIDout == 0))  // Turn on urn
    {
      digitalWrite (urn_pwr,1);
      digitalWrite (urn_pwr_led,0);
      UrnOn = true;
    } else  { // Turn off urn Allow for over shoot
      digitalWrite (urn_pwr,0);
      digitalWrite (urn_pwr_led,1);
      UrnOn = false;
    }
  } else { 
    digitalWrite (urn_pwr,0);
    digitalWrite (urn_pwr_led,1);
    UrnOn = false;  
 }
}


void SdCard(){
  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
}


void SdReadSetpoints(){
  myFile = SD.open("setp.txt");
  if (myFile) {
    Serial.println("setp.txt:");
    int b;
    if (myFile.available()) {
      b = myFile.read();
      UrnSetPoint = b;
      Serial.println(UrnSetPoint);
      b = myFile.read(); // read comma
      Serial.println(b,DEC);
      b = myFile.read();
      FridgeSetPoint = b;
      Serial.println(FridgeSetPoint);
      b = myFile.read(); // read comma
      Serial.println(b,DEC);
      b = myFile.read();
      RIMSSetPoint = b;
      PIDSetpoint = (double)RIMSSetPoint;
      Serial.println(RIMSSetPoint);
      b = myFile.read(); // read comma
      Serial.println(b,DEC);
      b = myFile.read();
      Fridge2SetPoint = b;
      Serial.println(Fridge2SetPoint);
    }
    myFile.close();
  } else {
    Serial.println("error opening setp.txt");
  }  
}

void doClearDisplay(void){
  lc.shutdown(0,true);
  lc.shutdown(1,true);
  lc.shutdown(2,true);
  lc1.shutdown(0,true);
  lc1.shutdown(1,true);
  lc1.shutdown(2,true);  
  lc.shutdown(0,false);
  lc.shutdown(1,false);
  lc.shutdown(2,false);
  lc1.shutdown(0,false);
  lc1.shutdown(1,false);
  lc1.shutdown(2,false);
  lc.clearDisplay(urn_display);  
  lc.clearDisplay(fridge2_display);  
  lc.clearDisplay(amb_display);  
  lc1.clearDisplay(fridge1_display); 
  lc1.clearDisplay(mash_display); 
  lc1.clearDisplay(rims_display); 
  //set a medium brightness for the Leds
  lc.setIntensity(0,5);
  lc.setIntensity(1,8);
  lc.setIntensity(2,8);
  lc1.setIntensity(0,5);
  lc1.setIntensity(1,8);
  lc1.setIntensity(2,8);
   
  doDisplay1();
}


void doDisplay1(void){
  printNumber(&lc1,rims_display,RIMSSetPoint,RIMSTemp, 0); 
  printNumber(&lc1,fridge1_display,FridgeSetPoint,WireFridgeTemp, FridgeOn); 
  printNumber3(&lc1,mash_display,MashTemp,anaAverage); 
  printNumber(&lc,urn_display,UrnSetPoint,UrnTemp, 0);
  printNumber(&lc,fridge2_display,Fridge2SetPoint,WireFridge2Temp, Fridge2On); 
  printNumber3(&lc,amb_display,curt,curh); 
}

void doAdjustFridgeTemp(){
  if(digitalRead(fridge1_enable) == 1) 
  {
    if(WireFridgeTemp > FridgeSetPoint) { // Turn on fridge
      digitalWrite ( fridge1_pwr,0);
      FridgeOn = true;
    } else if ((WireFridgeTemp-0.5) < FridgeSetPoint-1) { // Turn off fridge Allow for over shoot
      digitalWrite ( fridge1_pwr,1);
      FridgeOn = false;
    }    
 } else { 
    digitalWrite ( fridge1_pwr,1);
    FridgeOn = false;  
 }
}

void doAdjustFridge2Temp(){
  if(digitalRead(fridge2_enable) == 1) 
  {
    if(WireFridge2Temp > Fridge2SetPoint) { // Turn on fridge
      digitalWrite ( fridge2_pwr,0);
      Fridge2On = true;
    } else if ((WireFridge2Temp-0.5) < Fridge2SetPoint-1) { // Turn off fridge Allow for over shoot
      digitalWrite ( fridge2_pwr,1);
      Fridge2On = false;
    }    
 } else { 
    digitalWrite ( fridge2_pwr,1);
    Fridge2On = false;  
 }
}

void doReadAnaTemp(){
  anaReading = analogRead(0);
  anaReading = anaReading / 9.31;
  anaAverage = ( anaAverage * 9 + anaReading ) /10; 
 // Serial.println(anaAverage);
}

/*
void doAdjustRIMSTemp(){
 //  Serial.println(WireCount);
  if(RIMSTemp < RIMSSetPoint) { // Turn on urn
    digitalWrite ( rims_pwr,0);
    RIMSOn = true;
  } else if (RIMSTemp > RIMSSetPoint-1) { // Turn off urn Allow for over shoot
    digitalWrite ( rims_pwr,1);
    RIMSOn = false;
  }
}*/

void doReadEnv() {
  curh = dht.readHumidity();
  curt = dht.readTemperature();
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(curt) || isnan(curh)) {
    Serial.println("Failed to read from DHT");
  } else {
    Serial.print("H: ");
    Serial.print(curh);
    Serial.print(" %\t");
    Serial.print("T: ");
    Serial.print(curt);
    Serial.println(" *C");
  }  
}

void doReadSwitches() {
  //RIMS
//  Serial.print(digitalRead (urn_enable));
  if(digitalRead (rims_enable) == 1){
      digitalWrite(rims_led, 0);
    }
  else {
    digitalWrite(rims_led, 1);
  }
  //URN
  if(digitalRead (urn_enable) == 1){
      digitalWrite(urn_led, 0);
//      digitalWrite(urn_pwr, 0);
    }
  else {
    digitalWrite(urn_led, 1);
//    digitalWrite(urn_pwr, 1);
  }
  //Fridge1
  if(digitalRead (fridge1_enable) == 1){
      digitalWrite(fridge1_led, 0);
    }
  else {
    digitalWrite(fridge1_led, 1);
  }
  //Fridge2
  if(digitalRead (fridge2_enable) == 1){
      digitalWrite(fridge2_led, 0);
    }
  else {
    digitalWrite(fridge2_led, 1);
  }
  //Pump
  if(digitalRead (pump_enable) == 1){
      digitalWrite(pump_led, 0);
      digitalWrite(pump_pwr, 0);
    }
  else {
    digitalWrite(pump_led, 1);
    digitalWrite(pump_pwr, 1);
  }
}

void doSetpointAdjust(){
  boolean change=false;
  //urn
  if(digitalRead ( urn_up) == 0){
    if(UrnSetPoint < 99) {
      UrnSetPoint++;
      change = true;
    }
  }
    if(digitalRead ( urn_down) == 0){
    if(UrnSetPoint > 0) {
      UrnSetPoint--;
      change = true;
    }
  }
  //RIMS
  if(digitalRead ( rims_up) == 0){
    if(RIMSSetPoint < 99) {
      RIMSSetPoint++;
      change = true;
    }
  }
  if(digitalRead ( rims_down) == 0){
    if(RIMSSetPoint > 0) {
       RIMSSetPoint--;
      change = true;
    }
  }
  //Fridge1
  if(digitalRead ( fridge1_up) == 0){
    if(FridgeSetPoint < 99) {
      FridgeSetPoint++;
      change = true;
    }
  }
  if(digitalRead ( fridge1_down) == 0){
    if(FridgeSetPoint > 0) {
      FridgeSetPoint--;
      change = true;
    }
  }
  //Fridge2
  if(digitalRead ( fridge2_up) == 0){
    if(Fridge2SetPoint < 99) {
      Fridge2SetPoint++;
      change = true;
    }
  }
  if(digitalRead ( fridge2_down) == 0){
    if(Fridge2SetPoint > 0) {
      Fridge2SetPoint--;
      change = true;
    }
  }
 
  if(change) {
    PIDSetpoint = (double)RIMSSetPoint;
    doDisplay1();
    myFile = SD.open("setp.txt", O_CREAT | O_TRUNC | O_RDWR);

    // if the file opened okay, write to it:
    if (myFile) {
      Serial.print("Writing to setp.txt");
      myFile.write(UrnSetPoint);
      myFile.write(",");
      myFile.write(FridgeSetPoint);
      myFile.write(",");
      myFile.write(RIMSSetPoint);
      myFile.write(",");
      myFile.write(Fridge2SetPoint);
      // close the file:
      myFile.close();
 //     Serial.println("done.");
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening setp.txt");
    }
  }
}


void doReadOneWire(void)
{
   // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
//  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
//  Serial.println("DONE");
  
//  Serial.println("Temperature for the device 1 (index 0) is: ");
  WireFridgeTemp = sensors.getTempCByIndex(0);
//  Serial.println(WireFridgeTemp);  
  UrnTemp = sensors.getTempCByIndex(1);
//  Serial.println(UrnTemp); 
  RIMSTemp = sensors.getTempCByIndex(2);
//  Serial.println(WireFridge2Temp); 
}

void doReadOneWire2(void)
{
  sensors2.requestTemperatures(); // Send the command to get temperatures
//  Serial.println(sensors2.getTempCByIndex(0));  
  WireFridge2Temp = sensors2.getTempCByIndex(0);
//  Serial.println(RIMSTemp); 
  PIDInput = (double)RIMSTemp;
 // Serial.println(sensors2.getTempCByIndex(1));  
  MashTemp = sensors2.getTempCByIndex(1);
//  Serial.println(MashTemp); 
}

String GetTimeNow(void)
{
   String sTime;
   if(hour() < 10)
     sTime.concat('0');
   sTime.concat(hour());
   sTime.concat(':');
   if(minute() < 10)
     sTime.concat('0');
   sTime.concat(minute());
   sTime.concat(':');
   if(second() < 10)
     sTime.concat('0');
   sTime.concat(second());
 //  Serial.println(sTime);
   return sTime;
}


String GetDateNow()
{
   String sDate;
   if(day() < 10)
     sDate.concat('0');
   sDate.concat(day());
   sDate.concat('-');
   if(month() < 10)
     sDate.concat('0');
   sDate.concat(month());
   sDate.concat('-');
   sDate.concat(year());
//   Serial.println(sDate);
   return sDate;
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void printNumber(LedControl *lcp, int display, int v1, float v2, boolean on) {
    int ones;
    int tens;
    int tenths;
    int hundredths;
    boolean negative;  

    ones=(int)v1%10;
    tens=(int)v1/10;
    tens=tens%10;
     if(on) {
       //print character '-' in the leftmost column 
       lcp->setChar(display,5,'o',false);
       lcp->setChar(display,4,'n',false);
    }
    else {
       //print a blank in the sign column
       lcp->setChar(display,5,' ',false);
       lcp->setChar(display,4,' ',false);
    }
    //Now print the number digit by digit
    if(v1==100) { // Do not display digits
       lcp->setChar(display,7,' ',false);
       lcp->setChar(display,6,' ',false);      
    } else {
      lcp->setDigit(display,7,(byte)tens,false);
      lcp->setDigit(display,6,(byte)ones,false);
    }
    
    ones=(int)v2%10;
    tens=(int)(v2/10)%10;
  
    lcp->setDigit(display,3,(byte)tens,false);
    lcp->setDigit(display,2,(byte)ones,true);
    v2=(v2*100);
    hundredths=(int)v2%10;
    tenths=(int)(v2/10)%10;
    lcp->setDigit(display,1,(byte)tenths,false);
    lcp->setDigit(display,0,(byte)hundredths,false);
}


void printNumber3(LedControl *lcp, int display, float v1, float v2) {
    int ones;
    int tens;
    int hundreds;
    int tenths;
    int hundredths;
    boolean negative;  

  
    ones=(int)v1%10;
    tens=(int)(v1/10)%10;
    hundreds=(int)(v1/100)%10;
    if(hundreds > 0){
      lcp->setChar(display,7,(byte)hundreds,false);
    }else{
      lcp->setChar(display,7,' ',false);
    }
    lcp->setDigit(display,6,(byte)tens,false);
    lcp->setDigit(display,5,(byte)ones,true);
    v1=(v1*100);
    hundredths=(int)v1%10;
    tenths=(int)(v1/10)%10;
    lcp->setDigit(display,4,(byte)tenths,false);
   
    
    ones=(int)v2%10;
    tens=(int)(v2/10)%10;
    hundreds=(int)(v2/100)%10;
    if(hundreds > 0){
      lc.setChar(display,3,(byte)hundreds,false);
    }else{
      lcp->setChar(display,3,' ',false);
    }    
    lcp->setDigit(display,2,(byte)tens,false);
    lcp->setDigit(display,1,(byte)ones,true);
    v2=(v2*100);
    hundredths=(int)v2%10;
    tenths=(int)(v2/10)%10;
    lcp->setDigit(display,0,(byte)tenths,false);
}

void doPIDwrite(){
   Serial.print(PIDout);
  Serial.print(",");
  Serial.print(PIDOutput);
  Serial.print(",");
  Serial.print(m - windowStartTime);
  Serial.print(",");
  Serial.print(PIDInput);
  Serial.print(",");
  Serial.print(PIDSetpoint);
  Serial.println();
}
