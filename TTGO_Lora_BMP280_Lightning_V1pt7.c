/*****
 * M Hamby Jan 21 , 2023 pulled together for TTGO V3.2 LOra with BMP280 and Lightning detector
 * adjusted for our location 
 * Poffset_HPA = 4.00;  adjust pressure reading to match local airport
 * Nominal_HPA = 981.51;  atmospheric elevation correction
 * alarm levels via local levels observed from surrounding weather stations
 * for Entertainment Only!
 * Use only as directed, don't take if you alergic to it..
 * Side effects - smug sense of weather awareness superiority
 * Some assembly requried
 * 
 ***********/
/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/ttgo-lora32-sx1276-arduino-ide/
*********/
/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x32 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/
/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
  See the LICENSE file for details.
 ***************************************************************************/
/*

  This example demonstrates how to detect lightning! It has a few basic
  settings to help with rejecting noise or "disturbers" (false lightning events). 
  It uses the onboard interrupt hardware pin, so in addition to attaching to
  it data lines you'll need to connnect to the interrupt pin labled "INT". 

  By: Elias Santistevan
  SparkFun Electronics
  Date: May, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

*/
/*************************************************************************
 * Version 1.5  D 14Jan2023 MAH - BMP280/AS3935 
 * workig OLED - some pressure indication graphics..
 * addition of lightning detection module -  graphics pressure display cleanup
 * to be done 
 *[X] add lighting readout on OLED display
 *[ ] incorporate level indicated for pressure or lighting distance..(Note 1)
 *[ ] incorporate Encoder/Button (Note 2)
 *[ ] incorporate menu/settins INote 3)
 *[ ] incorporate WIFI UDP unicast
 *[ ] incorporate WIFI AP webpage
 *[ ] incorporate LORa wireless transmitt
 *[ ] build LOra remote reciever hand held..
Notes: 1, 2, and 3 - all these features would more easily be added to a unit webpage
with the exp32 acting as a Access Point (Stand Alone)
*************************************************************************/
//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Libraries for Sensors and barometric
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Libraries for Lightning detector 
#include "SparkFun_AS3935.h"

// Settings adjustment for Lightning detector.. 

// 0x03 is default, but the address can also be 0x02, 0x01.
// Adjust the address jumpers on the underside of the product. 
#define AS3935_ADDR 0x03 
#define INDOOR 0x12 
#define OUTDOOR 0xE
#define LIGHTNING_INT 0x08
#define DISTURBER_INT 0x04
#define NOISE_INT 0x01

SparkFun_AS3935 lightning(AS3935_ADDR);

// Interrupt pin for lightning detection 

const int lightningInt = 23;  // pin on TTGo_Lora_OLED
const int threshVal = 5;
// This variable holds the number representing the lightning or non-lightning
// event issued by the lightning detector. 
int intVal = 0;
int noise = 2; // Value between 1-7 
int disturber = 3; // Value between 1-10
float Distance_Miles;
float Distance_km;
int Level_Lightning;
int Lightning_Status = 0 ; //lightning Status varible
/*

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26


//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6
*/


//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//packet counter
int counter = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

//BME280 stuff...

float Pressure_HPA; // sea level is (1013.25)
float Temp_DegF; // degrees F (unit reads in C)
float Temp_DegC; // degrees C (unit reads in C)
float Humidity_per; // Humidity in percent..
float Altitude; // unit reads in meters
float PNominal_HPA; // nominal air pressure at Tullahoma TN
int pindex; // pressure index..
char ptrend; // pressure trend name..
float Pressure_Delta; //differential between nominal and atmosphere
float Poffset_HPA; //pressure correction..

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;



void setup() {
 
 
  // When lightning is detected the interrupt pin goes HIGH.
  pinMode(lightningInt, INPUT); 

 
 
  //initialize Serial Monitor
  Serial.begin(9600);

  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("KI4UJY Weather Rock ");
  display.display();

  /*Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.print("LoRa Initializing OK!");
  display.display();
  delay(2000);  */

    //Serial.begin(9600);
    while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));

    unsigned status;

    // default settings
    status = bme.begin();
    // You can also pass in a Wire library object like &Wire2
    //status = bme.begin(0x76, &Wire2)
    status = bme.begin(0x76);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }

    Serial.println("-- Default Test --");
    delayTime = 1000;

    Serial.println();
Poffset_HPA = 4.00;
PNominal_HPA = 981.51;

//Pressure Index setup not used!!!
char *ptrend[] {
    "**Nominal**",
    "|---^+----|",
    "|----+^---|",
    "|--^-+----|",
    "|----+-^--|",
    "|-^--+----|",
    "|----+--^-|",
    "|^---+----|",
    "|----+---^|",
    "****HIGH***",
    "***DOWN****",
    "****LOW****",
    "***DANGER**"};

//Lighning detector stuff and fluff 

  Serial.println("AS3935 Franklin Lightning Detector"); 

  Wire.begin(); // Begin Wire before lightning sensor. 

  if( !lightning.begin() ) { // Initialize the sensor. 
    Serial.println ("Lightning Detector did not start up, freezing!"); 
    while(1); 
  }
  else
    Serial.println("Schmow-ZoW, Lightning Detector Ready!");

  // The lightning detector defaults to an indoor setting at 
  // the cost of less sensitivity, if you plan on using this outdoors 
  // uncomment the following line:
  //lightning.setIndoorOutdoor(OUTDOOR); 


}

void loop() {

  Serial.print("Reading IC2: ");
  Serial.println(counter);

  /*

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();



  display.clearDisplay();
  display.setCursor(0,0);
  display.println("LORA SENDER");
  display.setCursor(0,20);
  display.setTextSize(1);
  display.print("LoRa packet sent.");
  display.setCursor(0,30);
  display.print("Counter:");
  display.setCursor(50,30);
  display.print(counter);
  display.display();
  */


 AtmosValues(); //read atmospheric pressure/parameters  Put your name or call signs in Digital Weather rock statement

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Digital Weather Rock");
  display.setCursor(0,15);
  display.setTextSize(1);
  display.print("Atmospheric");
  display.setCursor(0,25);
  display.print("Pres.: ");
  display.print(Pressure_HPA);
  display.print(" (hPa)");
  //display.setCursor(0,35);
  //display.print(Pressure_HPA);
  //display.display();
  display.setCursor(0,45);
  display.print("Lighning Status: ");
  display.print(Lightning_Status);
  display.setCursor(0,55);
  display.print("Lght. Dist.: ");
  display.print(Distance_Miles);
  display.print(" Mi.");
  display.display();
  
 

 BaroTrend(); //determine trend status from pressure

  counter++;

LightningDetect(); //run the lightning detect code..

  delay(1000);
}

void AtmosValues() {
    Serial.print("Temperature = ");
    Temp_DegC = bme.readTemperature();
    Temp_DegF = ((Temp_DegC * 1.8) + 32.0);
    Serial.print(Temp_DegF);
    Serial.println(" °F");

    Serial.print("Pressure = ");
    Pressure_HPA = bme.readPressure() / 100.0;
    Pressure_HPA = Pressure_HPA + Poffset_HPA;
    Serial.print(Pressure_HPA);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA) * 3.28084;
    Serial.print(Altitude);
    Serial.println(" ft");

    Serial.print("Humidity = ");
    Humidity_per = bme.readHumidity();
    Serial.print(Humidity_per);
    Serial.println(" %");

    Serial.println();
}

void BaroTrend(){
  // create a pressure states
  // pressure average comming up to nominal on initlaiation
  // or pressure rising
  // pressure from the pi is pa * 1000
  //Chadd Dosset weather station recorded a dropo
  // of .015 pa or 15 counts during a close pass of
  // a tornado credit to him for these values... 

  // begin offset calculation and mode

  Pressure_Delta =  Pressure_HPA - PNominal_HPA; //differential from norm
   pindex = 0; //default value

 
// begin delta range calculation -4 to +4) 
   if (Pressure_Delta <= -10.0){
    int pindex = 12; 
    display.setCursor(0,35);
    display.print("<<<L    +    H_DANGER");
    display.display();} 
     if ((Pressure_Delta <= -7.0) && (Pressure_Delta > -10.0)){
    int pindex = 11; 
    display.setCursor(0,35);
    display.print(" <<L    +    H_ALARM");
    display.display();}    
    if ((Pressure_Delta <= -6.0) &&(Pressure_Delta > -7.0)){
    int pindex = 10; 
    display.setCursor(0,35);
    display.print("  <L    +    H_LOW");
    display.display();}    
  if ((Pressure_Delta <= -4.0) && (Pressure_Delta > -6.0)){
    int pindex = 7;
    display.setCursor(0,35);
    display.print("   L>   +    H_Nom.");
    display.display();}
  if ((Pressure_Delta <= -3.0)&& (Pressure_Delta > -4.0)){
    int pindex = 5;
    display.setCursor(0,35);
    display.print("   L>>  +    H_Nom.");
    display.display();}
   if ((Pressure_Delta <= -2.0)&& (Pressure_Delta > -3.0)){
    int pindex = 3;
    display.setCursor(0,35);
    display.print("   L>>> +    H_Nom.");
    display.display();}
  if ((Pressure_Delta <= -1.0) && (Pressure_Delta > -2.0)){
    int pindex = 1;
    display.setCursor(0,35);
    display.print("   L>>>>+    H_Nom.");
    display.display();}
  if ((Pressure_Delta < 0.0) && (Pressure_Delta > -1.0)) {
    int pindex = 0;
    display.setCursor(0,35);
    display.print("   L>>>>\    H_Nom.");
    display.display();}  
  if ((Pressure_Delta >= 0.0) && (Pressure_Delta < 1.0)) {
    int pindex = 0;
    display.setCursor(0,35);
    display.print("   L>>>>/    H_Nom.");
    display.display();}
  if ((Pressure_Delta >= 1.0) && (Pressure_Delta < 2.0)){
    int pindex = 2;
    display.setCursor(0,35);
    display.print("   L>>>>+>   H_Nom.");
    display.display();}
  if ((Pressure_Delta >= 2.0) && (Pressure_Delta < 3.0)){
    int pindex = 4;
    display.setCursor(0,35);
    display.print("   L>>>>+>>  H_Nom.");
    display.display();}
  
  if ((Pressure_Delta >= 3.0) && (Pressure_Delta < 4.0)) {
    int pindex = 6;
    display.setCursor(0,35);
    display.print("   L>>>>+>>> H_Nom.");
    display.display();}
  
  if ((Pressure_Delta >= 4.0) && (Pressure_Delta < 4.5)) {
    int pindex = 8;
    display.setCursor(0,35);
    display.print("   L>>>>+>>>>H_Nom.");
    display.display();}
  if (Pressure_Delta >= 4.5){
    int pindex = 9;
    display.setCursor(0,35);
    display.print("   L>>>>+>>>>H>HIGH");
    display.display();}
 
 
    }
void LightningDetect(){
 if(digitalRead(lightningInt) == HIGH){
    // Hardware has alerted us to an event, now we read the interrupt register
    // to see exactly what it is. 
    intVal = lightning.readInterruptReg();
    if(intVal == NOISE_INT){
      Serial.println("Noise.");
      Lightning_Status = 1 ; //Noise
      // Too much noise? Uncomment the code below, a higher number means better
      // noise rejection.
      //lightning.setNoiseLevel(setNoiseLevel); 
    }
    else if(intVal == DISTURBER_INT){
      Serial.println("Disturber."); 
      Lightning_Status = 2 ; //Disturber
      // Too many disturbers? Uncomment the code below, a higher number means better
      // disturber rejection.
      lightning.watchdogThreshold(threshVal);  
    }
    else if(intVal == LIGHTNING_INT){
      Serial.println("Lightning Strike Detected!"); 
      Lightning_Status = 3 ; //Lighning Strike
      // Lightning! Now how far away is it? Distance estimation takes into
      // account any previously seen events in the last 15 seconds. 
      byte distance = lightning.distanceToStorm(); 
      Serial.print("Approximately: "); 
      Serial.print(distance); 
      Serial.println("km away!"); 
      Distance_km = (float) distance;
      Distance_Miles =  Distance_km * 0.6213712;
      Serial.print(Distance_Miles); 
      Serial.println("Miles away!"); 
      Level_Lightning = (int)intVal;
      Serial.print(Level_Lightning); 
      Serial.println("Lighting Signal Detect 'Level'"); 
    }
      //delay(100); // Slow it down.

  }
}