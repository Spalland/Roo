
// INCLUDES

#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPR121.h>
#include <RotaryEncoder.h>
#include "FastLED.h"

// STRUCTS

struct Switches { 
  bool kitchenMain = true;
  bool bedMain = true;
  bool topCubbyLED = true;
  bool bedCubbyLED = true;
  bool kitchenLED = true;
  bool bedLED = true;
}; //Switches

struct RGBValue { 
  byte r = byte(255);
  byte g = byte(255);
  byte b = byte(255);
}; //RGBValue

struct LEDSettings { 
  RGBValue kitchenMain;
  RGBValue bedMain;
  RGBValue topCubbyLED;
  RGBValue bedCubbyLED;
  RGBValue kitchenLED;
  RGBValue bedLED;
}; //LedSettings





// DEFINES - LIGHTING OUTPUT LED

#define TOP_CUBBY_LEDS 2 // ***FIXME*** needs real number
#define BED_CUBBY_LEDS 2 // ***FIXME*** needs real number
#define KITCHEN_LEDS 58 // ***FIXME*** needs real number
#define BED_LEDS 58 // ***FIXME*** needs real number
#define TOP_CUBBY_LEDS_DATA_PIN 1 // ***FIXME*** needs real number
#define BED_CUBBY_LEDS_DATA_PIN 2 // ***FIXME*** needs real number
#define KITCHEN_LEDS_DATA_PIN 3 // ***FIXME*** needs real number
#define BED_LEDS_DATA_PIN 4 // ***FIXME*** needs real number

// DEFINES - LIGHTING CONTROL LED

#define ENCODER_MULTIPLIER 11
#define ENCODER_LED_RED 13
#define ENCODER_LED_GREEN 12
#define ENCODER_LED_BLUE 11

// DEFINES - CAPACITIVE TOUCH 

#define CAPACITIVE_TOUCH 6 // ***FIXME*** needs real number

// DEFINES - CONTROL ENCODER 

#define ENCODER_CONTROL_BUTTON 22 // ***FIXME*** needs real number

// DEFINES - INDICATOR LEDS

#define TOP_CUBBY_LEDS_RGB_INDICATOR 8 // ***FIXME*** needs real number
#define BED_CUBBY_LEDS_RGB_INDICATOR 9 // ***FIXME*** needs real number
#define KITCHEN_LEDS_RGB_INDICATOR 10 // ***FIXME*** needs real number
#define BED_LEDS_RGB_INDICATOR 11 // ***FIXME*** needs real number
#define TOP_CUBBY_LEDS_EDIT_INDICATOR 12 // ***FIXME*** needs real number
#define BED_CUBBY_LEDS_EDIT_INDICATOR 13 // ***FIXME*** needs real number
#define KITCHEN_LEDS_EDIT_INDICATOR 14 // ***FIXME*** needs real number
#define BED_LEDS_EDIT_INDICATOR 15 // ***FIXME*** needs real number

// DEFINES - LCD

#define LCD_I2C_ADDR 0x27  // Define I2C Address where the PCF8574A is
#define LCD_BACKLIGHT_PIN     3
#define LCD_EN_PIN  2
#define LCD_RW_PIN  1
#define LCD_RS_PIN  0
#define LCD_D4_PIN  4
#define LCD_D5_PIN  5
#define LCD_D6_PIN  6
#define LCD_D7_PIN  7

// DEFINES - SERIAL INTERRUPTS

#define SERIAL_SEND_INTERRUPT 54
#define SERIAL_RECEIVE_INTERRUPT 55

// INSTANTIATIONS - LIGHTING OUTPUT 

CRGB topCubbyLeds[TOP_CUBBY_LEDS];
CRGB bedCubbyLeds[BED_CUBBY_LEDS];
CRGB kitchenLeds[KITCHEN_LEDS];
CRGB bedLeds[BED_LEDS];

// INSTANTIATIONS - LIGHTING CONTROL

RotaryEncoder encoderR(15, 14);
RotaryEncoder encoderG(17, 16);
RotaryEncoder encoderB(18, 19);

// INSTANTIATIONS - CAPACITIVE TOUCH

Adafruit_MPR121 capacitiveTouch = Adafruit_MPR121();

// INTANTIATIONS - CONTROL ENCODER

RotaryEncoder controlEncoder(20, 21); // ***FIXME*** needs real number

// INSTANTIATIONS - LCD

LiquidCrystal_I2C  lcd(LCD_I2C_ADDR,LCD_EN_PIN,LCD_RW_PIN,LCD_RS_PIN,LCD_D4_PIN,LCD_D5_PIN,LCD_D6_PIN,LCD_D7_PIN);

// VARIABLES - CAPACITIVE TOUCH

volatile uint16_t lasttouched = 0;
volatile uint16_t currtouched = 0;

// VARIABLES - CONTROL ENCODER

volatile int controlEncoderCurrentPosition = 0;

// VARIABLES - LIGHTING

volatile Switches currentSwitches; 
volatile LEDSettings currentLEDSettings; 


void setup() {

  // CALIBRATION DELAY

  delay(2000);

  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  char currentString[20] = {"bob"};
  char *currentStringPtr = &currentString[20];
  //lightingProfile[0] = currentStringPtr;

  

  //SETUP LIGHTING OUTPUT LED
  
  FastLED.addLeds<WS2811, TOP_CUBBY_LEDS_DATA_PIN, BRG>(topCubbyLeds, TOP_CUBBY_LEDS);
  FastLED.addLeds<WS2811, BED_CUBBY_LEDS_DATA_PIN, BRG>(bedCubbyLeds, BED_CUBBY_LEDS);
  FastLED.addLeds<WS2811, KITCHEN_LEDS_DATA_PIN, BRG>(kitchenLeds, KITCHEN_LEDS);
  FastLED.addLeds<WS2811, BED_LEDS_DATA_PIN, BRG>(bedLeds, BED_LEDS);

  //SETUP LIGHTING CONTROL LED

  pinMode(ENCODER_LED_RED, OUTPUT);
  pinMode(ENCODER_LED_GREEN, OUTPUT);
  pinMode(ENCODER_LED_BLUE, OUTPUT);

  // SETUP - CAPACITIVE TOUCH

  if (!capacitiveTouch.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");

  // SETUP CAPACITIVE TOUCH

  pinMode(CAPACITIVE_TOUCH, INPUT);

  // SETUP INTERRUPTS - INITIAL 
  
  attachInterrupts();

  // SETUP CONTROL BUTTON

  pinMode(ENCODER_CONTROL_BUTTON, INPUT);

  // SETUP SERIAL INTERRUPTS

  pinMode(SERIAL_SEND_INTERRUPT, OUTPUT);
  pinMode(SERIAL_RECEIVE_INTERRUPT, INPUT); 

  // SETUP LCD
  lcd.begin (20,4);
  
  // Switch on the backlight
  lcd.setBacklightPin(LCD_BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home(); 

  formatStringToLCD("        ROO           Lighting Control         V:0.1"); 

  //delay(5000);
  lcd.clear();

  formatStringToLCD("Sleeping...."); 

  delay(500);
  lcd.clear();

  lcd.setBacklight(LOW);

  waitForConfirmOrCancel("Confirm save new    profile?"); 
  
  

} // setup()

void loop() {
  // put your main code here, to run repeatedly:

} // loop() 


//Need Function To Get Pressed Button or Touched Sensor

//lightingControl

//Detach Interrupts
//Begin Listen Loop for Button Activations

//SWITCH

//CASE 

// Light Switches
// Colour Edit Buttons
// Control Button


void loadProfiles(){};

bool waitForConfirmOrCancel(char msg[]){

  analogWrite(ENCODER_LED_RED,255); //TEMP CALL TO ACTIVATE LED-BUTTON BREAKOUT
  
  bool currentSelection = false;
  lcd.clear();
  lcd.setBacklight(HIGH);
  formatStringToLCD(msg); 
  lcd.setCursor(0,4); 
  lcd.print(char(126));
  lcd.print(" Cancel");
  lcd.setCursor(9,4);
  lcd.print(char(17));
  lcd.print(" Confirm");
  
  while(1){ 

    int encoderDirection = getEncoderDirection(); 

    if(encoderDirection == -1){ 
      
      currentSelection = false;
      lcd.setCursor(0,4); 
      lcd.print(char(126)); 
      lcd.print(" Cancel"); 
      lcd.setCursor(9,4); 
      lcd.print(char(17)); 
      lcd.print(" Confirm"); 
        
    }else if(encoderDirection == 1){ 
      
      currentSelection = true;  
      lcd.setCursor(0,4); 
      lcd.print(char(17));
      lcd.print(" Cancel");
      lcd.setCursor(9,4);
      lcd.print(char(126));
      lcd.print(" Confirm");
   
    }

    if(digitalRead(ENCODER_CONTROL_BUTTON)){ 

      lcd.clear();
      lcd.print("You have selected to");
      lcd.setCursor(6,2);
      lcd.print(currentSelection ? "Confirm" : " Cancel");
  
      return currentSelection; 
    }
  }

  
} // waitForConfirmOrCancel()


int getEncoderDirection(){ 
 
  encoderR.tick(); 

  int newPos = encoderR.getPosition(); 

  if(controlEncoderCurrentPosition == newPos){ 

    controlEncoderCurrentPosition = newPos;
    return 0; 
    
  }else if ( controlEncoderCurrentPosition < newPos){ 

    controlEncoderCurrentPosition = newPos;
    return -1;
  }else{ 

    controlEncoderCurrentPosition = newPos;
    return 1;
  }
}

void saveProfile(){};

void lightingControl(){ 

  // Detach Interrupts

  detachInterrupts();

  while(1){  // Listen Loop
    
    
  }
  
} // lightingControl()


void editLedColour(){

  static int posR = 0;
  static int posG = 0;
  static int posB = 0;
  static int lumR = 0;
  static int lumG = 0;
  static int lumB = 0;
  encoderR.tick();
  encoderG.tick();
  encoderB.tick();

  int newPosR = encoderR.getPosition();
  int newPosG = encoderG.getPosition();
  int newPosB = encoderB.getPosition();
  
  if (posR != newPosR) {
    if(newPosR > posR){
      lumR += ENCODER_MULTIPLIER;
      lumR > 255 ? lumR = 255 : lumR;
    }else{ 
      lumR -= ENCODER_MULTIPLIER;
      lumR < 0 ? lumR = 0 : lumR;
    }
    posR = newPosR;
  }

  if (posG != newPosG) {
    if(newPosG > posG){
      lumG += ENCODER_MULTIPLIER;
      lumG > 255 ? lumG = 255 : lumG;
    }else{ 
      lumG -= ENCODER_MULTIPLIER;
      lumG < 0 ? lumG = 0 : lumG;
    }
    posG = newPosG;
  }

  if (posB!= newPosB) {
    if(newPosB > posB){
      lumB += ENCODER_MULTIPLIER;
      lumB > 255 ? lumB = 255 : lumB;
    }else{ 
      lumB -= ENCODER_MULTIPLIER;
      lumB < 0 ? lumB = 0 : lumB;
    }

    Serial.println(lumB);
    posB = newPosB;
   
  }

  analogWrite(ENCODER_LED_RED, lumR);
  analogWrite(ENCODER_LED_GREEN, lumG);
  analogWrite(ENCODER_LED_BLUE, lumB);
  
} // editLedColour()

void getTouched(){ 

  currtouched = capacitiveTouch.touched();
  
  for (uint8_t i=0; i<12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" touched");
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" released");
    }
  }
} // getTouched()


void attachInterrupts(){ 

   attachInterrupt(digitalPinToInterrupt(CAPACITIVE_TOUCH), lightingControl, RISING);
   attachInterrupt(digitalPinToInterrupt(CAPACITIVE_TOUCH), lightingControl, RISING);

}

void detachInterrupts(){ 

  detachInterrupt(CAPACITIVE_TOUCH);

}

void formatStringToLCD(char wrapString[]) {
   int lineCount = 0;
   int lineNumber = 0;
   byte stillProcessing = 1;
   byte charCount = 1;
   lcd.clear();
   lcd.setCursor(0,0);

   while(stillProcessing) {
        if (++lineCount > 20) {    // have we printed 20 characters yet (+1 for the logic)
             lineNumber += 1;
             lcd.setCursor(0,lineNumber);   // move cursor down
             lineCount = 1;
        }

        lcd.print(wrapString[charCount - 1]);

        if (!wrapString[charCount]) {   // no more chars to process?
             stillProcessing = 0;
        }
        charCount += 1;
   }
} // formatStringForLCD

void requestProfiles(char *profile[]){
  
  digitalWrite(SERIAL_SEND_INTERRUPT, HIGH);
  digitalWrite(SERIAL_SEND_INTERRUPT, LOW); 
  static int timeoutCounter = millis();

  while(1){ 
    
    if(digitalRead(SERIAL_RECEIVE_INTERRUPT)){
       continue;  
    }else if(millis() > (timeoutCounter + 5000)){
       lcd.clear();
       lcd.print("ERROR:              Connection Timed Out");
       delay(5000);
       return;  
    }
     
  }
  
  Serial.write("profiles");

  if(Serial.available() > 0) {
    
    // read the incoming byte:
    digitalWrite(SERIAL_SEND_INTERRUPT, HIGH);
    digitalWrite(SERIAL_SEND_INTERRUPT, LOW); 
    return Serial.readString();
              
  }else{
    lcd.clear();
    lcd.print("ERROR:              No Data Received");
    delay(5000);
    return; 
  }

  char currentString[20] = {"bob"};
  char *currentStringPtr = &currentString[20];
  *profile[0] = currentStringPtr;


}// requestProfiles()

class LightingProfile{

  char *profileName;
  Switches profileSwitches; 
  LEDSettings profileLEDValues;

  public:
    LightingProfile(){ 
      
    }
    LightingProfile ( char profileName_[], 
                      Switches profileSwitches_, 
                      LEDSettings profileLEDValues_ 
                      ){
      
      profileName = profileName_;
      profileSwitches = profileSwitches_;
      profileLEDValues = profileLEDValues_;
      
    }

    
    Switches getSwitchValues() {
      return profileSwitches;
    };
  
    void setSwitchValues(Switches target, bool source[]) { 
      profileSwitches.kitchenMain = source[0]; 
      profileSwitches.bedMain = source[1]; 
      profileSwitches.topCubbyLED = source[2]; 
      profileSwitches.bedCubbyLED = source[3]; 
      profileSwitches.kitchenLED = source[4]; 
      profileSwitches.bedLED = source[5];  
    }

    void setRGBValues(RGBValue target, byte source[]){
      target.r = source[0];
      target.g = source[1];
      target.b = source[2]; 
    }
};

