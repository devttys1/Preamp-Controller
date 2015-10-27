/*
 The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 7
 * LCD D5 pin to digital pin 6
 * LCD D6 pin to digital pin 5
 * LCD D7 pin to digital pin 4
 * LCD R/W pin to ground
 */
#include <LiquidCrystal.h>
#include <IRremote.h>
#include <SPI.h>
#include <EEPROM.h>
#include <LcdBarGraph.h>
#include <math.h>

byte lcdNumCols = 20;
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);
LcdBarGraph lbg(&lcd, lcdNumCols);

#define encoder0PinA 2
#define encoder0PinB 3

const int RECV_PIN = 21;
const int PGA_CS_PIN = 22;
const int PGA_SCLK_PIN = 23;
const int PGA_SDI_PIN = 24;
const int PGA_MUTE_PIN = 13;
const int LCD_LED_PIN = 8;
const int FP_BUTTONS = 54;
const int BUTTON1_PIN = 30;
const int BUTTON2_PIN = 31;
const int BUTTON3_PIN = 32;
const int BUTTON4_PIN = 33;
const int BUTTON5_PIN = 39;
const int RELAY1_PIN = 35;
const int RELAY2_PIN = 40;
const int RELAY3_PIN = 41;
const int RELAY4_PIN = 38;
const int RELAY5_PIN = 34;
const int TXLED = 37;
const int RXLED = 36;


IRrecv irrecv(RECV_PIN);
decode_results results;

volatile byte encoder0Pos = EEPROM.read(0);
volatile byte volumeLevel = 0;

int buttonState1 = 0; 
int buttonState2 = 0; 
int buttonState3 = 0; 
int buttonState4 = 0;
int buttonState5 = 0;
int lastButtonState1 = 0;
int lastButtonState2 = 0;
int lastButtonState3 = 0;
int lastButtonState4 = 0;
int lastButtonState5 = 0;
int IRbutton = 0;
int Power = 0;

//int Input = EEPROM.read(1);
//int LCDBrightness = EEPROM.read(2);
int Input = 2;
int LCDBrightness = 128;

//decode_results results;

unsigned long codeValue;   // Decoded value

unsigned long VolIncr = 1587632295;
unsigned long VolDecr = 1587664935;
unsigned long MuteCode = 1587624135;
unsigned long PowerCode = 2122415745;
unsigned long Input1Code = 1587643260;
unsigned long Input2Code = 1587620565;
unsigned long Input3Code = 1587636375;
unsigned long Input4Code = 1587653205;

bool ValidCode;
bool muteActive;

byte count;
long unsigned int data;
byte lastValue;

void setup() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(20, 4);
  lcd.print("Standby");
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(BUTTON3_PIN, INPUT);
  pinMode(BUTTON4_PIN, INPUT);
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE); 
  // IR pin on interrupt 2 (pin 21)
  attachInterrupt(2, getIR, CHANGE);
  irrecv.enableIRIn(); // Start the receiver 
  Serial1.begin(9600);
  pinMode(PGA_CS_PIN,OUTPUT);
  pinMode(PGA_SCLK_PIN,OUTPUT);
  pinMode(PGA_SDI_PIN,OUTPUT);
  pinMode(PGA_MUTE_PIN,OUTPUT);
  pinMode(RELAY1_PIN,OUTPUT);
  pinMode(RELAY2_PIN,OUTPUT);
  pinMode(RELAY3_PIN,OUTPUT);
  pinMode(RELAY4_PIN,OUTPUT);
  pinMode(RELAY5_PIN,OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  digitalWrite(PGA_CS_PIN,HIGH);
  digitalWrite(PGA_SCLK_PIN,HIGH);
  digitalWrite(PGA_SDI_PIN,HIGH);
  //LearnIR();
}

void loop() {
  //Check if On
  if (Power){
    //Check if Volume Changed
    if(volumeLevel != encoder0Pos){
      volumeLevel = encoder0Pos;
      updateVolume(volumeLevel);
    }

    //Do Pushbuttons
    switch (ReadButtons()){
    case 1:
      //Button 1 Input
      Input ++;
      if (Input > 4){
        Input = 1;
      } 
      else if(Input < 1){
        Input = 4;
      }
      SetInput(Input);
      break;
    case 2:
      //Button 2 Menu
      if (Power)
      {
        PowerOff();
      }
      else
      {
        PowerOn();
      }
      break;
    case 3:
      //Button 3 Left
      LCDBrightness = LCDBrightness - 10;
      if (LCDBrightness > 255)
      {
        LCDBrightness = 255;
      } 
      else if(LCDBrightness < 0)
      {
        LCDBrightness = 0;
      }
      SetLCDBrightness(LCDBrightness);
      break;
    case 4:
      //Button 4 Right
      LCDBrightness = LCDBrightness + 10;
      if (LCDBrightness > 255)
      {
        LCDBrightness = 255;
      } 
      else if(LCDBrightness < 0)
      {
        LCDBrightness = 0;
      }
      SetLCDBrightness(LCDBrightness);
      break;
    default:
      //Do nothing
      break;
    }
  }
  else{
    //wait for power on
    switch (ReadButtons()){
    case 2:
      PowerOn();
      break;
    default:
      //do nothing
      break;
    }
  }
}

int ReadButtons()
{
  buttonState1 = digitalRead(BUTTON1_PIN);
  buttonState2 = digitalRead(BUTTON2_PIN);
  buttonState3 = digitalRead(BUTTON3_PIN);
  buttonState4 = digitalRead(BUTTON4_PIN);

  if (buttonState1 != lastButtonState1) {
    if (buttonState1 == HIGH) {
      Serial1.println("Button 1 On");
      lastButtonState1 = buttonState1;
    } 
    else {
      Serial1.println("Button 1 Off");
      lastButtonState1 = buttonState1;
      return(1); 
    }
  }

  if (buttonState2 != lastButtonState2) {
    if (buttonState2 == HIGH) {
      Serial1.println("Button 2 On");
      lastButtonState2 = buttonState2;
    } 
    else {
      Serial1.println("Button 2 Off");
      lastButtonState2 = buttonState2;
      return(2);  
    }
  }

  if (buttonState3 != lastButtonState3) {
    if (buttonState3 == HIGH) {
      Serial1.println("Button 3 On");
      lastButtonState3 = buttonState3;
    } 
    else {
      Serial1.println("Button 3 Off"); 
      lastButtonState3 = buttonState3;
      return(3); 
    }
  }

  if (buttonState4 != lastButtonState4) {
    if (buttonState4 == HIGH) {
      Serial1.println("Button 4 On");
      lastButtonState4 = buttonState4;
    } 
    else {
      Serial1.println("Button 4 Off"); 
      lastButtonState4 = buttonState4; 
      return(4);
    }
  }
  return(0);
}

void PGA_set_volume(byte value)
{
  digitalWrite(PGA_CS_PIN, LOW);     // assert CS
  SPI_write(value);            // left value (0..255)
  digitalWrite(PGA_CS_PIN, HIGH);    // deassert CS
}

static inline void SPI_write(byte out_spi_byte)
{
  int i;
  // loop thru each of the 8-bits in the byte
  for (i=0; i<8; i++)  {	
    digitalWrite(PGA_SDI_PIN, !!(out_spi_byte & (1 << (7 - i))));	// data out
    digitalWrite(PGA_SCLK_PIN, HIGH);                        // clock it
    digitalWrite(PGA_SCLK_PIN, LOW);		                    // reset clk 
  }
  digitalWrite(PGA_CS_PIN, 1); 
  digitalWrite(PGA_SDI_PIN, LOW);	// data out reset
}

void doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial1.println (encoder0Pos, DEC);  
  if(encoder0Pos < 1){
    encoder0Pos = 1;
  }
  if(encoder0Pos > 254){
    encoder0Pos = 255;
  }
}

void doEncoderB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial1.println (encoder0Pos, DEC); 
  if(encoder0Pos < 1){
    encoder0Pos = 1;
  }
  if(encoder0Pos > 254){
    encoder0Pos = 255;
  }
}

// convert signed float in the range of [ -95,5 to + 35,5 ] to a channel Byte Value
uint8_t dB2Byte (float dBValue)
{
  if (dBValue >= 31.5)
    return 255;
  if (dBValue <= -95.5)
    return 0;

  return (uint8_t)(255 + (dBValue - 31.5)*2);
}

// convert a Byte Value to the corresponding dB value
float byte2dB(uint8_t Value)
{
  // calc val to dB
  return (float)(31.5 - ((255 - Value)*0.5)); 
}

void LearnIR()
{
  // learn IR
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press Vol + ");
  while(!getIRValue());
  VolIncr = codeValue;
  //EEPROM.write(4, VolIncr);
  delay(100);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press Vol - ");
  while(!getIRValue());
  VolDecr = codeValue;
  //EEPROM.write(5, VolDecr);
  delay(100);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press Mute ");
  while(!getIRValue());
  MuteCode = codeValue;
  //EEPROM.write(6, MuteCode); 
  delay(100); 

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press Input 1 ");
  while(!getIRValue());
  Input1Code = codeValue;
  //EEPROM.write(6, MuteCode); 
  delay(100); 

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press Input 2 ");
  while(!getIRValue());
  Input2Code = codeValue;
  //EEPROM.write(6, MuteCode); 
  delay(100); 

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press Input 3 ");
  while(!getIRValue());
  Input3Code = codeValue;
  //EEPROM.write(6, MuteCode); 
  delay(100); 

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press Input 4 ");
  while(!getIRValue());
  Input4Code = codeValue;
  //EEPROM.write(6, MuteCode); 
  delay(100); 

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Press Power ");
  while(!getIRValue());
  PowerCode = codeValue;
  //EEPROM.write(6, MuteCode); 
  delay(100); 

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Done.          ");
  delay(500);
}
bool getIRValue(void)
{
  if (irrecv.decode(&results)) {
    codeValue = results.value;
    if ((results.decode_type == NEC) || (results.decode_type == SONY)) {
      // NEC or SONY remote  
      if (results.value == REPEAT) {
        irrecv.resume(); // Receive the next value
        return false;
      }
    }
    Serial1.println(codeValue);
    irrecv.resume(); // Receive the next value
    return true;
  }
  return false;
}
void getIR()
{
  if (irrecv.decode(&results)) {
    if ((results.decode_type == NEC) || (results.decode_type == SONY)) {
      //NEC or SONY remote
      if (results.value == REPEAT) {
        if ((codeValue == MuteCode)||(codeValue == Input1Code)||(codeValue == Input2Code)||(codeValue == Input3Code)||(codeValue == Input4Code)||(codeValue == PowerCode)) {
          //do not repeat Mute
          irrecv.resume();
          return;
        }
      }
      else {
        //lcd.setCursor(0, 1);
        codeValue = results.value;
        Serial1.println(codeValue);
      }
    }
    if (codeValue == MuteCode) {
      muteActive = !muteActive;
      if(muteActive){
        digitalWrite(PGA_MUTE_PIN, LOW);
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 2);
        lcd.print("Volume: Mute");
        Serial1.println("Mute On");
      } 
      else {
        digitalWrite(PGA_MUTE_PIN, HIGH);
        lcd.setCursor(0, 2);
        lcd.print("                    ");
        lcd.setCursor(0, 2);
        lcd.print("Volume: ");
        lcd.print(byte2dB(volumeLevel));
        lcd.print(" dB");
      }
    }
    else if (codeValue == VolIncr) {
      //incr volume
      updateVolume(incrVol(volumeLevel));
      Serial1.println("Vol+");
    }
    else if (codeValue == VolDecr) {
      //decr volume
      updateVolume(decrVol(volumeLevel));
      Serial1.println("Vol-");
    }
    else if (codeValue == PowerCode) {
      //set power
      Serial1.println("Power");
      if (Power)
      {
        PowerOff();
      }
      else
      {
        PowerOn();
      }
    }
    else if (codeValue == Input1Code) {
      Serial1.println("Input 1");
      SetInput(1);
    }
    else if (codeValue == Input2Code) {
      Serial1.println("Input 2");
      SetInput(2);
    }
    else if (codeValue == Input3Code) {
      Serial1.println("Input 3");
      SetInput(3);
    }
    else if (codeValue == Input4Code) {
      Serial1.println("Input 4");
      SetInput(4);
    }
  } 
  irrecv.resume();
}
void PowerOn()
{
  digitalWrite(RXLED, LOW);
  digitalWrite(TXLED, HIGH);
  //fade on LCD
  for(int i = 0; i < LCDBrightness; i++){
  analogWrite(LCD_LED_PIN, i);
  delay(30);
  }
  //LearnIR();
  for(int i = 5; i > 0; i--)
  {
    lcd.clear();
    lcd.print("Preheating Tubes");
    lcd.setCursor(0,1);
    lcd.print("B+ on in: ");
    lcd.print(i);
    lcd.print(" sec");
    digitalWrite(RXLED, !digitalRead(RXLED));
    digitalWrite(TXLED, !digitalRead(TXLED));
    delay(1000);
  }
  lcd.clear();
  digitalWrite(RELAY5_PIN, HIGH);
  lcd.print("PreAmp Controller");
  volumeLevel = EEPROM.read(0);
  delay(1000);
  SetInput(Input);
  updateVolume(volumeLevel);
  Power = true;
}
void PowerOff()
{
  lcd.clear();
  lcd.print("Standby");
  digitalWrite(PGA_MUTE_PIN, LOW);
  EEPROM.write(0, volumeLevel);
  volumeLevel = 0;
  delay(100);
  digitalWrite(RELAY5_PIN, LOW);
  //fade out LCD
  for (int i = LCDBrightness; i > 0; i--)
  {
    analogWrite(LCD_LED_PIN, i);
    delay(30);
  }
  
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
  Power = false;
}
void SetInput(int InputVal)
{
  lcd.setCursor(0, 1);
  lcd.print("Input: ");
  lcd.print(InputVal);
  EEPROM.write(1, InputVal);
  switch(InputVal)
  {
  case 1:
    digitalWrite(PGA_MUTE_PIN, LOW);
    delay(50);
    digitalWrite(RELAY1_PIN, HIGH);
    digitalWrite(RELAY2_PIN, LOW);
    digitalWrite(RELAY3_PIN, LOW);
    digitalWrite(RELAY4_PIN, LOW);
    delay(50);
    digitalWrite(PGA_MUTE_PIN, HIGH);
    break;

  case 2:
    digitalWrite(PGA_MUTE_PIN, LOW);
    delay(50);
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, HIGH);
    digitalWrite(RELAY3_PIN, LOW);
    digitalWrite(RELAY4_PIN, LOW);
    delay(50);
    digitalWrite(PGA_MUTE_PIN, HIGH);
    break;

  case 3:
    digitalWrite(PGA_MUTE_PIN, LOW);
    delay(50);
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    digitalWrite(RELAY3_PIN, HIGH);
    digitalWrite(RELAY4_PIN, LOW);
    delay(50);
    digitalWrite(PGA_MUTE_PIN, HIGH);
    break;

  case 4:
    digitalWrite(PGA_MUTE_PIN, LOW);
    delay(50);
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    digitalWrite(RELAY3_PIN, LOW);
    digitalWrite(RELAY4_PIN, HIGH);
    delay(50);
    digitalWrite(PGA_MUTE_PIN, HIGH);
    break;

  default:
    break;
  }
}
void SetLCDBrightness(byte LCDBrightnessVal)
{
  analogWrite(LCD_LED_PIN, LCDBrightnessVal);
  EEPROM.write(2, LCDBrightnessVal);
  Serial1.println(LCDBrightnessVal);
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  lcd.print("LCD: ");
  lcd.print(LCDBrightnessVal);
}
byte incrVol (int val)
{
  val++;
  if (val > 255) val = 255;
  encoder0Pos = val;
  return val;
}

byte decrVol (int val)
{
  val--;
  if (val < 0) val = 0;
  encoder0Pos = val;
  return val;
}
void updateVolume(int val)
{
  PGA_set_volume(val);
  //EEPROM.write(0, val);
  Serial1.print("Volume: ");
  Serial1.println(val);
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(0, 2);
  lcd.print("Volume: ");
  lcd.print(byte2dB(val));
  lcd.print(" dB");
  lbg.drawValue(val, 256);
  digitalWrite(PGA_MUTE_PIN,HIGH);
}



