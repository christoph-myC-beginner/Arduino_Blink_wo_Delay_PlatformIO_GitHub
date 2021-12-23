//Einfacher Code mit Blink durch millis() und Serieller Ausgabe

#include <Arduino.h>

<<<<<<< HEAD
#include <OneWire.h>
#include <DallasTemperature.h>
=======
//variables declarations
int toggleInterval = 200; //Zeit, die die LED an bzw aus ist in Millisekunden
int msgInterval = 1000;
>>>>>>> 0b4e6a1307d2f6a6bd2b7cc294e806e66df17f7c

OneWire oneWire(2); //OneWire on Pin(Number)
DallasTemperature DS18B20(&oneWire);

float tempC;

int temp_einer = 0;
int temp_zehner = 0;

//variables declarations
int toggleInterval = 30000; //Zeit, die die LED an bzw aus ist in Millisekunden
int msgInterval = 1000;
int askIntervall = 500;
int cnt = 0;
int scrollTime = 280;

int latchPin = 7; // RCLK (Register Clock / Latch) Pin des 74HC595, mit dessen High Pegel die Daten aus dem Memory an den Ausgang gegeben werden
int clockPin = 8; // SRCLK (Shit Register Clock) Pin des 74HC595 mit dessen Flankenwechsel der an SER anliegende Pegel als 0 oder 1 eingeschoben wird
int dataPin = 5;  // SER (Serial input) Pin des 74HC595, der die eingeschobenen Bits aufnimmt
int enablePin = 6;// EN (low active) Pin, der den Ausgang "freischaltet" 




//                    abcdefgHigh-Gitter
byte ciphArr[19] = {0b00000000, //blank
                    0b01100001, //1
                    0b11011011, //2
                    0b11110011, //3
                    0b01100111, //4
                    0b10110111, //5
                    0b10111111, //6
                    0b11100001, //7
                    0b11111111, //8
                    0b11110111, //9
                    0b11111101, //0 - 10
                    0b11101111, //A - 11
                    0b10011101, //C - 12
                    0b10011111, //E - 13 
                    0b10001111, //F - 14
                    0b01101111, //H - 15
                    0b00011101, //L - 16
                    0b00101011, //n - 17
                    0b11000111};//° - 18


bool doFlag = 0;
bool doneFlag = 1;

//functions declarations
void noDelayBlink(byte pin, int blinkZeit );
void putShiftRegister(int li, int re);
void askInterval(int askZeit);
void serialMsg(int msgZeit);
void pwmInit(void);
void pwmStop(void);
void convTemp(float f);
void Lena(void);
//////////////////////////////////////////////////////
// Setup fuer die LED und Serielle Ausgabe
void setup() {
  //Serial.begin(9600);
  
  pinMode(LED_BUILTIN, OUTPUT); //LED_BUILTIN ist keyword fuer die LED auf dem Board, bei Arduino an Pin 13
  digitalWrite(LED_BUILTIN, LOW); 
  pinMode(11, OUTPUT); //PWM fuer H-Bruecke
  pinMode(2, INPUT); // OneWire DS18B20
  pinMode(3, OUTPUT); //PWM fuer H-Brucke

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  digitalWrite(enablePin, LOW); //Low active !

  //DS18B20 related
  DS18B20.begin(); 
  //End of Ds18B20 related
  
  pwmStop();
  pwmInit();
}

//////////////////////////////////////////////////////
// the loop function runs over and over again forever
void loop() {
  //serialMsg(msgInterval);
  askInterval(askIntervall);
  
  noDelayBlink(LED_BUILTIN, toggleInterval);
    if (doFlag == 1) {
      if (doneFlag == 0) {
        doneFlag = 1;
        cnt = cnt + 1;
      }
      if (cnt >= 19) {
        cnt = 0;
      }
      Lena();
    }
    else {
      doneFlag = 0;
      if (temp_einer == 0 && tempC > 9) {
        temp_einer = 10;
      }
      putShiftRegister(ciphArr[temp_zehner],ciphArr[temp_einer]);
    } 
 
}
//////////////////////////////////////////////////////
/////////////////////////////////////////////////////

void putShiftRegister(int li, int re) {
  //digitalWrite(enablePin, HIGH); //Low active !
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,li);
  shiftOut(dataPin,clockPin,MSBFIRST,re);
  digitalWrite(latchPin, HIGH);
  digitalWrite(enablePin, LOW); //Low active !
}

void askInterval(int askZeit) {
  static unsigned long askMillis; //eigene Variable gleichen Namens, unabhaengig von der Variable in der "toggle-Funktion"
  if (millis() - askMillis > askZeit) {
    askMillis = millis();
    DS18B20.requestTemperaturesByIndex(0);
    tempC = DS18B20.getTempCByIndex(0);
    convTemp(tempC);
  }
}

void convTemp(float f) {
  int ziffern = f - 1.2;
  temp_einer = ziffern % 10;
  temp_zehner = ziffern / 10;
}

void Lena(void) {
  putShiftRegister(ciphArr[0],ciphArr[0]);
  delay(150);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[16]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(enablePin, LOW);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[13]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[17]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[11]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[0]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[0]);
  digitalWrite(latchPin, HIGH);
  //delay(200);
  doFlag = 0;
}

void serialMsg(int msgZeit) {
  static unsigned long blinkMillis; //eigene Variable gleichen Namens, unabhaengig von der Variable in der "toggle-Funktion"
  if (millis() - blinkMillis > msgZeit) {
    blinkMillis = millis();
    Serial.println("Test Message");
    Serial.print("Vergangene Millisekunden seit Programmstart: ");
    Serial.println(millis());
    Serial.println("----- ----- ----- ----- -----");
    
    Serial.print("DS18B20-Temp Float: ");
    Serial.println(tempC);
    Serial.println("***** ----- ----- ----- *****");
    Serial.print("DS18B20-Temp Einer: ");
    Serial.println(temp_einer);
    Serial.print("DS18B20-Temp Zehne: ");
    Serial.println(temp_zehner);
    Serial.println("----- ##### ----- ##### -----");
    Serial.print("doFlag: ");
    Serial.println(doFlag);
    
  }
}

void pwmInit(void) {
  //Timer2 setup
  //TCCR2A = 0; //Timer/Counter Control Register A
  TCCR2A |= (1<<COM2A1) | (1<<COM2A0) | (1<<COM2B1) | (0<<COM2B0);
  TCCR2A |= (1<<WGM21) | (1<<WGM20);
  //TCCR2B = 0; //Timer/Counter Control Register B
  TCCR2B |= (0<<CS22) | (1<<CS21) | (0<<CS20); //0 1 0 entspricht 1 zu 8
  TCCR2B |= (0<<WGM22);
  //TCNT2 = 0; // Timer/Counter Register: The Timer/Counter Register gives direct access, both for read and write operations, to theTimer/Counter unit 8-bit counte
  //OCR2A = 0; //Output Compare Register A
  OCR2A = 12;
  //OCR0B = 0; //Output Compare Register B
  OCR2B = 243;
  //TIMSK2 = 0; // Timer/Counter Interrupt Mask Registe
  //TIFR2 = 0; //  Timer/Counter 0 Interrupt Flag Registe
  //Ende Timer2 setup
}

void pwmStop(void) {
  TCCR2A &= (0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0);
  TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20));
  //digitalWrite(PB0, LOW);
  //digitalWrite(PB1, LOW);
}

void noDelayBlink(byte pin, int blinkZeit ) {
  static unsigned long blinkMillis;
  if (millis() - blinkMillis > blinkZeit) {
    blinkMillis = millis();
    //digitalWrite(pin, !digitalRead(pin)); //toggeln eines Digitalausgangs
    doFlag = 1; //digitalRead(pin);
  }
}