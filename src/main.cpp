//Einfacher Code mit Blink durch millis() und Serieller Ausgabe
//Simple "Vorlage"
//Tests mit Github

#include <Arduino.h>

//variables declarations
int toggleInterval = 200; //Zeit, die die LED an bzw aus ist in Millisekunden
int msgInterval = 1000;

bool testKeepFlag = 0;

bool keepFlag = 0;
unsigned long keepHmillis;

//functions declarations
void noDelayBlink(byte pin, int blinkZeit );
void serialMsg(int msgZeit);
void keepPinHighForTime(int keepHTime, int pin);
bool keepAnyFlagHighForTime(int keepItTime, bool anyFlag);
void simplePWM(int pin, int cycleOn, int cycleFull);

//////////////////////////////////////////////////////
// Setup fuer die LED und Serielle Ausgabe
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); //LED_BUILTIN ist keyword fuer die LED auf dem Board, bei Arduino an Pin 13
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, INPUT_PULLUP);
}

//////////////////////////////////////////////////////
// the loop function runs over and over again forever
void loop() {
  noDelayBlink(LED_BUILTIN, toggleInterval);
  serialMsg(msgInterval);
  keepPinHighForTime(1000, 12);
  if (digitalRead(8) == HIGH) {
    //Serial.println("8 ist HIGH");
    keepFlag = 0;
    testKeepFlag = 1;
  }
  else {
    testKeepFlag = 0;
  }
  testKeepFlag = keepAnyFlagHighForTime(2000, testKeepFlag);
  digitalWrite(11, testKeepFlag);

}
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
void noDelayBlink(byte pin, int blinkZeit ) {
  static unsigned long blinkMillis;
  if (millis() - blinkMillis > blinkZeit) {
    blinkMillis = millis();
    digitalWrite(pin, !digitalRead(pin)); //toggeln eines Digitalausgangs
  }
}

void serialMsg(int msgZeit) {
  static unsigned long blinkMillis; //eigene Variable gleichen Namens, unabhaengig von der Variable in der "toggle-Funktion"
  if (millis() - blinkMillis > msgZeit) {
    blinkMillis = millis();
    Serial.println("Test Message");
    Serial.print("Vergangene Millisekunden seit Programmstart: ");
    Serial.println(millis());
    Serial.println("----- ----- ----- ----- -----");
  }
}

void keepPinHighForTime(int keepHTime, int pin) {
  static bool doneFlag;
  if (keepFlag == 0) {
    keepHmillis = millis();
    doneFlag = 0;
    keepFlag = 1;
    //Serial.println("SET");
  }
  if (millis() - keepHmillis < keepHTime && doneFlag == 0) {
    digitalWrite(pin, HIGH);
  }
  else {//if (millis() - keepHmillis > keepHTime ODER done Flag == 1)
    digitalWrite(pin, LOW);
    doneFlag = 1;
  }
}

bool keepAnyFlagHighForTime(int keepItTime, bool anyFlag) {
  static unsigned long keepFlagMillis;
  static bool doneFlag;
  if (anyFlag == 1 && doneFlag == 0) {
    keepFlagMillis = millis();
    doneFlag = 1; //set the millis()
  }
  else if (anyFlag == 0) {
    return 0;
  }
  if (millis() - keepFlagMillis < keepItTime) {
    return 1;
  }
  else { //if (millis() - keepFlagmillis > keepItTime ODER anyFlag == 0)
    doneFlag = 0;
    return 0;
  }
}

void simplePWM(int pin, int cycleOn, int cycleFull){
  int pulsingPin = pin;
  static unsigned long previousMicros;
  if ((micros() - previousMicros) < cycleFull) {
    if ((micros() - previousMicros) < cycleOn) {
      digitalWrite(pulsingPin, LOW);
    }
    else {
      digitalWrite(pulsingPin, HIGH);
    }
  }
  else {
    previousMicros = micros();
  }
}
