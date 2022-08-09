#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#define encA 2
#define encB 3
#define set 8
#define oneWireBus 13
#define levelSensor A0
#define relay 9
#define buzzer 10

LiquidCrystal lcd(12, 11, 4, 5, 6, 7);
OneWire oneWire(oneWireBus);
DallasTemperature tempSensor(&oneWire);

// Encoder
volatile int counter = 30;
volatile int lastStateA;
volatile int lastStateB;

//Timer set and Temp set
boolean tempSet = false;
boolean timeSet = false;
int timer;
unsigned long prevTimer, curTimer;
const unsigned long period = 60000;
boolean done = false;
boolean timerActive = false;

//PID Constants
double kp = 50;
double ki = 0.4;
double kd = 100;

//PID Variables
unsigned long PIDLastTime, PIDCurTime;
double changeTime;
double error;
double lastError;
double cumError;
double changeError;
int targetTemp;
double curTemp;
int powerMax = 100;

//Relay Time Interval
unsigned long intLastTime, intCurTime, intervalTime;
double powerLevel = 0;
int intervalLength = 10; //Period of the relay switching

boolean buzzerOn = true;

void setup() {
  Serial.begin(9600);

  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
  lastStateA = digitalRead(encA);
  lastStateB = digitalRead(encB);
  attachInterrupt(0, readEnc, RISING);
  attachInterrupt(1, readEnc, RISING);

  pinMode(levelSensor, INPUT);
  pinMode(buzzer, OUTPUT);

  lcd.begin(16, 2);
  tempSensor.begin();

  pinMode(set, INPUT);
  pinMode(relay, OUTPUT);

  setTemp();
  setTime();
}

void loop() {
  if (!done) {
    tempSensor.requestTemperatures();
    curTemp = tempSensor.getTempCByIndex(0);
    printLCD("SET:" + String(targetTemp) + "C CUR:" + String(curTemp) + "C ", 0);
    delay(10);

    if (timerActive) {
      updateTimer();
      printLCD("TIMER:" + String(timer) + " MIN      ", 1);
      delay(10);
    }
    else {
      printLCD("Hold for timer", 1);
      delay(10);
      if (digitalRead(set) == 1) {
        startTimer();
      }
    }

    if(analogRead(A0) > 450){
      delay(10);
      powerLevel = PIDout();
      updatePID();
      delay(10);
    }
    else{
      digitalWrite(relay, LOW);
      lcd.clear();
      printLCD("Water Level Low", 0);
      printLCD("Add Water", 1);
    }
    Serial.println(String(powerLevel));
   
  }
  else {
    digitalWrite(relay, LOW);
    if(digitalRead(set) == 1){
      buzzerOn = false;
    }
  else if(buzzerOn){
      tone(buzzer,400,100);
      tone(buzzer,1000,100);
      delay(100);
      lcd.clear();
      printLCD("DONE", 0);
      printLCD("Press to end", 1);
  }
    delay(100);
  }
}

void updatePID() {
  intCurTime = millis();
  unsigned long intChangeTime = intCurTime - intLastTime;
  intervalTime += intChangeTime;

  if (intervalTime > intervalLength * 1000) {
    intervalTime = 0;
  }
  else if (intervalTime < intervalLength * 1000 * powerLevel) {
    digitalWrite(relay, HIGH);
  }
  else {
    digitalWrite(relay, LOW);
  }
  intLastTime = intCurTime;
}

double PIDout() {
  PIDCurTime = millis();
  changeTime = double(PIDCurTime - PIDLastTime) / 1000;

  error = targetTemp - curTemp;
  changeError = (error - lastError) / changeTime;

  cumError += error * changeTime;
  cumError = constrain(cumError, 0, powerMax * 0.5);

  double output = kp * error + ki * cumError + kd * changeError;
  output = constrain(output, 0, powerMax);
  lastError = error;
  PIDLastTime = PIDCurTime;

  return output / powerMax;
}

void printLCD(String message, int line) {
  lcd.setCursor(0, line);
  lcd.print(message);
}

void setTemp() {
  while (!tempSet) {

    counter = constrain(counter, 30, 80);
    double tempInF = counter*(9.0/5.0)+32;

    if (digitalRead(set) == 1) {
      targetTemp = counter;
      tempSet = true;
    }
    printLCD("SET TEMP:" + String(counter) + "C", 0);
    printLCD("         " + String(tempInF) + "F", 1);
    delay(100);
    lcd.clear();
  }
}

void setTime() {
  delay(100);
  counter = 0;
  while (!timeSet) {
    if (counter < 0) {
      counter = 0;
    }
    if (digitalRead(set) == 1) {
      timer = counter;
      timeSet = true;
    }
    printLCD("SET TIME:" + String(counter), 0);
    delay(100);
    lcd.clear();
  }
}
void startTimer() {
  prevTimer = millis();
  timerActive = true;
}

void updateTimer() {
  curTimer = millis();
  if (curTimer - prevTimer >= period) {
    prevTimer = curTimer;
    timer --;
  }
  else if (timer <= 0) {
    done = true;
  }
}

void readEnc() {
  int currentStateA = digitalRead(encA);
  int currentStateB = digitalRead(encB);

  if (currentStateA == 1 && currentStateB == 1) {
    if (lastStateA == 1 && lastStateB == 0) {
      counter --;
    }
    else if (lastStateA == 0 && lastStateB == 1) {
      counter ++;
    }
  }

  lastStateA = currentStateA;
  lastStateB = currentStateB;
}
