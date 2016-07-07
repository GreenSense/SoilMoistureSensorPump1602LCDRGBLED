#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

long second = 1000;
long minute = second * 60;
long hour = minute * 60;

long lastReadingTime = 0;
//long readingInterval = hour;
long readingInterval = 3 * second;

#define moisturePin 0
#define sensorPowerPin 8
#define ledPin 13
#define button1Pin 2
#define button2Pin 3
#define button3Pin 4
#define button4Pin 5
#define pumpPin 7

#define redPin 10
#define greenPin 9
#define bluePin 11

int moistureLevel = 0;
int moistureLevelRaw = 0;

int wetReading = 1024;
int dryReading = 0;

int dryReadingAddress = 0;
int wetReadingAddress = 1;

long lastDebounceTime = 0;
long debounceDelay = 200;

int threshold = 50;

bool pumpIsOn = 0;
long pumpStartTime = 0;
long lastPumpFinishTime = 0;
int pumpBurstDuration = 1*second;
long pumpWaitOffDuration = 1*minute;
//long pumpWaitOffDuration = 30*second;

bool screenIsOn = true;
bool sensorIsOn = true;
long lastSensorOnTime = 0;

long lastDisplayTime = 0;
long displayRefreshInterval = 1000;

bool buttonOnValue = 0; // 1 if button is active HIGH and 0 if button is active LOW

#define CALIBRATION_MODE_OFF 0
#define CALIBRATION_MODE_DRY 1
#define CALIBRATION_MODE_WET 2

long calibrationTriggerTime = 0;
long calibrationInterval = 3000;
int calibrationMode = CALIBRATION_MODE_OFF;

long lastSerialOutputTime = 0;
long serialOutputInterval = readingInterval;

bool isDebug = true;

void setup()
{
  Serial.begin(115200);

  Serial.println("Starting irrigator");

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(sensorPowerPin, OUTPUT);

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);
  pinMode(button4Pin, INPUT_PULLUP);

  if (EEPROM.read(dryReadingAddress) != 0)
    dryReading = getDryReading();
  else
    setDryReading(dryReading);

  if (EEPROM.read(wetReadingAddress) != 0)
    wetReading = getWetReading();
  else
    setWetReading(wetReading);

  lcd.init();

  lcd.backlight();

  // If the interval is less than 2 seconds turn the sensor on and leave it on (otherwise it will be turned on just before it's needed)
  if (readingInterval <= 2000)
  {
    sensorOn();

    // TODO: Remove if not needed
    //delay(2000);
  }

}

void loop()
{
  checkCommand();

  checkButton();

  checkCalibrationTimeout();

  takeReading();

  serialPrintData();

  setRgbLed();

  displayReading();

  irrigateIfNeeded();

// TODO: Remove if not needed
//  delay(1);
}

/* Commands */
void checkCommand()
{
  // TODO: Is this function still required now that calibration is completed using buttons?
  while (Serial.available() > 0)
  {
    char command = Serial.read();

    Serial.println(command);

    switch (command)
    {
      case 'D':
        lastReadingTime = 0;
        takeReading();
        setDryReading(moistureLevelRaw);
        break;
      case 'W':
        lastReadingTime = 0;
        takeReading();
        setWetReading(moistureLevelRaw);
        break;
    }
  }
}

/* Buttons */
void checkButton()
{
  if (lastDebounceTime + debounceDelay < millis()
      || lastDebounceTime == 0) {

    int reading1 = digitalRead(button1Pin);
    int reading2 = digitalRead(button2Pin);
    int reading3 = digitalRead(button3Pin);
    int reading4 = digitalRead(button4Pin);

    if (reading1 == buttonOnValue)
      button1Pressed();

    if (reading2 == buttonOnValue)
      button2Pressed();

    if (reading3 == buttonOnValue)
      button3Pressed();

    if (reading4 == buttonOnValue)
      button4Pressed();

    if (reading1 == buttonOnValue
        || reading2 == buttonOnValue
        || reading3 == buttonOnValue
        || reading4 == buttonOnValue) {

      lastDebounceTime = millis();

      Serial.println("Button pressed");

      // Reset the last reading time to force another reading
      lastReadingTime = 0; // TODO: Check if needed
    }
  }
}

void button1Pressed()
{
  Serial.println("Button 1 pressed");

  if (calibrationMode == CALIBRATION_MODE_OFF)
    initiateDryCalibration();
  else if (calibrationMode == CALIBRATION_MODE_DRY && calibrationTriggerTime + calibrationInterval > millis())
    confirmCalibrateDry();
  else
    cancelCalibration();
}

void button2Pressed()
{
  Serial.println("Button 2 pressed");

  if (calibrationMode == CALIBRATION_MODE_OFF)
    initiateWetCalibration();
  else if (calibrationMode == CALIBRATION_MODE_WET && calibrationTriggerTime + calibrationInterval > millis())
    confirmCalibrateWet();
  else
    cancelCalibration();
}

void button3Pressed()
{
  Serial.println("Button 3 pressed");

  threshold ++;

  Serial.print("Threshold: " );
  Serial.println(threshold);

  refreshDisplay();
}

void button4Pressed()
{
  Serial.println("Button 4 pressed");

  threshold --;
  Serial.print("Threshold: " );
  Serial.println(threshold);

  refreshDisplay();
}

/* Readings */
void takeReading()
{
  if (lastReadingTime + readingInterval < millis()
      || lastReadingTime == 0)
  {
    if (!sensorIsOn && readingInterval > 2000)
    {
      sensorOn();
    }
    else if (sensorIsOn && lastSensorOnTime + 2000 < millis()
             || readingInterval < 2000)
    {
      if (isDebug)
        Serial.println("Preparing to take reading");

      int readingSum  = 0;
      int totalReadings = 5;

      for (int i = 0; i < totalReadings; i++)
      {
        int reading = analogRead(moisturePin);

        readingSum += reading;
      }

      int averageReading = readingSum / totalReadings;

      moistureLevelRaw = averageReading;

      moistureLevel = calculateMoistureLevel(averageReading);

      if (moistureLevel < 0)
        moistureLevel = 0;

      if (moistureLevel > 100)
        moistureLevel = 100;

      // If the interval is less than 2 seconds then don't turn the sensor off
      if (readingInterval > 2000)
      {
        sensorOff();
      }
      
      lastReadingTime = millis();
    }
  }
}

int calculateMoistureLevel(int reading)
{
  return map(reading, dryReading, wetReading, 0, 100);
}

/* Serial Output */
void serialPrintData()
{
  if (lastSerialOutputTime + serialOutputInterval < millis()
      || lastSerialOutputTime == 0)
  {
    Serial.print("D;"); // This prefix indicates that the line contains data.
    Serial.print("Raw:");
    Serial.print(moistureLevelRaw);
    Serial.print(";");
    Serial.print("Calibrated:");
    Serial.print(moistureLevel);
    Serial.print(";");
    Serial.print("Threshold:");
    Serial.print(threshold);
    Serial.print(";");
    Serial.print("WaterNeeded:");
    Serial.print(moistureLevel < threshold);
    Serial.print(";");
    Serial.print("PumpOn:");
    Serial.print(pumpIsOn);
    Serial.print(";");
    Serial.print("SecondsSincePumpOn:");
    Serial.print((millis()-lastPumpFinishTime)/1000);
    Serial.print(";");
    Serial.print("Dry:");
    Serial.print(dryReading);
    Serial.print(";");
    Serial.print("Wet:");
    Serial.print(wetReading);
    Serial.print(";");
    Serial.println();

    if (isDebug)
      {
        Serial.print("Last pump start time:");
        Serial.println(pumpStartTime);
        Serial.print("Last pump finish time:");
        Serial.println(lastPumpFinishTime);
      }

    lastSerialOutputTime = millis();
  }
}

/* Display */
void displayReading()
{
  if (millis() > lastDisplayTime + displayRefreshInterval)
  {
    lcd.clear();
    lcd.setCursor(0, 0);

    if (!isDebug)
    {
      lcd.print("Moisture: ");
      if (moistureLevelRaw < dryReading)
        lcd.print("<");
      lcd.print(moistureLevel);
      lcd.print("%");
      lcd.setCursor(0, 1);
      if (calibrationMode == CALIBRATION_MODE_OFF)
      {
        lcd.print("Irrigate <");
        lcd.print(threshold);
        lcd.print("%");
      }
      else if (calibrationMode == CALIBRATION_MODE_DRY)
      {
        lcd.print("Dry:");
        lcd.print(moistureLevelRaw);
      }
      else if (calibrationMode == CALIBRATION_MODE_WET)
      {
        lcd.print("Wet:");
        lcd.print(moistureLevelRaw);
      }
    }
    else
    {
      if (moistureLevelRaw < dryReading)
        lcd.print("<");
      lcd.print(moistureLevel);
      lcd.print("% (");
      lcd.print(moistureLevelRaw);
      lcd.print(")");
      lcd.print("  T");
      lcd.print(threshold);

      lcd.setCursor(0, 1);
      lcd.print("D:");
      lcd.print(dryReading);

      if (calibrationMode == CALIBRATION_MODE_DRY)
      {
        lcd.print("(c)");
      }

      lcd.print(" W:");
      lcd.print(wetReading);

      if (calibrationMode == CALIBRATION_MODE_WET)
      {
        lcd.print("(c)");
      }
    }

    lastDisplayTime = millis();
  }
}

void refreshDisplay() {
  // Reset the last display time to force a refresh
  lastDisplayTime = 0;

  displayReading();
}


void toggleScreen()
{
  screenIsOn = !screenIsOn;

  Serial.println("Toggle screen");
  if (screenIsOn)
    lcd.backlight();
  else
    lcd.noBacklight();
}

/* Irrigation */
void irrigateIfNeeded()
{
  bool pumpBurstFinished = pumpStartTime + pumpBurstDuration < millis();
  bool waterIsNeeded = moistureLevel <= threshold;
  bool pumpIsReady = lastPumpFinishTime + pumpWaitOffDuration < millis() || lastPumpFinishTime == 0;

  if (pumpIsOn)
  {
    //Serial.println("Pump is on"); // Line disabled because it floods the serial output
    if (pumpBurstFinished)
    {
      if (isDebug)
        Serial.println("Pump burst finished");
      pumpOff();
    }
  }
  else if (waterIsNeeded)
  {

    //Serial.println("Water is needed"); // Line disabled because it floods the serial output
    if (pumpIsReady)
    {
      if (isDebug)
        Serial.println("Pump is turning on");
      pumpOn();
    }
    //  else
    //  Serial.println("Pump is waiting"); // Line disabled because it floods the serial output
  }
}

void pumpOn()
{
  digitalWrite(pumpPin, HIGH);
  pumpIsOn = true;

  pumpStartTime = millis();
}

void pumpOff()
{
  digitalWrite(pumpPin, LOW);
  pumpIsOn = false;

  lastPumpFinishTime = millis();
}

/* Sensor */
void sensorOn()
{
  if (isDebug)
    Serial.println("Turning sensor on");

  digitalWrite(sensorPowerPin, HIGH);

  lastSensorOnTime = millis();

  sensorIsOn = true;
}

void sensorOff()
{
  if (isDebug)
    Serial.println("Turning sensor off");

  digitalWrite(sensorPowerPin, LOW);

  sensorIsOn = false;
}

/* RGB LED */
void setRgbLed()
{
  if (calibrationMode != CALIBRATION_MODE_OFF
      && calibrationTriggerTime + calibrationInterval > millis())
  {
    analogWrite(greenPin, 255);
    analogWrite(redPin, 0);
    analogWrite(bluePin, 0);
  }
  else
  {
    if (moistureLevel > threshold)
    {
      analogWrite(bluePin, 255);
      analogWrite(redPin, 0);
      analogWrite(greenPin, 0);
    }
    else
    {
      analogWrite(redPin, 255);
      analogWrite(bluePin, 0);
      analogWrite(greenPin, 0);
    }
  }
}

/* Calibration */
void initiateDryCalibration()
{
  calibrationMode = CALIBRATION_MODE_DRY;
  calibrationTriggerTime = millis();

  refreshDisplay();
}

void initiateWetCalibration()
{
  calibrationMode = CALIBRATION_MODE_WET;
  calibrationTriggerTime = millis();

  refreshDisplay();
}

void confirmCalibrateDry()
{
  lastReadingTime = 0;
  takeReading();
  setDryReading(moistureLevelRaw);

  // Calibration mode is completed. Go back to normal
  calibrationMode = CALIBRATION_MODE_OFF;

  refreshDisplay();
}

void confirmCalibrateWet()
{
  lastReadingTime = 0;
  takeReading();
  setWetReading(moistureLevelRaw);

  // Calibration mode is completed. Go back to normal
  calibrationMode = CALIBRATION_MODE_OFF;

  refreshDisplay();
}

void checkCalibrationTimeout()
{
  if (calibrationMode != CALIBRATION_MODE_OFF
      && calibrationTriggerTime + calibrationInterval < millis())
  {
    cancelCalibration();
    Serial.println("Calibration timed out");
  }
}

void cancelCalibration()
{
  calibrationMode = CALIBRATION_MODE_OFF;

  refreshDisplay();
}

void setDryReading(int reading)
{
  dryReading = reading;

  if (isDebug)
  {
    Serial.print("Setting dry reading: ");
    Serial.println(reading);
  }

  EEPROM.write(dryReadingAddress, reading / 4); // Must divide by 4 to make it fit in eeprom
}

void setWetReading(int reading)
{
  wetReading = reading;

  if (isDebug)
  {
    Serial.print("Setting wet reading: ");
    Serial.println(reading);
  }

  EEPROM.write(wetReadingAddress, reading / 4); // Must divide by 4 to make it fit in eeprom
}

int getDryReading()
{
  int value = EEPROM.read(dryReadingAddress);

  if (value == 255)
    return dryReading;
  else
    return value * 4; // Must multiply by 4 to get the original value
}

int getWetReading()
{
  int value = EEPROM.read(wetReadingAddress);

  if (value == 255)
    return wetReading;
  else
    return value * 4; // Must multiply by 4 to get the original value
}
