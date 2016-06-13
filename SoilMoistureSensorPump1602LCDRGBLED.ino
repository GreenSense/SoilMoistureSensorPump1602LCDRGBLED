#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

long second = 1000;
long minute = second * 60;
long hour = minute * 60;

long lastReadingTime = 0;
//long readingInterval = hour;
long readingInterval = second;

#define moisturePin 0
#define sensorPowerPin 8
#define ledPin 13
#define button1Pin 7
#define button2Pin 6
#define button3Pin 5
#define button4Pin 4
#define pumpPin 3

#define redPin 10
#define greenPin 9
#define bluePin 11

int moistureLevel = 0;
int moistureLevelRaw = 0;

int wetReading = 0;
int dryReading = 150;
//int dryReading = 250;
//int dryReading = 1023;

int dryReadingAddress = 0;
int wetReadingAddress = 1;

long lastDebounceTime = 0;
long debounceDelay = 200;

int threshold = 50;

int pumpOnDuration = 1000;
int pumpWaitDuration = 60000;

int interval = 2000;

long lastPumpTime = 0;

bool screenIsOn = true;
bool sensorIsOn = true;
long sensorOnTime = 0;

long lastDisplayTime = 0;
long displayRefreshInterval = 1000;

void setup()
{
  Serial.begin(115200);


  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(sensorPowerPin, OUTPUT);

  // Uncomment these lines to use the values above. Otherwise they will be replaced by
  // values from EEPROM in the following lines
  //setWetReading(wetReading);
  //setDryReading(dryReading);

  dryReading = getDryReading();
  wetReading = getWetReading();

  Serial.println("Starting soil moisture sensor");

  Serial.print("Dry reading: ");
  Serial.println(dryReading);
  Serial.print("Wet reading: ");
  Serial.println(wetReading);


  lcd.init();

  lcd.backlight();

  // If the interval is less than 2 seconds turn the sensor on and leave it on (otherwise it will be turned on just before it's needed)
  if (interval <= 2000)
  {
    sensorOn();

    //delay(2000);
  }

}

void loop()
{
  checkCommand();

  checkButton();

  takeReading();


  delay(100);
}

void displayReading()
{
  if (millis() > lastDisplayTime + displayRefreshInterval)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moisture: ");
    lcd.print(cstr(moistureLevel));
    lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("Irrigate at: ");
    lcd.print(threshold);
    lcd.print("%");

    lastDisplayTime = millis();
  }
}
void checkCommand()
{
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

void checkButton()
{

  if ((millis() - lastDebounceTime) > debounceDelay) {

    int reading1 = digitalRead(button1Pin);
    int reading2 = digitalRead(button2Pin);
    int reading3 = digitalRead(button3Pin);
    int reading4 = digitalRead(button4Pin);

    if (reading1 == HIGH)
      toggleScreen();

    if (reading2 == HIGH)
      Serial.println("Button 2");

    if (reading3 == HIGH)
    {
      calibrateWet();
    }

    if (reading4 == HIGH)
    {
      calibrateDry();
    }



    if (reading1 == HIGH
        || reading2 == HIGH
        || reading3 == HIGH
        || reading4 == HIGH) {

      lastDebounceTime = millis();

      Serial.println("Button pressed");

      //digitalWrite(ledPin, HIGH);

      // Reset the last reading time to force another reading
      lastReadingTime = 0;

      //digitalWrite(ledPin, LOW);
    }
  }
}

void calibrateWet()
{
  Serial.println("Calibrating wet soil");

  //lcd.clear();
 // lcd.setCursor(0, 0);
  //lcd.print("Confirm calibrate wet... ");
  //lcd.setCursor(0, 1);
 // lcd.print("Press button 1");

  //if ((millis() - lastDebounceTime) > debounceDelay) {

   // int reading1 = digitalRead(button1Pin);
    lastReadingTime = 0; // Force another reading
    takeReading();
    setWetReading(moistureLevelRaw);

  //  Serial.println("Confirmed");
  //}
}

void calibrateDry()
{
  Serial.println("Calibrating dry soil");
  
  lastReadingTime = 0; // Force another reading
  takeReading();
  setDryReading(moistureLevelRaw);
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

void takeReading()
{
  if (lastReadingTime + readingInterval < millis()
      || lastReadingTime == 0)
  {

    //Serial.println("Preparing to take reading");

    if (!sensorIsOn && interval < 2000)
    {
      sensorOn();
    }
    else if (sensorIsOn && sensorOnTime + 2000 < millis()
             || interval < 2000)
    {
      Serial.println("Taking reading");

      lastReadingTime = millis();

      // If the interval is less than 2 seconds the sensor is already be on. No need to turn it on again.
      /* if (interval > 2000)
        {
         sensorOn();

         delay(2000);
        }*/

      int readingSum  = 0;
      int totalReadings = 10;

      for (int i = 0; i < totalReadings; i++)
      {
        int reading = analogRead(moisturePin);

        readingSum += reading;
      }

      int averageReading = readingSum / totalReadings;

      moistureLevelRaw = averageReading;

      Serial.print("Average raw reading: ");
      Serial.println(averageReading);

      moistureLevel = map(averageReading, dryReading, wetReading, 0, 100);

      if (moistureLevel < 0)
        moistureLevel = 0;

      if (moistureLevel > 100)
        moistureLevel = 100;

      Serial.print("Moisture level: ");
      Serial.print(moistureLevel);
      Serial.println("%");

      // If the interval is less than 2 seconds then don't turn the sensor off
      if (interval > 2000)
      {
        sensorOff();
      }

      setRgbLed();

      displayReading();

      irrigateIfNeeded();

      //Serial.print("Wet raw value: ");
      //Serial.print(wetReading);
      //Serial.print("   ");

      //Serial.print("Dry raw value: ");
      //Serial.print(dryReading);
      //Serial.println();

    }
  }
  else
  {
    outputTimeRemaining();
  }
}

void setRgbLed()
{
  if (moistureLevel > threshold)
  {
    analogWrite(bluePin, 255);
    analogWrite(redPin, 0);
  }
  else
  {
    analogWrite(redPin, 255);
    analogWrite(bluePin, 0);
  }
}

void irrigateIfNeeded()
{
  if (moistureLevel <= threshold
      && (lastPumpTime + pumpWaitDuration < millis()
          || lastPumpTime == 0))
  {
    Serial.println("Irrigating");

    digitalWrite(pumpPin, HIGH);

    Serial.println("Pump is on");

    delay(pumpOnDuration);

    digitalWrite(pumpPin, LOW);

    Serial.println("Pump is off");

    lastPumpTime = millis();

  }
}

void outputTimeRemaining()
{
  long timeSinceLastReading = millis();
  if (lastReadingTime > 0)
    timeSinceLastReading = millis() - lastReadingTime;
  long millisecondsRemaining = readingInterval - timeSinceLastReading;
  int secondsRemaining = millisecondsRemaining / 1000;

  //Serial.println(secondsRemaining);
}

void sensorOn()
{
  Serial.println("Turning sensor on");

  digitalWrite(sensorPowerPin, HIGH);

  sensorOnTime = millis();

  sensorIsOn = true;
}

void sensorOff()
{
  Serial.println("Turning sensor off");

  digitalWrite(sensorPowerPin, LOW);

  sensorIsOn = false;
}

void setDryReading(int reading)
{
  dryReading = reading;

  Serial.print("Setting lowest reading: ");
  Serial.println(reading);

  EEPROM.write(dryReadingAddress, reading / 4); // Must divide by 4 to make it fit in eeprom
}

void setWetReading(int reading)
{
  wetReading = reading;

  Serial.print("Setting highest reading: ");
  Serial.println(reading);

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
