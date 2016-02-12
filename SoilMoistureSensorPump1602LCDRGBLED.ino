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
#define sensorPowerPin 6
#define ledPin 13
#define buttonPin 4
#define pumpPin 3

#define redPin 10
#define greenPin 9
#define bluePin 11

int moistureLevel = 0;
int moistureLevelRaw = 0;

int wetReading = 250;
int dryReading = 0;

int dryReadingAddress = 0;
int wetReadingAddress = 1;

long lastDebounceTime = 0;
long debounceDelay = 50;

int threshold = 50;

int pumpOnDuration = 1000;
int pumpWaitDuration = 60000;

int interval = 1000;

long lastPumpTime = 0;

void setup()
{
  Serial.begin(115200);


  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(sensorPowerPin, OUTPUT);

  dryReading = getDryReading();
  wetReading = getWetReading();

  Serial.println("Starting soil moisture sensor");

  Serial.print("Lowest reading: ");
  Serial.println(dryReading);
  Serial.print("Highest reading: ");
  Serial.println(wetReading);

setWetReading(wetReading);
setDryReading(dryReading);
  lcd.init();

  lcd.backlight();
}

void loop()
{
  checkCommand();

  checkButton();

  takeReading();


  delay(interval);
}

void displayReading()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Moisture: ");
  lcd.print(moistureLevel);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("Threshold: ");
  lcd.print(threshold);
  lcd.print("%");
}
void checkCommand()
{
  while (Serial.available() > 0)
  {
    char command = Serial.read();

    Serial.println(command);

    switch (command)
    {
      case 'L':
        lastReadingTime = 0;
        takeReading();
        setdryReading(moistureLevelRaw);
        break;
      case 'H':
        lastReadingTime = 0;
        takeReading();
        setwetReading(moistureLevelRaw);
        break;
    }
  }
}

void checkButton()
{
  int reading = digitalRead(buttonPin);

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == HIGH) {

      lastDebounceTime = millis();

      Serial.println("Button pressed");

      digitalWrite(ledPin, HIGH);

      // Reset the last reading time to force another reading
      lastReadingTime = 0;
      takeReading();

      digitalWrite(ledPin, LOW);
    }
  }
}

void takeReading()
{
  if (lastReadingTime + readingInterval < millis()
      || lastReadingTime == 0)
  {
    //Serial.println("Taking reading");

    lastReadingTime = millis();

    // If the interval is less than 2 seconds the sensor doesn't need to be turned off
    if (interval > 2000)
    {
      sensorOn();

      delay(2000);
    }

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

    Serial.print("Wet raw value: ");
    Serial.print(wetReading);
    Serial.print("   ");
    
    Serial.print("Dry raw value: ");
    Serial.print(dryReading);
    Serial.println();
    
    
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
  if (moistureLevel < threshold
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
  //Serial.println("Turning sensor on");

  digitalWrite(sensorPowerPin, HIGH);
}

void sensorOff()
{
  //Serial.println("Turning sensor off");

  digitalWrite(sensorPowerPin, LOW);
}

void setdryReading(int reading)
{
  dryReading = reading;
  
  Serial.print("Setting lowest reading: ");
  Serial.println(reading);

  EEPROM.write(dryReadingAddress, reading / 4); // Must divide by 4 to make it fit in eeprom
}

void setwetReading(int reading)
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
