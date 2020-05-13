#include <Wire.h>                 // Servo Library
#include <LiquidCrystal_I2C.h>        // LCD Library
#include <Servo.h>                    // Servo Library

Servo myservo;                        // Servo Initialization
LiquidCrystal_I2C lcd(0x27, 16, 2);   // Type of LCD

// Pins Initiliaztion
int gate = 10;                        // Gate Sensor
int parking1 = 6;                     // Parking 1 sensor
int parking2 = 11;                    // Parking 2 sensor
int trigPin1 = 3;                     // Ultrasonic Sensor 1 trigger pin
int echoPin1 = 2;                     // Ultrasonic Sensor 1 echo pin
int trigPin2 = 7;                     // Ultrasonic Sensor 2 trigger pin
int echoPin2 = 8;                     // Ultrasonic Sensor 2 echo pin

// Variables
int pos = 0;                          // Servo Postions
long duration, distance, RightSensor, LeftSensor;   // For Ultrasonic

void setup()
{
  Serial.begin(19200);                // Baud Rate

  // Type
  pinMode(gate, INPUT);
  pinMode(parking1, INPUT);
  pinMode(parking2, INPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  myservo.attach(9);                  // Write servo positions on 9th pin of Arduino

  // LCD Initialization
  lcd.init();
  lcd.backlight();
}

void loop()
{
  int gateTemp = digitalRead(gate);
  int parkingTemp1 = digitalRead(parking1);
  int parkingTemp2 = digitalRead(parking2);
  int netDistance = 0;
  int temp = 0;
  int threshold1 = 15;               // Car Dimensions
  int threshold2 = 60;

  lcd.setCursor(0, 0);
  Serial.println("Parking System");

  // Ultrasonic Calculations

  SonarSensor(trigPin1, echoPin1);
  LeftSensor = distance;

  SonarSensor(trigPin2, echoPin2);
  RightSensor = distance;

  netDistance = LeftSensor + RightSensor;

  // Checking Availability of Parking Slots

  if (parkingTemp1 == 0 && parkingTemp2 == 1)               // Parking  Occupied, Parking 2 Vaccant
  {
    lcd.setCursor(0, 1);
    lcd.print("parking1 Occ. parking2 Vac.");
    Serial.println("parking1 Occ parking2 Vac");
  }

  else if (parkingTemp1 == 1 && parkingTemp2 == 0)          // Parking  Vaccant, Parking 2 Occupied
  {
    lcd.setCursor(0, 1);
    lcd.print("parking1 Vac parking2 occ");
    Serial.println("parking1 Vac parking2 occ");
  }

  else if (parkingTemp1 == 0 && parkingTemp2 == 0)          // Parking  Occupied, Parking 2 Occupied
  {
    lcd.setCursor(0, 1);
    lcd.print("parking1 occ parking2 occ");
    Serial.println("parking1 occ parking2 occ");

  }

  else if (parkingTemp1 == 1 && parkingTemp2 == 1)          // Parking  Vaccant, Parking 2 Vaccant
  {
    lcd.setCursor(0, 1);
    lcd.print("parking1 vac parking2 vac");
    Serial.println("parking1 vac parking2 vac");
  }

  lcd.clear();

  if (gateTemp == 0)                                       // If any one is in front of gate
  {
    if (netDistance > threshold1 && netDistance < threshold2 )    // Checking whether the vehicle is Car
    {
      Serial.println("Car at Entry");
      temp = 1;
    }      

    else if (netDistance < threshold1 && netDistance > threshold2 )
      temp = 0;

    if (parkingTemp1 == 1 && parkingTemp2 == 0 && temp == 1 || parkingTemp1 == 0 && parkingTemp2 == 1 && temp == 1 || parkingTemp1 == 1 && parkingTemp2 == 1 && temp == 1 )  // Checking Availbility of Parking Slots
    {
      for (pos = 0; pos <= 90; pos += 1)
      {
        myservo.write(pos);
        delay(15);
        Serial.println("Gate Opening");
      }

      delay(5000);

      for (pos = 90; pos >= 0; pos -= 1)
      {
        myservo.write(pos);
        delay(15);
        Serial.println("Gate Closing");
      }
    }
  }

  else if (gateTemp == 1 && temp == 0)
    Serial.println("No One is in front of Gate");
}

// Ultrasonic Calculations

void SonarSensor(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;
}
