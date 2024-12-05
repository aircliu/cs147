#define BLYNK_TEMPLATE_ID "TMPL2Nwh2csoD"
#define BLYNK_TEMPLATE_NAME "Fake Airtag"
#define BLYNK_AUTH_TOKEN "CvByw8Egr7QDhwunZqFFiCAm4V848_Bk"

#include <TinyGPS++.h>
#include <SparkFunLSM6DSO.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Pin Definitions
#define RXD2 25
#define TXD2 26
#define BUZZER_PIN 33
#define GPS_BAUD 9600

#define VPIN_LATITUDE V0  
#define VPIN_LONGITUDE V1 
#define VPIN_SPEED V2     
#define VPIN_MOTION V3    
#define VPIN_BUZZER V4
#define VPIN_BATTERY V5 
#define VPIN_STATUS V6
#define VPIN_DEVLAT V7
#define VPIN_DEVLONG V8
#define VPIN_DIST V9

float MOTION_THRESHOLD = 1.0;
bool motionDetected = false;
unsigned long lastMotionTime = 0;
unsigned long MOTION_TIMEOUT = 10000;
float currentSpeed = 0;
float lastAccel = 0;
unsigned long lastSpeedUpdate = 0;
const float ACCEL_THRESHOLD = 0.2;
const float SPEED_DECAY = 0.85;
const float SPEED_FACTOR = 0.5;

unsigned long lastValidData = 0;
const double EARTH_RADIUS_MILES = 3958.8; 
const double MILES_TO_FEET = 5280.0;
int sentenceCount = 0;
bool validDataReceived = false;
float gpsLat = 0.0;
float gpsLong = 0.0;
float distance = 0.0;

TinyGPSPlus gps;
LSM6DSO myIMU;
unsigned long lastBlynkUpdate = 0;
float devLat = 0.0;
float devLong = 0.0; 



// Add this function with your other functions
float readBatteryLevel() {
    const int SAMPLES = 10;
    float totalVoltage = 0;
    
    // Take multiple samples for more stable reading
    for(int i = 0; i < SAMPLES; i++) {
        int rawValue = analogRead(BATTERY_PIN);
        float voltage = (rawValue / (float)ADC_RESOLUTION) * ADC_REFERENCE * VOLTAGE_DIVIDER_RATIO;
        totalVoltage += voltage;
        delay(10);
    }
    
    // Calculate average voltage
    float averageVoltage = totalVoltage / SAMPLES;
    
    // Calculate percentage based on max and min voltages
    float percentage = ((averageVoltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
    percentage = constrain(percentage, 0.0, 100.0);
    
    batteryVoltage = averageVoltage;
    batteryPercentage = percentage;
    
    return percentage;
}

BLYNK_WRITE(VPIN_BUZZER) //get/change buzzer status from blynk
{
  int state = param.asInt();
  digitalWrite(BUZZER_PIN, state == 1 ? HIGH : LOW);
}

double degreesToRadians(double degrees)
{
  return degrees * (M_PI / 180.0);
}

double haversine(double lat1, double lon1, double lat2, double lon2) { //get distance between tracker and phone and return in feet
    lat1 = degreesToRadians(lat1);
    lon1 = degreesToRadians(lon1);
    lat2 = degreesToRadians(lat2);
    lon2 = degreesToRadians(lon2);

    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;

    double a = std::pow(std::sin(dLat / 2), 2) +
               std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dLon / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    double distanceMiles = EARTH_RADIUS_MILES * c;

    return distanceMiles * MILES_TO_FEET;
}
void processValues() {
  if (devLat != 0.0 && devLong != 0.0) {
    distance = haversine(gpsLat, gpsLong, devLat, devLong);
    Blynk.virtualWrite(VPIN_DIST, distance);
  }
}

BLYNK_WRITE(VPIN_DEVLAT) {
    devLat = std::stof(param.asString()); 
    if (devLong != 0.0) {
       processValues();  
    }
}

BLYNK_WRITE(VPIN_DEVLONG) {
    devLong = std::stof(param.asString()); 
    if (devLat != 0.0) {
      processValues();  
    }
}

/*class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.println("*********");
      Serial.print("New value: ");
      for (int i = 0; i < value.length(); i++)
        Serial.print(value[i]);
      Serial.println();
      Serial.println("*********");

      if (value == "BUZZ") {
        Serial.println("Starting buzzer");
      }
      else {
        int space = value.find(' ');
        if (space != std::string::npos) {
          std::string latitude = value.substr(0, space);
          std::string longitude = value.substr(space + 1);

          double distance = haversine(gpsLat, gpsLong, std::stod(latitude), std::stod(longitude));
          std::string currDistance = std::to_string(distance) + " " + "feet away";
          pCharacteristic->setValue(currDistance);
        }
      }
    }
  }
};*/

bool checkMotion()
{
  float x, y, z;
  float totalAccel;

  x = myIMU.readFloatAccelX();
  y = myIMU.readFloatAccelY();

  // Calculate total acceleration magnitude
  totalAccel = sqrt((x * x + y * y) / 2);

  if (totalAccel > MOTION_THRESHOLD)
  {
    motionDetected = true;
    lastMotionTime = millis();
    return true;
  }

  if (motionDetected && (millis() - lastMotionTime > MOTION_TIMEOUT))
  {
    motionDetected = false;
  }

  return motionDetected;
}

float calcSpeed()
{
  float x = myIMU.readFloatAccelX();
  float y = myIMU.readFloatAccelY();
  float totalAccel = sqrt(x * x + y * y);

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastSpeedUpdate) / 1000.0;
  lastSpeedUpdate = currentTime;

  if (abs(totalAccel) > ACCEL_THRESHOLD)
  {

    float accelMS = totalAccel * SPEED_FACTOR;

    currentSpeed += accelMS; 
  }
  else
  {
    currentSpeed *= SPEED_DECAY;
  }

  // Ensure speed doesn't go below 0
  if (currentSpeed < 0)
  {
    currentSpeed = 0;
  }
  return currentSpeed;
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    delay(100); 

  Serial2.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2); //start the gps module
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Wire.begin(21, 22);
  delay(10);
  while (!myIMU.begin())
  {
    Serial.println("Could not connect to IMU.");
    delay(1000);
  }
  Serial.println("Ready.");
  if (myIMU.initialize(BASIC_SETTINGS))
    Serial.println("Loaded Settings.");
  myIMU.setAccelRange(2);
  myIMU.setAccelDataRate(52);

  Blynk.begin(BLYNK_AUTH_TOKEN, "", ""); //put wifi details in

  // Initialize the Serial display
  /*BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_WRITE
                                        );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();*/
}

void loop()
{
  // Read and echo raw data
  bool newData = false;
  unsigned long startTime = millis();
  Blynk.run();

  if (gps.location.isValid())
  {
    //Blynk.virtualWrite(VPIN_STATUS, "GPS Fixed");
  }
  else
  {
    //Blynk.virtualWrite(VPIN_STATUS, "Searching GPS");
  }

  while ((millis() - startTime) < 1000)
  {
    while (Serial2.available() > 0)
    {
      char c = Serial2.read();

      if (gps.encode(c))
      {
        newData = true;
        sentenceCount++;
      }
    }
  }

  if (newData)
  {
    Serial.println("Parsed GPS Data:");
    Serial.println("----------------");

    if (gps.location.isValid())
    {
      validDataReceived = true;
      lastValidData = millis();

      Serial.print("Latitude: ");
      gpsLat = gps.location.lat();
      Serial.println(gpsLat, 6);
      Serial.print("Longitude: ");
      gpsLong = gps.location.lng();
      Serial.println(gpsLong, 6);

      if (gps.altitude.isValid())
      {
        Serial.print("Altitude: ");
        Serial.print(gps.altitude.meters());
        Serial.println(" m");
      }

      if (gps.speed.isValid())
      {
        Serial.print("Speed: ");
        Serial.print(gps.speed.kmph());
        Serial.println(" km/h");
      }
    }
  }

  float speed = calcSpeed();
  Serial.println(speed);
  if (gps.location.isValid())
  {
    Blynk.virtualWrite(VPIN_LATITUDE, gps.location.lat());
    Blynk.virtualWrite(VPIN_LONGITUDE, gps.location.lng());

  }
  Blynk.virtualWrite(VPIN_SPEED, speed);
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("\n*** NO GPS DATA RECEIVED ***");
    Serial.println("Check wiring and baud rate");
  }
  else if (!validDataReceived && millis() > 10000)
  {
    Serial.println("\n*** NO VALID POSITION DATA ***");
    Serial.println("Waiting for GPS fix");
  }
  else if (validDataReceived && (millis() - lastValidData) > 10000)
  {
    Serial.println("\n*** GPS SIGNAL LOST ***");
    Serial.println("Check GPS antenna and visibility");
  }

  Serial.println("\n==============================\n");

  bool isMoving = checkMotion();
  Blynk.virtualWrite(VPIN_MOTION, isMoving ? 1 : 0);

  delay(500);
}
