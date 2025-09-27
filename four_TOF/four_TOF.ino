#include <Wire.h>
#include <VL53L0X.h>

// Number of sensors
#define SENSOR_COUNT 4

// Moving Median Filter - Best for fast response + noise filtering
class MedianFilter {
private:
  static const int WINDOW_SIZE = 5;
  float readings[WINDOW_SIZE];
  int readIndex;
  int numReadings;

public:
  MedianFilter() {
    readIndex = 0;
    numReadings = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      readings[i] = 0;
    }
  }
  
  float updateEstimate(float measurement) {
    readings[readIndex] = measurement;
    readIndex = (readIndex + 1) % WINDOW_SIZE;
    if (numReadings < WINDOW_SIZE) numReadings++;
    
    // Create sorted copy
    float sorted[WINDOW_SIZE];
    for (int i = 0; i < numReadings; i++) {
      sorted[i] = readings[i];
    }
    
    // Simple insertion sort for small array
    for (int i = 1; i < numReadings; i++) {
      float key = sorted[i];
      int j = i - 1;
      while (j >= 0 && sorted[j] > key) {
        sorted[j + 1] = sorted[j];
        j--;
      }
      sorted[j + 1] = key;
    }
    
    return sorted[numReadings / 2];
  }
  
  void reset() {
    readIndex = 0;
    numReadings = 0;
  }
};

// XSHUT pins for each sensor (GPIO12-GPIO15)
const int xshutPins[SENSOR_COUNT] = {12, 13, 14, 15};

// I2C addresses to assign to each sensor (default is 0x29)
const int sensorAddresses[SENSOR_COUNT] = {0x30, 0x31, 0x32, 0x33};

// Create sensor objects
VL53L0X sensors[SENSOR_COUNT];

// Offset calibration values for each sensor (in mm)
// float sensorOffsets[SENSOR_COUNT] = {0.0, 0.0, 0.0, 0.0};

// Median filters for each sensor
MedianFilter medianFilters[SENSOR_COUNT];

// I2C pins for ESP8266
// SDA = D2 (GPIO4)
// SCL = D1 (GPIO5)

void setup() {
  Serial.begin(115200);
  Serial.println("VL53L0X Multiple Sensor Test");

  // Initialize I2C with custom pins
  Wire.begin(4, 5); // SDA=GPIO4(D2), SCL=GPIO5(D1)
  
  // Initialize XSHUT pins
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW); // Keep all sensors in reset
  }
  
  delay(100);
  
  // Initialize each sensor with unique I2C address
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("Initializing sensor ");
    Serial.print(i + 1);
    Serial.print(" on address 0x");
    Serial.println(sensorAddresses[i], HEX);
    
    // Enable current sensor by bringing XSHUT high
    digitalWrite(xshutPins[i], HIGH);
    delay(50);
    
    // Initialize the sensor
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i + 1);
      while (1) {}
    }
    
    // Set the I2C address for this sensor
    sensors[i].setAddress(sensorAddresses[i]);
    
    // Configure sensor for better performance
    sensors[i].setMeasurementTimingBudget(50000); // 50ms measurement time
    
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.println(" initialized successfully!");
  }
  
  Serial.println("All sensors initialized!");
  Serial.println("Starting measurements...");
  Serial.println();
}

void loop() {
  Serial.print("Filtered (cm): ");
  
  // Read from all sensors
  for (int i = 0; i < SENSOR_COUNT; i++) {
    uint16_t rawDistance = sensors[i].readRangeSingleMillimeters();
    
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(":");
    
    if (sensors[i].timeoutOccurred()) {
      Serial.print("TIMEOUT");
    } else {
      // Apply offset correction
      // float correctedMM = rawDistance - sensorOffsets[i];
      // float rawCM = correctedMM / 10.0;
      float rawCM = rawDistance / 10.0; //remove this one if calibration is done and put previous one
      
      float filteredCM = medianFilters[i].updateEstimate(rawCM);
      Serial.print(filteredCM, 1);
    }
    
    if (i < SENSOR_COUNT - 1) {
      Serial.print(" | ");
    }
  }
  
  Serial.println();
  delay(100);
}

// Function to read a specific sensor
uint16_t readSensor(int sensorIndex) {
  if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
    return sensors[sensorIndex].readRangeSingleMillimeters();
  }
  return 0;
}

// Function to check if a sensor has timed out
bool sensorTimeout(int sensorIndex) {
  if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
    return sensors[sensorIndex].timeoutOccurred();
  }
  return true;
}

// Function to get median filtered distance in centimeters
float getFilteredDistanceCM(int sensorIndex) {
  if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
    uint16_t rawDistance = sensors[sensorIndex].readRangeSingleMillimeters();
    if (sensors[sensorIndex].timeoutOccurred()) {
      return -1;
    }
    // Apply offset correction
    // float correctedMM = rawDistance - sensorOffsets[sensorIndex];
    // float rawCM = correctedMM / 10.0;
    float rawCM = rawDistance / 10.0; //remove this and this if previous calibration done
    
    return medianFilters[sensorIndex].updateEstimate(rawCM);
  }
  return -1;
}

// Function to reset a specific filter
void resetFilter(int sensorIndex) {
  if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
    medianFilters[sensorIndex].reset();
  }
}

// Function to set individual sensor offset
/*
void setSensorOffset(int sensorIndex, float offsetMM) {
  if (sensorIndex >= 0 && sensorIndex < SENSOR_COUNT) {
    sensorOffsets[sensorIndex] = offsetMM;
  }
}
*/