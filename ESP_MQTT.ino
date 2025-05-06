#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define BUTTON_PIN 0 

//For setting flag
bool flag = 0;                  // Toggle flag
bool buttonPressed = false;     // Track button press
unsigned long lastDebounceTime = 0; 
const unsigned long debounceDelay = 50; 

// For IR debounce
#define IR_SENSOR_PIN 16
bool irSensorTriggered = false; 
int IR = 0; // Variable to store the flag 
unsigned long lastDebounceTimeIR = 0; 

//Touch me
const int touchPin = A0; 
const int threshold = 10;
const int numReadings = 10; 

int readings[numReadings]; // Array to store the readings 
int readIndex = 0; // Index of the current reading 
int total = 0; // Total of the readings 
int touchFlag = 0; // Flag to indicate touch status 
bool previousTouch = false; // To track previous touch state
unsigned long lastDebounceTouch = 0; // Time of the last debounce


// Calibration offsets
float pitchOffset = 0.0;
float rollOffset = 0.0;

// Kalman filter variables
float qAngle = 0.0005; // Process noise variance for the accelerometer
float qBias = 0.003;  // Process noise variance for the gyro bias
float rMeasure = 0.03; // Measurement noise variance

struct Kalman {
    float angle; // Estimated angle
    float bias;  // Bias
    float rate;  // Rate of change of angle

    float P[2][2]; // Error covariance matrix
};

Kalman kalmanPitch = {0};
Kalman kalmanRoll = {0};


//Main code
Adafruit_MPU6050 mpu;

// WiFi credentials
const char* ssid = "Admin";
const char* password = "Admin";

// MQTT broker details
const char* mqtt_server = "Your MQTT server IP";
const int mqtt_port = 1883;
const char* mqtt_topic = "encoder/reading";

// Encoder pins
Encoder myEncoder(14, 12); // D5 (GPIO14) for CLK, D6 (GPIO12) for DT

WiFiClient espClient;
PubSubClient client(espClient);

///////////////////////////////////////////////////////Functions
void callback(char* topic, byte* payload, unsigned int length)  //Callback
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


void connectToMQTT() //MQTT connection
{
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to ");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.println(mqtt_port);
    if (client.connect("ESP8266Client")) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT connect failed, rc=");
      Serial.print(client.state());
      Serial.println(". Trying again in 5 seconds...");
      delay(5000);
    }
  }
}

void calibrateSensor() //Calibrate sensor
{
    const int calibrationSamples = 500;
    float sumPitch = 0.0;
    float sumRoll = 0.0;

    for (int i = 0; i < calibrationSamples; i++) {
        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp)) {
            float accelX = a.acceleration.x;
            float accelY = a.acceleration.y;
            float accelZ = a.acceleration.z;

            float accPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
            float accRoll = atan2(-accelX, accelZ) * 180 / PI;

            sumPitch += accPitch;
            sumRoll += accRoll;
        }
        delay(10);
    }

    pitchOffset = sumPitch / calibrationSamples;
    rollOffset = sumRoll / calibrationSamples;
    Serial.print("Pitch offset: ");
    Serial.println(pitchOffset);
    Serial.print("Roll offset: ");
    Serial.println(rollOffset);
}


float kalmanFilter(Kalman &kalman, float newAngle, float newRate, float dt)  //Kalman Filter 
{
    // Predict
    kalman.rate = newRate - kalman.bias;
    kalman.angle += dt * kalman.rate;

    kalman.P[0][0] += dt * (dt * kalman.P[1][1] - kalman.P[0][1] - kalman.P[1][0] + qAngle);
    kalman.P[0][1] -= dt * kalman.P[1][1];
    kalman.P[1][0] -= dt * kalman.P[1][1];
    kalman.P[1][1] += qBias * dt;

    // Update
    float S = kalman.P[0][0] + rMeasure; // Estimate error
    float K[2];                          // Kalman gain
    K[0] = kalman.P[0][0] / S;
    K[1] = kalman.P[1][0] / S;

    float y = newAngle - kalman.angle; // Angle difference
    kalman.angle += K[0] * y;
    kalman.bias += K[1] * y;

    float P00Temp = kalman.P[0][0];
    float P01Temp = kalman.P[0][1];

    kalman.P[0][0] -= K[0] * P00Temp;
    kalman.P[0][1] -= K[0] * P01Temp;
    kalman.P[1][0] -= K[1] * P00Temp;
    kalman.P[1][1] -= K[1] * P01Temp;

    return kalman.angle;
}

bool getFlag() 
{
  static bool lastButtonState = HIGH; // Button state in the last function call
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Detect a button state change
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis(); // Reset debounce timer
  }

  // Check if the state is stable after debounce delay
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentButtonState == LOW && !buttonPressed) {
      buttonPressed = true; // Mark as pressed
      flag = !flag; // Toggle the flag
    }
  }

  // Reset buttonPressed if the button is released
  if (currentButtonState == HIGH) {
    buttonPressed = false;
  }

  lastButtonState = currentButtonState; // Save the state for next iteration
  return flag; // Return the current flag value
}


int debounceIRSensor() 
{ 
  static bool lastIRState = HIGH; // IR sensor state in the last function call 
  bool currentIRState = digitalRead(IR_SENSOR_PIN); 
  
  // Detect a state change in the IR sensor 
  if (currentIRState != lastIRState) 
  { lastDebounceTimeIR = millis(); // Reset debounce timer 
  } 
  
  // Check if the state is stable after debounce delay 
  if ((millis() - lastDebounceTimeIR) > debounceDelay) { 
    if (currentIRState == LOW && !irSensorTriggered) { 
      irSensorTriggered = true; // Mark as triggered 
      IR = (IR == 0) ? 1 : 0; // Toggle the IR flag (1 or 0) 
      } 
    } 
      
   // Reset irSensorTriggered if the IR sensor is not triggered 
   if (currentIRState == HIGH) { 
    irSensorTriggered = false; 
    } 
    
    lastIRState = currentIRState; 
    return IR; 
}

int calculateMovingAverage(int newValue) { 
  // Subtract the last reading 
  total = total - readings[readIndex]; 
  
  // Read the new value 
  readings[readIndex] = newValue; 
  
  // Add the new value to the total 
  total = total + readings[readIndex]; 
  
  // Advance to the next position in the array 
  readIndex = readIndex + 1; 
  
  // If we're at the end of the array, wrap around to the beginning 
  if (readIndex >= numReadings) { readIndex = 0; } 
  
  // Calculate the average 
   int average = total / numReadings;
   //Serial.print("Touch value"); 
   //Serial.println(average); 
   
  
  // Determine touch status and set the flag 
  bool currentTouch = (average > threshold); 
  
  // Debounce logic and toggle flag 
  unsigned long currentTime = millis(); 
  if (currentTouch != previousTouch && (currentTime - lastDebounceTouch) > debounceDelay) { 
    lastDebounceTouch = currentTime; 
    if (currentTouch) 
    { touchFlag = !touchFlag; // Toggle the touch flag 
    }
   }
  previousTouch = currentTouch; // Update previous touch state 
      
  return touchFlag; 
}
//////////////////////////////////////////////////////////////////////////////

void setup() 
{
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  connectToMQTT();
  
   //For MPU
  if (!mpu.begin()) 
  { 
    Serial.println("Failed to find MPU6050 chip"); 
    while (1) 
    { delay(10); 
    } 
   } 
  Serial.println("MPU6050 Found!"); 
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); 
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  // Perform initial calibration
  Serial.println("Calibrating... Please keep the sensor still.");
  delay(5000);
  calibrateSensor();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  for (int i = 0; i < numReadings; i++) 
  { readings[i] = 0; 
  }
}


void loop() 
{
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();

  ////////////////////////////////////////////////Kalman MPU
  yield();

  static unsigned long lastTime = 0;
  const unsigned long interval = 5; // ~60Hz
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) 
  {
      //lastTime = currentTime;

      sensors_event_t a, g, temp;
      if (!mpu.getEvent(&a, &g, &temp)) {
          Serial.println("Failed to get sensor event");
        return;
      }

   float accelX = a.acceleration.x;
   float accelY = a.acceleration.y;
   float accelZ = a.acceleration.z;
   float gyroX = g.gyro.x * (PI / 180.0); // Convert to radians
   float gyroY = g.gyro.y * (PI / 180.0); // Convert to radians
   float temperature = temp.temperature;
   float dt = (currentTime-lastTime) / 1000.0;

   // Calculate accelerometer angles
   float accPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI - pitchOffset;
   float accRoll = atan2(-accelX, accelZ) * 180 / PI - rollOffset;

   // Apply Kalman filter
   float pitch = kalmanFilter(kalmanPitch, accPitch, gyroX, dt);
   float roll = kalmanFilter(kalmanRoll, accRoll, gyroY, dt);

   IR= debounceIRSensor();
   flag = getFlag();
   
   int touchValue = analogRead(touchPin); 
   int touchFlag = calculateMovingAverage(touchValue);
   
   
   Serial.print("Pitch: ");
   Serial.println(roll);

   Serial.print("Roll: ");
   Serial.println(pitch); 

   Serial.print("Temperature: ");
   Serial.println(temperature);

   Serial.print("IR: ");
   Serial.println(IR);

   Serial.print("Flag: ");
   Serial.println(flag);

   Serial.print("TouchFlag: ");
   Serial.println(touchFlag);
   
  //////////////////////////////////////////////////////////Encoder
  int reading = myEncoder.read();
  bool buttonPressed = digitalRead(13) == LOW; // Button status

  // Create a JSON formatted message
  String message = "{\"encoderValue\": " + String(reading) + 
                   ", \"buttonPressed\": " + String(buttonPressed) + 
                   ", \"Pitch\": " + String(roll) + 
                   ", \"Roll\": " + String(pitch) + 
                   ", \"Temp\": " + String(temperature) + 
                   ", \"Flag\": " + String(flag) + 
                   ", \"IR\": " + String(IR) + 
                   ", \"TouchFlag\": " + String(touchFlag) +"}";
  
  client.publish(mqtt_topic, message.c_str()); 
  //Serial.println("Encoder and sensor reading: " + message);
  lastTime = currentTime;
  
 }
 delay(10);
}
