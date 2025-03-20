/**************************************************************************
 * 
 * Author     : [PIE.SPACE]
 * Start Date : [10-1-2025]
 * End Date   : [22-1-2025]
 * 
 * MPU6050 Check 
**************************************************************************/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h> // For WS2812B LED stripe

/**************************************************************************
  Function Prototypes
**************************************************************************/
void calibrateAccelerometer();
void calibrateGyroscope();
void sensorSelfTest();
void saveCalibrationData();
void loadCalibrationData();
void resetDisplacement();
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

/**************************************************************************
  EEPROM Calibration Storage Definitions
**************************************************************************/
#define EEPROM_CALIB_SIGNATURE 0xA55A55A5UL
#define EEPROM_SIGNATURE_ADDR  0
#define EEPROM_CALIB_ADDR      4

struct CalibrationData {
  float accXOffset;
  float accYOffset;
  float accZOffset;
  float gyroXOffset;
  float gyroYOffset;
  float gyroZOffset;
};

CalibrationData calibData;
bool calibrationLoaded = false;

/**************************************************************************
  Global Objects and Variables
**************************************************************************/
Adafruit_MPU6050 mpu;

// Timing for sensor updates
unsigned long previousTime = 0;

// Displacement calculation (via double integration)
float velocityX = 0.0f, velocityY = 0.0f, velocityZ = 0.0f;
float displacementX = 0.0f, displacementY = 0.0f, displacementZ = 0.0f;

// Shock detection
#define SHOCK_THRESHOLD 15.0f  // m/s^2 threshold
bool shockDetected = false;

// Vibration sensing (rolling standard deviation)
#define VIBRATION_WINDOW_SIZE  50
float vibrationBuffer[VIBRATION_WINDOW_SIZE];
int vibrationIndex = 0;
bool bufferFilled = false;
float vibrationLevel = 0.0f;

// Min/Max tracking for angular velocity and linear acceleration
float minAngularVelocity = 3.4e38f;
float maxAngularVelocity = -3.4e38f;
float minLinearAcceleration = 3.4e38f;
float maxLinearAcceleration = -3.4e38f;

// Noise threshold for acceleration (to mitigate drift in integration)
#define ACC_NOISE_THRESHOLD 0.1f

// Madgwick variables
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
const float beta = 0.1f; // Madgwick algorithm gain

/**************************************************************************
  WS2812B LED Stripe Definitions and Variables
**************************************************************************/
#define LED_PIN 3       // Pin connected to the WS2812B stripe 
#define NUM_LEDS 1      // Number of LEDs in the stripe
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// For continuous rainbow glow (non-blocking)
unsigned long previousRainbowMillis = 0;
uint8_t rainbowPos = 0;
const uint16_t RAINBOW_INTERVAL = 20; // Update interval in milliseconds

/**************************************************************************
  Musical Note Definitions
**************************************************************************/
#define Buzzer_Pin 9

#define NOTE_B0   31
#define NOTE_C1   33
#define NOTE_CS1  35
#define NOTE_D1   37
#define NOTE_DS1  39
#define NOTE_E1   41
#define NOTE_F1   44
#define NOTE_FS1  46
#define NOTE_G1   49
#define NOTE_GS1  52
#define NOTE_A1   55
#define NOTE_AS1  58
#define NOTE_B1   62
#define NOTE_C2   65
#define NOTE_CS2  69
#define NOTE_D2   73
#define NOTE_DS2  78
#define NOTE_E2   82
#define NOTE_F2   87
#define NOTE_FS2  93
#define NOTE_G2   98
#define NOTE_GS2  104
#define NOTE_A2   110
#define NOTE_AS2  117
#define NOTE_B2   123
#define NOTE_C3   131
#define NOTE_CS3  139
#define NOTE_D3   147
#define NOTE_DS3  156
#define NOTE_E3   165
#define NOTE_F3   175
#define NOTE_FS3  185
#define NOTE_G3   196
#define NOTE_GS3  208
#define NOTE_A3   220
#define NOTE_AS3  233
#define NOTE_B3   247
#define NOTE_C4   262
#define NOTE_CS4  277
#define NOTE_D4   294
#define NOTE_DS4  311
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_FS4  370
#define NOTE_G4   392
#define NOTE_GS4  415
#define NOTE_A4   440
#define NOTE_AS4  466
#define NOTE_B4   494
#define NOTE_C5   523
#define NOTE_CS5  554
#define NOTE_D5   587
#define NOTE_DS5  622
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_FS5  740
#define NOTE_G5   784
#define NOTE_GS5  831
#define NOTE_A5   880
#define NOTE_AS5  932
#define NOTE_B5   988
#define NOTE_C6   1047
#define NOTE_CS6  1109
#define NOTE_D6   1175
#define NOTE_DS6  1245
#define NOTE_E6   1319
#define NOTE_F6   1397
#define NOTE_FS6  1480
#define NOTE_G6   1568
#define NOTE_GS6  1661
#define NOTE_A6   1760
#define NOTE_AS6  1865
#define NOTE_B6   1976
#define NOTE_C7   2093
#define NOTE_CS7  2217
#define NOTE_D7   2349
#define NOTE_DS7  2489
#define NOTE_E7   2637
#define NOTE_F7   2794
#define NOTE_FS7  2960
#define NOTE_G7   3136
#define NOTE_GS7  3322
#define NOTE_A7   3520
#define NOTE_AS7  3729
#define NOTE_B7   3951
#define NOTE_C8   4186
#define NOTE_CS8  4435
#define NOTE_D8   4699
#define NOTE_DS8  4978

/**************************************************************************
  WS2812B LED Helper Functions
**************************************************************************/
// The Wheel function maps a number between 0 and 255 to a color value.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

// Startup LED flash with rainbow for 5 seconds.
void StartUpFlashLed() {
  Serial.println("Showing rainbow for 5 seconds...");
  unsigned long startTime = millis();
  int iteration = 0;
  while (millis() - startTime < 5000) {
    // Use an unsigned type for the LED index to match numPixels() return type.
    for(uint16_t i = 0; i < strip.numPixels(); i++) {
      int colorIndex = (i + iteration) & 255;
      strip.setPixelColor(i, Wheel(colorIndex));
    }
    strip.show();
    iteration++;
    delay(10);
  }
  strip.clear();
  strip.show();
}

/**************************************************************************
  Buzzer Melody Patterns
**************************************************************************/
static int droneStartupMelody[][2] = {
  {NOTE_G4,  200},
  {NOTE_A4,  200},
  {NOTE_B4,  250},
  {NOTE_G4,  200},
  {NOTE_A4,  200},
  {NOTE_B4,  300},
  {0,        150},
  {NOTE_D5,  200},
  {NOTE_FS5, 200},
  {NOTE_A5,  250},
  {NOTE_D5,  200},
  {NOTE_FS5, 200},
  {NOTE_A5,  300},
};
int startupLength = sizeof(droneStartupMelody) / sizeof(droneStartupMelody[0]);

#define ASC_SWEEP_START_FREQ   300
#define ASC_SWEEP_END_FREQ    1500
#define ASC_SWEEP_STEP         50
#define ASC_SWEEP_DELAY        30

#define DESC_SWEEP_START_FREQ 1500
#define DESC_SWEEP_END_FREQ    300
#define DESC_SWEEP_STEP         50
#define DESC_SWEEP_DELAY        30

#define URGENT_BEEP_REPEATS    4
#define URGENT_BEEP_ON_MS    150
#define URGENT_BEEP_OFF_MS   150

static int victoryMelody[][2] = {
  {NOTE_C4, 200}, {NOTE_E4, 200}, {NOTE_G4, 200},
  {NOTE_C5, 250},
  {0, 100},
  {NOTE_G4, 200}, {NOTE_E4, 200}, {NOTE_C4, 300}
};
int victoryLength = sizeof(victoryMelody) / sizeof(victoryMelody[0]);

static int errorMelody[][2] = {
  {NOTE_G3, 200}, {NOTE_E3, 200}, {NOTE_C3, 300},
  {0, 100},
  {NOTE_C3, 150}, {NOTE_C3, 150}
};
int errorLength = sizeof(errorMelody) / sizeof(errorMelody[0]);

static int futuristicArp[][2] = {
  {NOTE_C4,  100},
  {NOTE_E4,  100},
  {NOTE_G4,  100},
  {NOTE_C5,  100},
  {NOTE_G4,  100},
  {NOTE_E4,  100},
  {NOTE_C4,  100},
  {0,        100}
};
int futuristicArpLength = sizeof(futuristicArp) / sizeof(futuristicArp[0]);

// Helper function to play a melody array.
void playMelody(const int melody[][2], int length) {
  for (int i = 0; i < length; i++) {
    int freq = melody[i][0];
    int dur  = melody[i][1];
    if (freq > 0) {
      tone(Buzzer_Pin, freq, dur);
    } else {
      noTone(Buzzer_Pin);
    }
    delay(dur + 20);
    noTone(Buzzer_Pin);
  }
}

void playStartupMelody() {
  Serial.println("Playing Drone Startup Melody...");
  playMelody(droneStartupMelody, startupLength);
}
void ascendingSweep() {
  Serial.println("Ascending Sweep...");
  for (int freq = ASC_SWEEP_START_FREQ; freq <= ASC_SWEEP_END_FREQ; freq += ASC_SWEEP_STEP) {
    tone(Buzzer_Pin, freq);
    delay(ASC_SWEEP_DELAY);
  }
  noTone(Buzzer_Pin);
}
void descendingSweep() {
  Serial.println("Descending Sweep...");
  for (int freq = DESC_SWEEP_START_FREQ; freq >= DESC_SWEEP_END_FREQ; freq -= DESC_SWEEP_STEP) {
    tone(Buzzer_Pin, freq);
    delay(DESC_SWEEP_DELAY);
  }
  noTone(Buzzer_Pin);
}
void urgentBeep() {
  Serial.println("Urgent Beep Pattern...");
  for (int i = 0; i < URGENT_BEEP_REPEATS; i++) {
    digitalWrite(Buzzer_Pin, HIGH);
    delay(URGENT_BEEP_ON_MS);
    digitalWrite(Buzzer_Pin, LOW);
    delay(URGENT_BEEP_OFF_MS);
  }
  noTone(Buzzer_Pin);
}
void playVictoryMelody() {
  Serial.println("Playing Victory Melody...");
  playMelody(victoryMelody, victoryLength);
}
void playErrorMelody() {
  Serial.println("Playing Error Melody...");
  playMelody(errorMelody, errorLength);
}
void playFuturisticArpeggio() {
  Serial.println("Playing Futuristic Arpeggio...");
  for (int i = 0; i < 3; i++) {
    playMelody(futuristicArp, futuristicArpLength);
  }
}
void randomFuturisticBeep(int beepCount) {
  Serial.println("Playing Random Futuristic Beeps...");
  for (int i = 0; i < beepCount; i++) {
    int randFreq = random(200, 2000);
    int randDur  = random(50, 300);
    tone(Buzzer_Pin, randFreq, randDur);
    delay(randDur + 30);
    noTone(Buzzer_Pin);
  }
}

// Renamed original buzzer setup & loop (not used):
void unusedBuzzerSetup() {
  // Original buzzer code remains here but is not called.
}
void unusedBuzzerLoop() {
  // Empty in the original example.
}

/**************************************************************************
  Arduino Setup (Primary)
**************************************************************************/
void setup() {
  // 1) Set up buzzer pin
  pinMode(Buzzer_Pin, OUTPUT);

  // 2) Initialize LED stripe for WS2812B
  strip.begin();
  strip.show(); // Ensure all LEDs are off initially

  // 3) Start serial communication at 230400 baud
  Serial.begin(230400); 
  while (!Serial) { delay(1); }
  Serial.println("Starting. Buzzer Tone first...");

  // 4) Play buzzer melodies and sweeps
  playStartupMelody();    // Drone-style startup
  ascendingSweep();       // Ascending sweep
  descendingSweep();      // Descending sweep
  urgentBeep();           // Urgent beep pattern

  // 5) Flash the LED stripe with a rainbow pattern for 5 seconds
  StartUpFlashLed();
  urgentBeep();

  Serial.println("Buzzer sequences done. Now initializing MPU6050...");

  // 6) Initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println(F("Failed to find MPU6050 chip. Stopping."));
    while (1) { delay(10); }
  }
  Serial.println(F("MPU6050 Found."));

  // Set accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.print(F("Accelerometer range set to: "));
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:  Serial.println(F("±2G"));  break;
    case MPU6050_RANGE_4_G:  Serial.println(F("±4G"));  break;
    case MPU6050_RANGE_8_G:  Serial.println(F("±8G"));  break;
    case MPU6050_RANGE_16_G: Serial.println(F("±16G")); break;
  }

  // Set gyroscope range
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  Serial.print(F("Gyro range set to: "));
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:  Serial.println(F("±250 deg/s"));  break;
    case MPU6050_RANGE_500_DEG:  Serial.println(F("±500 deg/s"));  break;
    case MPU6050_RANGE_1000_DEG: Serial.println(F("±1000 deg/s")); break;
    case MPU6050_RANGE_2000_DEG: Serial.println(F("±2000 deg/s")); break;
  }

  // Set digital low-pass filter
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print(F("Filter bandwidth set to: "));
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ: Serial.println(F("260 Hz")); break;
    case MPU6050_BAND_184_HZ: Serial.println(F("184 Hz")); break;
    case MPU6050_BAND_94_HZ:  Serial.println(F("94 Hz"));  break;
    case MPU6050_BAND_44_HZ:  Serial.println(F("44 Hz"));  break;
    case MPU6050_BAND_21_HZ:  Serial.println(F("21 Hz"));  break;
    case MPU6050_BAND_10_HZ:  Serial.println(F("10 Hz"));  break;
    case MPU6050_BAND_5_HZ:   Serial.println(F("5 Hz"));   break;
  }

  // Sensor self-test
  sensorSelfTest();

  // Load calibration data; if unavailable, perform calibration
  loadCalibrationData();
  if (!calibrationLoaded) {
    calibrateAccelerometer();
    calibrateGyroscope();
  }

  previousTime = millis();
  Serial.println("Setup complete.\n");
}

/**************************************************************************
  Arduino Loop (Primary)
**************************************************************************/
void loop() {
  // 1) Get MPU sensor events
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  // 2) Apply calibration offsets
  float ax = accelEvent.acceleration.x - calibData.accXOffset;
  float ay = accelEvent.acceleration.y - calibData.accYOffset;
  float az = accelEvent.acceleration.z - calibData.accZOffset;
  float gx = gyroEvent.gyro.x - calibData.gyroXOffset;
  float gy = gyroEvent.gyro.y - calibData.gyroYOffset;
  float gz = gyroEvent.gyro.z - calibData.gyroZOffset;

  // 3) Calculate time difference
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0f;
  previousTime = currentTime;

  // 4) Madgwick sensor fusion update
  MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);

  // 5) Convert quaternion to Euler angles (in degrees)
  float rollRad  = atan2(2.0f*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
  float pitchRad = asin(2.0f*(q0*q2 - q3*q1));
  float yawRad   = atan2(2.0f*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
  float roll  = rollRad  * (180.0f / PI);
  float pitch = pitchRad * (180.0f / PI);
  float yaw   = yawRad   * (180.0f / PI);

  // 6) Gravity removal for displacement
  float gravityX = sin(pitchRad) * 9.81f;
  float gravityY = -sin(rollRad) * 9.81f;
  float gravityZ = cos(pitchRad)*cos(rollRad) * 9.81f;
  float linAccX = ax - gravityX;
  float linAccY = ay - gravityY;
  float linAccZ = az - gravityZ;

  // 7) Apply noise threshold
  if (fabs(linAccX) < ACC_NOISE_THRESHOLD) linAccX = 0;
  if (fabs(linAccY) < ACC_NOISE_THRESHOLD) linAccY = 0;
  if (fabs(linAccZ) < ACC_NOISE_THRESHOLD) linAccZ = 0;

  // 8) Double integration for displacement
  velocityX += linAccX * dt;
  velocityY += linAccY * dt;
  velocityZ += linAccZ * dt;
  displacementX += velocityX * dt;
  displacementY += velocityY * dt;
  displacementZ += velocityZ * dt;

  // 9) Convert gyro readings to degrees per second
  float gxDeg = gx * (180.0f / PI);
  float gyDeg = gy * (180.0f / PI);
  float gzDeg = gz * (180.0f / PI);

  // 10) Track min/max angular velocity and linear acceleration
  float angularVelMag = sqrt(gxDeg*gxDeg + gyDeg*gyDeg + gzDeg*gzDeg);
  if (angularVelMag < minAngularVelocity) minAngularVelocity = angularVelMag;
  if (angularVelMag > maxAngularVelocity) maxAngularVelocity = angularVelMag;

  float linearAccMag = sqrt(ax*ax + ay*ay + az*az);
  if (linearAccMag < minLinearAcceleration) minLinearAcceleration = linearAccMag;
  if (linearAccMag > maxLinearAcceleration) maxLinearAcceleration = linearAccMag;

  // 11) Shock detection
  shockDetected = (linearAccMag > SHOCK_THRESHOLD);

  // 12) Vibration (rolling standard deviation)
  vibrationBuffer[vibrationIndex] = linearAccMag;
  vibrationIndex = (vibrationIndex + 1) % VIBRATION_WINDOW_SIZE;
  if (vibrationIndex == 0) {
    bufferFilled = true;
  }
  if (bufferFilled) {
    float mean = 0.0f;
    for (int i = 0; i < VIBRATION_WINDOW_SIZE; i++) {
      mean += vibrationBuffer[i];
    }
    mean /= (float)VIBRATION_WINDOW_SIZE;
    float sumSq = 0.0f;
    for (int i = 0; i < VIBRATION_WINDOW_SIZE; i++) {
      float diff = (vibrationBuffer[i] - mean);
      sumSq += diff * diff;
    }
    vibrationLevel = sqrt(sumSq / (float)VIBRATION_WINDOW_SIZE);
  }

  // 13) Print data in CSV format to Serial Monitor
  Serial.print(roll, 2);                  Serial.print(",");
  Serial.print(pitch, 2);                 Serial.print(",");
  Serial.print(yaw, 2);                   Serial.print(",");
  Serial.print(q0, 4);                    Serial.print(",");
  Serial.print(q1, 4);                    Serial.print(",");
  Serial.print(q2, 4);                    Serial.print(",");
  Serial.print(q3, 4);                    Serial.print(",");
  Serial.print(ax, 2);                    Serial.print(",");
  Serial.print(ay, 2);                    Serial.print(",");
  Serial.print(az, 2);                    Serial.print(",");
  Serial.print(gxDeg, 2);                 Serial.print(",");
  Serial.print(gyDeg, 2);                 Serial.print(",");
  Serial.print(gzDeg, 2);                 Serial.print(",");
  Serial.print(tempEvent.temperature, 1); Serial.print(",");
  Serial.print(minAngularVelocity, 2);    Serial.print(",");
  Serial.print(maxAngularVelocity, 2);    Serial.print(",");
  Serial.print(minLinearAcceleration, 2); Serial.print(",");
  Serial.print(maxLinearAcceleration, 2); Serial.print(",");
  Serial.print(displacementX, 2);         Serial.print(",");
  Serial.print(displacementY, 2);         Serial.print(",");
  Serial.print(displacementZ, 2);         Serial.print(",");
  Serial.print(shockDetected ? 1 : 0);    Serial.print(",");
  Serial.println(vibrationLevel, 2);

  // 14) Serial command processing (if any command is sent)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("calibrate accelerometer")) {
      calibrateAccelerometer();
    } else if (command.equalsIgnoreCase("calibrate gyroscope")) {
      calibrateGyroscope();
    } else if (command.equalsIgnoreCase("save calibration")) {
      saveCalibrationData();
    } else if (command.equalsIgnoreCase("load calibration")) {
      loadCalibrationData();
    } else if (command.equalsIgnoreCase("reset displacement")) {
      resetDisplacement();
    } else if (command.equalsIgnoreCase("self test")) {
      sensorSelfTest();
    }
  }

  // 15) Optionally, a small beep in each loop iteration (currently commented out)
  // tone(Buzzer_Pin, 1000, 100);

  // 16) Continuous non-blocking rainbow glow update for WS2812B LED stripe
  if (millis() - previousRainbowMillis >= RAINBOW_INTERVAL) {
    previousRainbowMillis = millis();
    rainbowPos++;  // Increment color position for smooth transition
    // Since only one LED is used, update pixel 0 with a rainbow color
    strip.setPixelColor(0, Wheel(rainbowPos));
    strip.show();
  }
}

/**************************************************************************
  Accelerometer Calibration
**************************************************************************/
void calibrateAccelerometer() {
  Serial.println(F("\n-- Accelerometer Calibration --"));
  Serial.println(F("Keep the sensor flat and still..."));

  const int numSamples = 100;
  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z;
    delay(10);
  }
  float avgX = sumX / numSamples;
  float avgY = sumY / numSamples;
  float avgZ = sumZ / numSamples;

  calibData.accXOffset = avgX;
  calibData.accYOffset = avgY;
  // Subtract 9.81 so the offset yields ~9.81 when stationary
  calibData.accZOffset = avgZ - 9.81f;

  Serial.print(F("Accelerometer Offsets: X="));
  Serial.print(calibData.accXOffset, 3);
  Serial.print(F(", Y="));
  Serial.print(calibData.accYOffset, 3);
  Serial.print(F(", Z="));
  Serial.print(calibData.accZOffset, 3);
  Serial.println(F(" (Z includes gravity)"));
  Serial.println(F("Accelerometer calibration complete.\n"));
}

/**************************************************************************
  Gyroscope Calibration
**************************************************************************/
void calibrateGyroscope() {
  Serial.println(F("\n-- Gyroscope Calibration --"));
  Serial.println(F("Keep the sensor still (no rotation)..."));

  const int numSamples = 100;
  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(10);
  }
  calibData.gyroXOffset = sumX / numSamples;
  calibData.gyroYOffset = sumY / numSamples;
  calibData.gyroZOffset = sumZ / numSamples;

  Serial.print(F("Gyroscope Offsets: X="));
  Serial.print(calibData.gyroXOffset, 5);
  Serial.print(F(", Y="));
  Serial.print(calibData.gyroYOffset, 5);
  Serial.print(F(", Z="));
  Serial.print(calibData.gyroZOffset, 5);
  Serial.println(F("\nGyroscope calibration complete.\n"));
}

/**************************************************************************
  Sensor Self-Test Routine
**************************************************************************/
void sensorSelfTest() {
  Serial.println(F("\n-- Sensor Self-Test --"));
  const int numSamples = 50;
  float sumAccX = 0.0f, sumAccY = 0.0f, sumAccZ = 0.0f;
  float sumGyroX = 0.0f, sumGyroY = 0.0f, sumGyroZ = 0.0f;

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumAccX += a.acceleration.x;
    sumAccY += a.acceleration.y;
    sumAccZ += a.acceleration.z;
    sumGyroX += g.gyro.x;
    sumGyroY += g.gyro.y;
    sumGyroZ += g.gyro.z;
    delay(5);
  }

  float avgAccX = sumAccX / numSamples;
  float avgAccY = sumAccY / numSamples;
  float avgAccZ = sumAccZ / numSamples;
  float avgGyroX = sumGyroX / numSamples;
  float avgGyroY = sumGyroY / numSamples;
  float avgGyroZ = sumGyroZ / numSamples;

  Serial.print(F("Average Acceleration: X="));
  Serial.print(avgAccX, 3);
  Serial.print(F(", Y="));
  Serial.print(avgAccY, 3);
  Serial.print(F(", Z="));
  Serial.print(avgAccZ, 3);
  Serial.println();

  Serial.print(F("Average Gyro: X="));
  Serial.print(avgGyroX, 5);
  Serial.print(F(", Y="));
  Serial.print(avgGyroY, 5);
  Serial.print(F(", Z="));
  Serial.print(avgGyroZ, 5);
  Serial.println();

  if (fabs(avgAccX) > 1.0 || fabs(avgAccY) > 1.0 || fabs(avgAccZ - 9.81) > 1.0) {
    Serial.println(F("Warning: Accelerometer readings out of expected range."));
  } else {
    Serial.println(F("Accelerometer readings are within expected range."));
  }

  if (fabs(avgGyroX) > 0.1 || fabs(avgGyroY) > 0.1 || fabs(avgGyroZ) > 0.1) {
    Serial.println(F("Warning: Gyroscope readings out of expected range."));
  } else {
    Serial.println(F("Gyroscope readings are within expected range."));
  }
  Serial.println(F("Self-test complete.\n"));
}

/**************************************************************************
  Save Calibration Data to EEPROM
**************************************************************************/
void saveCalibrationData() {
  Serial.println(F("\nSaving calibration data to EEPROM..."));
  uint32_t signature = EEPROM_CALIB_SIGNATURE;
  EEPROM.put(EEPROM_SIGNATURE_ADDR, signature);
  EEPROM.put(EEPROM_CALIB_ADDR, calibData);
  Serial.println(F("Calibration data saved.\n"));
}

/**************************************************************************
  Load Calibration Data from EEPROM
**************************************************************************/
void loadCalibrationData() {
  uint32_t signature;
  EEPROM.get(EEPROM_SIGNATURE_ADDR, signature);
  if (signature == EEPROM_CALIB_SIGNATURE) {
    EEPROM.get(EEPROM_CALIB_ADDR, calibData);
    calibrationLoaded = true;
    Serial.println(F("\nCalibration data loaded from EEPROM."));
    Serial.print(F("ACC Offsets: X="));
    Serial.print(calibData.accXOffset, 3);
    Serial.print(F(", Y="));
    Serial.print(calibData.accYOffset, 3);
    Serial.print(F(", Z="));
    Serial.println(calibData.accZOffset, 3);
    Serial.print(F("GYRO Offsets: X="));
    Serial.print(calibData.gyroXOffset, 5);
    Serial.print(F(", Y="));
    Serial.print(calibData.gyroYOffset, 5);
    Serial.print(F(", Z="));
    Serial.println(calibData.gyroZOffset, 5);
  } else {
    calibrationLoaded = false;
    Serial.println(F("\nNo valid calibration data found in EEPROM."));
  }
}

/**************************************************************************
  Reset Displacement and Velocity
**************************************************************************/
void resetDisplacement() {
  velocityX = velocityY = velocityZ = 0.0f;
  displacementX = displacementY = displacementZ = 0.0f;
  Serial.println(F("\nDisplacement and velocity reset.\n"));
}

/**************************************************************************
  MadgwickAHRSupdateIMU
**************************************************************************/
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3;
  float _4q0, _4q1, _4q2;
  float q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyro
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Normalize accelerometer measurement
  recipNorm = sqrt(ax * ax + ay * ay + az * az);
  if (recipNorm == 0.0f) return; // avoid division by zero
  recipNorm = 1.0f / recipNorm;
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  // Auxiliary variables
  _2q0 = 2.0f * q0;
  _2q1 = 2.0f * q1;
  _2q2 = 2.0f * q2;
  _2q3 = 2.0f * q3;
  _4q0 = 4.0f * q0;
  _4q1 = 4.0f * q1;
  _4q2 = 4.0f * q2;
  q0q0 = q0 * q0;
  q1q1 = q1 * q1;
  q2q2 = q2 * q2;
  q3q3 = q3 * q3;

  // Gradient descent algorithm corrective step
  float f1 = _2q1 * q3 - _2q0 * q2 - ax;
  float f2 = _2q0 * q1 + _2q2 * q3 - ay;
  float f3 = 1.0f - _2q1 * q1 - _2q2 * q2 - az;
  s0 = -_2q2 * f1 + _2q1 * f2;
  s1 =  _2q3 * f1 + _2q0 * f2 - 4.0f * q1 * f3;
  s2 = -_2q0 * f1 + _2q3 * f2 - 4.0f * q2 * f3;
  s3 =  _2q1 * f1 + _2q2 * f2;

  recipNorm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
  if (recipNorm == 0.0f) return;
  recipNorm = 1.0f / recipNorm;
  s0 *= recipNorm;
  s1 *= recipNorm;
  s2 *= recipNorm;
  s3 *= recipNorm;

  // Apply feedback step
  qDot1 -= beta * s0;
  qDot2 -= beta * s1;
  qDot3 -= beta * s2;
  qDot4 -= beta * s3;

  // Integrate rate of change of quaternion to yield new quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  // Normalize quaternion
  recipNorm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (recipNorm == 0.0f) return;
  recipNorm = 1.0f / recipNorm;
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
