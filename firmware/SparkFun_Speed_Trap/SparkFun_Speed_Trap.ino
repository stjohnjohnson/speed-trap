/*
 Displaying instantaneous speed from a LIDAR on two large 7-segment displays
 By: Nathan Seidle
 SparkFun Electronics
 Date: January 5th, 2015
 License: This code is public domain but you buy me a beer if you use this and
 we meet someday (Beerware license).

 Revisions:
 - St. John Johnson <st.john.johnson@gmail.com>
*/

#include <Wire.h>     // Used for I2C
#include <avr/wdt.h>  // We need watch dog for this program

#define LIDARLite_ADDRESS 0x62  // Default I2C Address of LIDAR-Lite.
#define RegisterMeasure 0x00    // Register to write to initiate ranging.
#define MeasureValue 0x04       // Value to initiate ranging.
#define RegisterHighLowB 0x8F   // Register to get High and Low bytes in 1 call.

// GPIO declarations
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

byte statLED = 13;    // On board status LED
byte en_LIDAR = A0;   // Low makes LIDAR go to sleep, high is normal operation

byte segmentLatch = 5;   // Display data when this pin goes high
byte segmentClock = 6;   // Clock one bit on each rising/falling edge
byte segmentSerial = 7;  // Serial data in

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Background Noise
int backgroundDistance = 0;

// Number of times we've zeroed out
int zeroedCount = 3;

// Keep track of N number of time/distance
const byte stackSize = 5;
int stackDistance[stackSize];
unsigned long stackTime[stackSize];
int stackIndex = 0;

// Current Speed
int currentSpeed = 0;
int minSpeed = 999;
int maxSpeed = 0;
int totalSpeed = 0;
int totalReadings = 0;

// Refresh trackers
unsigned long lastBlink = 0;
unsigned long lastSample = 0;
unsigned long lastDisplay = 0;
unsigned long lastClear = 0;

// Number of milliseconds between going back to 0
#define HOLDTIME 2000

// Number of milliseconds between screen updates
#define REFRESHRATE 200

// Number of milliseconds between speed measurements
#define SAMPLERATE 50

// This is the min speed before displaying
#define MINDISPLAYSPEED 1

// Verbosity with logs
#define VERBOSE false

void setup() {
  wdt_reset();    // Pet the dog
  wdt_disable();  // We don't want the watchdog during init

  Serial.begin(115200);
  Serial.println("Speed Trap");

  Wire.begin();

  pinMode(en_LIDAR, OUTPUT);

  pinMode(segmentClock, OUTPUT);
  pinMode(segmentLatch, OUTPUT);
  pinMode(segmentSerial, OUTPUT);

  digitalWrite(segmentClock, LOW);
  digitalWrite(segmentLatch, LOW);
  digitalWrite(segmentSerial, LOW);

  pinMode(statLED, OUTPUT);

  Serial.println("Coming online");

  showSpeed(42);  // Test pattern

  enableLIDAR();
  while (readLIDAR() == 0) {
    Serial.println("Failed LIDAR read");
    delay(100);
  }

  // Generate test pattern and identify background distance
  for (int i = 99; i >= 0; i -= 11) {
    backgroundDistance += readLIDAR();
    showSpeed(i);
    delay(100);
  }

  // Add some fuzzyness to the background
  backgroundDistance = abs(ceil(((float)backgroundDistance / 10.0) * 0.9));
  Serial.print("Background Distance: ");
  Serial.print(backgroundDistance);
  Serial.print("cm (");
  Serial.print(cmToFt(backgroundDistance), 2);
  Serial.println("ft)");

  wdt_reset();             // Pet the dog
  wdt_enable(WDTO_250MS);  // Unleash the beast
}

void loop() {
  wdt_reset();  // Pet the dog

  unsigned long now = millis();

  // Each second blink the status LED
  if (now - lastBlink >= 1000) {
    lastBlink = now;

    if (digitalRead(statLED) == LOW)
      digitalWrite(statLED, HIGH);
    else
      digitalWrite(statLED, LOW);
  }

  // Take a reading every SAMPLERATE ms
  if (now - lastSample >= SAMPLERATE) {
    lastSample = now;

    // Every loop let's get a reading
    int distance = readLIDAR();  // Go get distance in cm

    // During a normal run
    // 1. If we are zeroed out (min/max) for >150ms, reset the stack (record
    // car complete)
    // 2. If we are zeroed out (min/max), do not add to the stack
    // 3. Add the distance + time to the stack (increment index)
    // 4. Find the min/max of distance and time to identify speed.
    // 5. Record car.
    if (isZeroedOut(distance)) {
      zeroedCount++;
      if (zeroedCount == 3) {
        Serial.print("Car Recorded:");
        Serial.print(" min/");
        Serial.print(minSpeed);
        Serial.print(" max/");
        Serial.print(maxSpeed);
        Serial.print(" avg/");
        Serial.print(ceil(totalSpeed / totalReadings));
        Serial.print(" last/");
        Serial.print(currentSpeed);
        Serial.print(" time/");
        Serial.print(totalReadings * SAMPLERATE);
        Serial.println();

        // Clear stack
        memset(stackDistance, 0, stackSize);
        memset(stackTime, 0UL, stackSize);

        // Zero speed
        currentSpeed = 0;
        minSpeed = 999;
        maxSpeed = 0;
        totalSpeed = 0;
        totalReadings = 0;
        lastClear = now;
      }
      return;
    } else {
      zeroedCount = 0;
    }

    // Add distance + time to stack
    stackDistance[stackIndex] = distance;
    stackTime[stackIndex] = millis();
    stackIndex++;
    if (stackIndex >= stackSize) stackIndex = 0;  // Wrap this variable

    // Identify min/max
    int mph = getMPH();

    // Display speed
    currentSpeed = mph;
    minSpeed = min(mph, minSpeed);
    maxSpeed = max(mph, maxSpeed);
    totalSpeed += mph;
    totalReadings++;

    if (VERBOSE) {
      Serial.print("distance: ");
      Serial.print(distance);
      Serial.print("cm (");
      Serial.print(cmToFt(distance), 2);
      Serial.print("ft)");

      Serial.print("\t| speed: ");
      Serial.print(mph);
      Serial.println(" mph");
    }
  }

  // Update display every REFRESHRATE ms
  if (now - lastDisplay >= REFRESHRATE) {
    lastDisplay = now;
    if (currentSpeed != 0) {
      showSpeed(currentSpeed);
    } else {
      if (now - lastClear >= HOLDTIME) {
        lastClear = now;
        showSpeed(0);
      }
    }
  }
}

// Return the earliest recorded time
unsigned long getEarliestTime() {
  // Iterate through the stack going forward from index
  for (byte x = stackIndex; x < stackSize + stackIndex; x++) {
    byte y = x % stackSize;
    if (stackTime[y] > 0UL) {
      return stackTime[y];
    }
  }
  return millis();
}

// Return the latest recorded time
unsigned long getLatestTime() {
  if (stackIndex == 0) {
    return stackTime[stackSize - 1];
  } else {
    return stackTime[stackIndex - 1];
  }
}

// Return the current Miles Per Hour
int getMPH() {
  int minDistance = backgroundDistance, maxDistance = 0;
  int distance;
  unsigned long elapsedTime = getLatestTime() - getEarliestTime();

  for (byte x = 0; x < stackSize; x++) {
    distance = stackDistance[x];
    // Only read valid values
    if (distance > 1 && distance < backgroundDistance) {
      // Check distance
      minDistance = min(distance, minDistance);
      maxDistance = max(distance, maxDistance);
    }
  }

  // 22.3694 is conversion of cm per millisecond to miles per hour
  float deltaDistance = (float)(maxDistance - minDistance),
        loopTime = (float)elapsedTime, mph = deltaDistance / loopTime * 22.3694;

  if (VERBOSE) {
    Serial.println("\nDebug Data Dump");
    Serial.print("min/");
    Serial.print(minDistance);
    Serial.print(" max/");
    Serial.print(maxDistance);
    Serial.print(" delta/");
    Serial.print(deltaDistance, 2);
    Serial.print(" loopTime/");
    Serial.print(loopTime, 2);
    Serial.print(" mph/");
    Serial.print(mph, 2);
    Serial.println();
    Serial.print("previousTime/");
    Serial.print(getLatestTime());
    Serial.print(" oldestTime/");
    Serial.println(getEarliestTime());

    for (byte x = 0; x < stackSize; x++) {
      Serial.print("d[");
      Serial.print((int)x);
      Serial.print("] == ");
      Serial.println(stackDistance[x]);
    }
    for (byte x = 0; x < stackSize; x++) {
      Serial.print("t[");
      Serial.print((int)x);
      Serial.print("] == ");
      Serial.println(stackTime[x]);
    }
    Serial.println("End Debug Data Dump");
  }

  // Calculate speed by determining cm per millisecond and converting to miles
  // per hour
  return ceil(mph);
}

float cmToFt(int cm) { return (float)cm * 0.0328084; }

// Are we reading the edge of infinity?
bool isZeroedOut(int distance) {
  if (distance == 1 || distance > backgroundDistance) {
    return true;
  }
  return false;
}

// A watch dog friendly delay
void petFriendlyDelay(int timeMS) {
  long current = millis();

  while (millis() - current < timeMS) {
    delay(1);
    wdt_reset();  // Pet the dog
  }
}

// Get a new reading from the distance sensor
int readLIDAR(void) {
  int distance = 0;

  Wire.beginTransmission((int)LIDARLite_ADDRESS);  // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure);  // sets register pointer to  (0x00)
  Wire.write((int)MeasureValue);     // sets register pointer to  (0x00)
  Wire.endTransmission();            // stop transmitting

  delay(20);    // Wait 20ms for transmit
  wdt_reset();  // Pet the dog

  Wire.beginTransmission((int)LIDARLite_ADDRESS);  // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB);  // sets register pointer to (0x8f)
  Wire.endTransmission();             // stop transmitting

  delay(20);    // Wait 20ms for transmit
  wdt_reset();  // Pet the dog

  Wire.requestFrom((int)LIDARLite_ADDRESS,
                   2);  // request 2 bytes from LIDAR-Lite

  if (Wire.available() >= 2)  // if two bytes were received
  {
    distance = Wire.read();  // receive high byte (overwrites previous reading)
    distance = distance << 8;  // shift high byte to be high 8 bits
    distance |= Wire.read();   // receive low byte as lower 8 bits
    return (distance);
  } else {
    Serial.println("Read fail");
    disableLIDAR();
    delay(100);
    enableLIDAR();

    return (0);
  }
}

// Takes a speed and displays 2 numbers. Displays absolute value (no
// negatives)
void showSpeed(float speed) {
  int absSpeed = abs(speed);
  int number = absSpeed;  // Remove negative signs and any decimals

  for (byte x = 0; x < 2; x++) {
    byte output;
    int remainder = number % 10;

    // todo Skip leading 0s?
    // todo Blink above a certain speed
    // Skip if the speed is too low
    if (absSpeed < MINDISPLAYSPEED) {
      output = ' ';
    } else {
      output = remainder;
    }

    postNumber(output, false);
    number /= 10;
  }

  // Latch the current segment data
  digitalWrite(segmentLatch, LOW);
  digitalWrite(
      segmentLatch,
      HIGH);  // Register moves storage register on the rising edge of RCK
}

// Given a number, or '-', shifts it out to the display
void postNumber(byte number, boolean decimal) {
  //    -  A
  //   / / F/B
  //    -  G
  //   / / E/C
  //    -. D/DP

#define a 1 << 0
#define b 1 << 6
#define c 1 << 5
#define d 1 << 4
#define e 1 << 3
#define f 1 << 1
#define g 1 << 2
#define dp 1 << 7

  byte segments;

  // This method uses 7946 bytes
  switch (number) {
    case 1:
      segments = b | c;
      break;
    case 2:
      segments = a | b | d | e | g;
      break;
    case 3:
      segments = a | b | c | d | g;
      break;
    case 4:
      segments = f | g | b | c;
      break;
    case 5:
      segments = a | f | g | c | d;
      break;
    case 6:
      segments = a | f | g | e | c | d;
      break;
    case 7:
      segments = a | b | c;
      break;
    case 8:
      segments = a | b | c | d | e | f | g;
      break;
    case 9:
      segments = a | b | c | d | f | g;
      break;
    case 0:
      segments = a | b | c | d | e | f;
      break;
    case ' ':
      segments = 0;
      break;
    case 'c':
      segments = g | e | d;
      break;
    case '-':
      segments = g;
      break;
  }

  if (decimal) segments |= dp;

  for (byte x = 0; x < 8; x++) {
    digitalWrite(segmentClock, LOW);
    digitalWrite(segmentSerial, segments & 1 << (7 - x));
    digitalWrite(
        segmentClock,
        HIGH);  // Data transfers to the register on the rising edge of SRCK
  }
}

// Sometimes the LIDAR stops responding. This causes it to reset
void disableLIDAR() { digitalWrite(en_LIDAR, LOW); }

void enableLIDAR() { digitalWrite(en_LIDAR, HIGH); }
