#include <Arduino.h>
#include <Servo.h>

// --- Config ---
#define SERVO_PIN D7
// Serial2: Lidar Connection (PA3 = RX, PA2 = TX)
// Note: On Nucleo, 'Serial' is USB. 'Serial2' is USART2 (PA2/PA3).
// User said: "TX pin from lidar is connected to the RX pin on stm"
#define LIDAR_BAUD 230400
#define USB_BAUD 230400
// Instantiate Serial for Lidar on PA10 (RX) and PA9 (TX)
HardwareSerial LidarSerial(PA10, PA9);

Servo myServo;
int currentAngle = 90;
int sweepDir = 1;
unsigned long lastServoMoveTime = 0;
const int servoDelay = 15; // Speed of sweep

// LD19 Packet Structure
// Header: 0x54
// VerLen: 1 byte
// Speed: 2 bytes
// Start Angle: 2 bytes
// Data: 12 points * 3 bytes (dist, confidence)
// End Angle: 2 bytes
// Timestamp: 2 bytes
// CRC: 1 byte
// Total: 47 bytes

#define PACKET_SIZE 47
uint8_t rxBuffer[PACKET_SIZE];
int bufferIndex = 0;

void setup() {
  // Serial: USB Debugging (Send data to PC)
  Serial.begin(USB_BAUD);
  
  delay(100);

  // Serial1: Lidar Connection (PA10 = RX, PA9 = TX)
  // Connect Lidar TX to Pin D2 (PA10) on the Nucleo
  // Connect Lidar RX to Pin D8 (PA9) on the Nucleo
  LidarSerial.begin(LIDAR_BAUD);

  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  delay(100);
  

}

// Simple CRC8 Check (LD19 usually uses Checksum, sum of bytes)
// LD19 Manual: "CRC is the sum of all bytes from Header to Timestamp"
uint8_t CalcCRC(uint8_t *p, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = crc + p[i]; // Simple 8-bit sum
  }
  return crc;
}

void processPacket() {
  // Packet Format:
  // Byte 0: 0x54
  // Byte 1: VerLen (0x2C usually, fixed 12 points)
  // Byte 2-3: Radar Speed
  // Byte 4-5: Start Angle (unit: 0.01 deg)
  // Byte 6-41: 12 x 3 bytes (2 byte dist, 1 byte intensity)
  // Byte 42-43: End Angle
  // Byte 44-45: Timestamp
  // Byte 46: CRC (Sum of 0-45)

  uint8_t crc = 0;
  for (int i = 0; i < 46; i++)
    crc += rxBuffer[i];

  if (crc != rxBuffer[46]) {
    // CRC Fail
    return;
  }

  uint16_t startAngle = rxBuffer[4] | (rxBuffer[5] << 8);
  uint16_t endAngle = rxBuffer[42] | (rxBuffer[43] << 8);

  // Calculate step per point (interpolated)
  float step = 0;
  if (endAngle >= startAngle) {
    step = (float)(endAngle - startAngle) / 11.0;
  } else {
    step = (float)((endAngle + 36000) - startAngle) / 11.0;
  }

  // Print Points
  // Format: P <servo> <dist> <intensity> <angle>
  // Output everything so PC can handle Trig.

  for (int i = 0; i < 12; i++) {
    int base = 6 + (i * 3);
    uint16_t dist = rxBuffer[base] | (rxBuffer[base + 1] << 8);
    uint8_t conf = rxBuffer[base + 2];

    if (dist > 0) { // Valid point
      float angle = (startAngle + step * i) / 100.0;
      if (angle >= 360.0)
        angle -= 360.0;

      Serial.print("P ");
      Serial.print(currentAngle);
      Serial.print(" ");
      Serial.print(dist);
      Serial.print(" ");
      Serial.print(conf);
      Serial.print(" ");
      Serial.println(angle, 2);
    }
  }
}

void loop() {
  // Servo Sweep
  if (millis() - lastServoMoveTime > servoDelay) {
    lastServoMoveTime = millis();
    currentAngle += sweepDir;
    if (currentAngle >= 180 || currentAngle <= 0) {
      sweepDir = -sweepDir;
    }
    myServo.write(currentAngle);
  }

  // Lidar Parsing
  // Read from Serial2 (PA3 = RX, PA2 = TX)
  while (Serial2.available()) {
    uint8_t c = Serial2.read();

    // State machine to find header 0x54
    if (bufferIndex == 0) {
      if (c == 0x54) {
        rxBuffer[bufferIndex++] = c;
      }
    } else {
      rxBuffer[bufferIndex++] = c;
      if (bufferIndex >= PACKET_SIZE) {
        processPacket();
        bufferIndex = 0;
      }
    }
  }
}