/*
 * mmdc emulator
 * this program emulates a medallion mmdc by continually requesting water depth via rs485
 * 
 */


#include <HardwareSerial.h>
#include <math.h>

// RS485 serial and control pin definitions
#define RS485_RX_PIN 16   // Adjust as needed
#define RS485_TX_PIN 17   // Adjust as needed
#define RS485_DE_RE_PIN 21 // Adjust as needed (DE+RE tied together)

// Depth request and response parameters
const uint8_t DEPTH_REQUEST[] = {0x04, 0x09, 0x11, 0xE2};  // MMDC request
//const uint8_t DEPTH_REQUEST[] = {0x04, 0x09, 0x8B, 0x68};  // MMDC request
const size_t DEPTH_REQUEST_LEN = sizeof(DEPTH_REQUEST);
const size_t DEPTH_RESPONSE_LEN = 13;
const uint32_t BAUDRATE = 76800;
const unsigned long REQUEST_INTERVAL_MS = 1000;

HardwareSerial RS485Serial(2);

void enableTransmit() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
}

void enableReceive() {
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

// Calculate 8-bit two's complement checksum
uint8_t calculate_checksum(const uint8_t *data, uint8_t len) {
  uint8_t total = 0;
  for (uint8_t i = 0; i < len; i++) {
    total = (total + data[i]) & 0xFF;
  }
  return ((~total + 1) & 0xFF);
}

bool verifyChecksum(const uint8_t *response, size_t len) {
  if (len < 2) return false; // need at least 2 bytes to check
  uint8_t calculated = calculate_checksum(response, len - 1);
  uint8_t received = response[len - 1];
  return calculated == received;
}

// Extracts depth value from response (bytes 5=low [4], 6=high [5], in tenths of a foot)
float extractDepthFeet(const uint8_t* response) {
  uint16_t depth_encoded = ((uint16_t)response[5] << 8) | response[4];
  return depth_encoded / 10.0f;
}

void setup() {
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  enableReceive();

  Serial.begin(115200);
  RS485Serial.begin(BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  Serial.println("ESP32 RS485 Depth Requester started.");
  Serial.println("emulating rs485 water depth requests");
}

void loop() {
  // Send depth request
  
  enableTransmit();
  Serial.print("Sending water depth request (");
  Serial.print(DEPTH_REQUEST_LEN);
  Serial.print(" bytes): ");

  // Output each byte in hex format for debugging
  for (int i = 0; i < DEPTH_REQUEST_LEN; i++) {
    Serial.print("0x");
    if (DEPTH_REQUEST[i] < 0x10) Serial.print("0"); // Add leading zero for single digits
    Serial.print(DEPTH_REQUEST[i], HEX);
    if (i < DEPTH_REQUEST_LEN - 1) Serial.print(" ");
  }
  Serial.println();
  

  delayMicroseconds(200); // Allow line to settle
  RS485Serial.write(DEPTH_REQUEST, DEPTH_REQUEST_LEN);
  RS485Serial.flush();
  delayMicroseconds(500); // Wait for transmission to complete
  
  enableReceive();

  // Try to read the response
  uint8_t response[DEPTH_RESPONSE_LEN];
  size_t bytesRead = 0;
  unsigned long startTime = millis();
  while (bytesRead < DEPTH_RESPONSE_LEN && (millis() - startTime) < 500) { // 500ms timeout
    if (RS485Serial.available()) {
      response[bytesRead++] = RS485Serial.read();
    }
  }

  if (bytesRead == DEPTH_RESPONSE_LEN) {
    if (!verifyChecksum(response, DEPTH_RESPONSE_LEN)) {
      Serial.println("Checksum failed, ignoring response.");
      delay(REQUEST_INTERVAL_MS);
      return;
    }

    Serial.print("Received response: ");
    for (size_t i = 0; i < DEPTH_RESPONSE_LEN; i++) {
      if (response[i] < 0x10) Serial.print("0");
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Extract and display depth value in feet (from bytes 5 and 6, low/high)
    float depth_feet = extractDepthFeet(response);
    Serial.print("Extracted Depth: ");
    if (depth_feet < 100.0) {
      Serial.print(depth_feet, 1); // one decimal place
    } else {
      Serial.print((int)round(depth_feet)); // whole number, rounded
    }
    Serial.println(" ft");

  } else {
    Serial.print("No response received (timeout), got ");
    Serial.print(bytesRead);
    Serial.println(" bytes.");
  }

  delay(REQUEST_INTERVAL_MS);
}