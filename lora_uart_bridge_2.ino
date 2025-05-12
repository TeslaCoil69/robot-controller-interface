#include <SPI.h>
#include <LoRa.h>
#include <AES.h>
#include <CBC.h>
#include <EEPROM.h>

// LoRa pins for Heltec V3
#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10
#define LORA_CS 8
#define LORA_RST 12
#define LORA_IRQ 14
#define USER_BUTTON 0

// LoRa settings
#define BAND 915E6  // Base frequency
#define CHANNEL_SPACING 200E3  // 200kHz spacing
#define NUM_CHANNELS 8  // Number of hopping channels
#define HOP_INTERVAL 1000  // Hop every 1 second
#define PACKET_SIZE 64  // Maximum packet size

// Encryption
#define KEY_SIZE 16  // 128-bit key
uint8_t key[KEY_SIZE] = {0x2B,0x7E,0x15,0x16,0x28,0xAE,0xD2,0xA6,0xAB,0xF7,0x15,0x88,0x09,0xCF,0x4F,0x3C};  // Example key
AES128 aes128;
CBC<AES128> cbc;

// Channel hopping
uint32_t channels[NUM_CHANNELS];
uint8_t currentChannel = 0;
unsigned long lastHopTime = 0;
bool isPaired = false;
uint32_t deviceId = 0;
uint32_t pairedDeviceId = 0;

// Buffer for UART data
uint8_t uartBuffer[PACKET_SIZE];
uint8_t uartIndex = 0;

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  Serial1.begin(115200);  // UART for external device
  
  // Initialize LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  
  // Initialize button
  pinMode(USER_BUTTON, INPUT_PULLUP);
  
  // Generate device ID if not already set
  if (EEPROM.read(0) == 0xFF) {
    deviceId = random(0xFFFFFFFF);
    EEPROM.put(0, deviceId);
  } else {
    EEPROM.get(0, deviceId);
  }
  
  // Initialize channels
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channels[i] = BAND + (i * CHANNEL_SPACING);
  }
  
  // Start LoRa
  if (!LoRa.begin(BAND)) {
    Serial.println("LoRa init failed");
    while (1);
  }
  
  // Set LoRa parameters
  LoRa.setTxPower(20);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSpreadingFactor(7);
  
  Serial.println("LoRa UART Bridge 2 Ready");
  Serial.print("Device ID: 0x");
  Serial.println(deviceId, HEX);
}

void loop() {
  // Check for pairing button press
  if (digitalRead(USER_BUTTON) == LOW) {
    startPairing();
  }
  
  // Handle frequency hopping
  if (millis() - lastHopTime >= HOP_INTERVAL) {
    hopChannel();
  }
  
  // Read from UART
  while (Serial1.available() && uartIndex < PACKET_SIZE) {
    uartBuffer[uartIndex++] = Serial1.read();
  }
  
  // If we have data to send
  if (uartIndex > 0) {
    sendData();
  }
  
  // Check for incoming LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    receiveData();
  }
}

void startPairing() {
  Serial.println("Starting pairing...");
  isPaired = false;
  pairedDeviceId = 0;
  
  // Wait for pairing request
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {  // 5 second timeout
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      if (LoRa.read() == 0xAA) {  // Pairing request
        LoRa.readBytes((uint8_t*)&pairedDeviceId, 4);
        
        // Send pairing response
        LoRa.beginPacket();
        LoRa.write(0xBB);  // Pairing response marker
        LoRa.write((uint8_t*)&deviceId, 4);
        LoRa.endPacket();
        
        Serial.print("Paired with device: 0x");
        Serial.println(pairedDeviceId, HEX);
        isPaired = true;
        break;
      }
    }
  }
  
  if (!isPaired) {
    Serial.println("Pairing failed");
  }
}

void hopChannel() {
  currentChannel = (currentChannel + 1) % NUM_CHANNELS;
  LoRa.setFrequency(channels[currentChannel]);
  lastHopTime = millis();
}

void sendData() {
  if (!isPaired) return;
  
  // Encrypt data
  uint8_t encrypted[PACKET_SIZE];
  memcpy(encrypted, uartBuffer, uartIndex);
  cbc.encrypt(encrypted, encrypted, uartIndex);
  
  // Send packet
  LoRa.beginPacket();
  LoRa.write(0xCC);  // Data marker
  LoRa.write((uint8_t*)&deviceId, 4);
  LoRa.write(uartIndex);
  LoRa.write(encrypted, uartIndex);
  LoRa.endPacket();
  
  uartIndex = 0;  // Reset buffer
}

void receiveData() {
  if (!isPaired) return;
  
  uint8_t marker = LoRa.read();
  if (marker == 0xCC) {  // Data packet
    uint32_t senderId;
    LoRa.readBytes((uint8_t*)&senderId, 4);
    
    // Only process packets from paired device
    if (senderId != pairedDeviceId) return;
    
    uint8_t dataSize = LoRa.read();
    uint8_t encrypted[PACKET_SIZE];
    LoRa.readBytes(encrypted, dataSize);
    
    // Decrypt data
    uint8_t decrypted[PACKET_SIZE];
    memcpy(decrypted, encrypted, dataSize);
    cbc.decrypt(decrypted, decrypted, dataSize);
    
    // Send to UART
    Serial1.write(decrypted, dataSize);
  }
} 