#include <Arduino.h>
#include <Wire.h>

// --- Configuration ---
const int PSU_I2C_ADDR = 0x5B;  // Your detected address
const int PIN_SDA      = 8;
const int PIN_SCL      = 9;
const uint32_t I2C_SPEED = 10000; // 10kHz as requested (Standard is 100kHz)

// --- PMBus Command Codes (Standard PMBus 1.3) ---
// Configuration
const byte CMD_VOUT_MODE        = 0x20; // VOUT mode and exponent

// Status Commands
const byte CMD_STATUS_WORD      = 0x79; // Status Word (2 bytes)
const byte CMD_STATUS_VOUT      = 0x7A; // Output Voltage Status
const byte CMD_STATUS_IOUT      = 0x7B; // Output Current Status
const byte CMD_STATUS_INPUT     = 0x7C; // Input Status
const byte CMD_STATUS_TEMP      = 0x7D; // Temperature Status
const byte CMD_STATUS_CML       = 0x7E; // Communications/Logic Status
const byte CMD_STATUS_MFR       = 0x80; // Manufacturer Specific Status

// Voltage/Current/Power Readings
const byte CMD_READ_VIN         = 0x88; // Input Voltage (Volts) - Linear11
const byte CMD_READ_IIN         = 0x89; // Input Current (Amps) - Linear11
const byte CMD_READ_VOUT        = 0x8B; // Output Voltage (Volts) - Linear16!
const byte CMD_READ_IOUT        = 0x8C; // Output Current (Amps) - Linear11
const byte CMD_READ_TEMP_1      = 0x8D; // Temperature 1 (°C) - Linear11
const byte CMD_READ_TEMP_2      = 0x8E; // Temperature 2 (°C) - Linear11
const byte CMD_READ_FAN_SPEED_1 = 0x90; // Fan Speed 1 (RPM) - Linear11
const byte CMD_READ_FAN_SPEED_2 = 0x91; // Fan Speed 2 (RPM) - Linear11
const byte CMD_READ_POUT        = 0x96; // Output Power (Watts) - Linear11
const byte CMD_READ_PIN         = 0x97; // Input Power (Watts) - Linear11

// Manufacturer Information
const byte CMD_MFR_ID           = 0x99; // Manufacturer ID
const byte CMD_MFR_MODEL        = 0x9A; // Model Name String
const byte CMD_MFR_REVISION     = 0x9B; // Revision String
const byte CMD_MFR_LOCATION     = 0x9C; // Location String
const byte CMD_MFR_DATE         = 0x9D; // Manufacture Date
const byte CMD_MFR_SERIAL       = 0x9E; // Serial Number

// Global: VOUT exponent (read from VOUT_MODE at startup)
int8_t vout_exponent = -9; // Default for 12V PSUs

// --- Helper: Decode Linear11 Format (Standard PMBus Float) ---
// Used for: VIN, IIN, IOUT, TEMP, FAN, PIN, POUT
// Value = Mantissa * 2^Exponent
float pmbusLinear11ToFloat(uint16_t input_val) {
  // Extract Exponent (Top 5 bits)
  int16_t exponent = input_val >> 11;
  // Sign extend exponent if negative (5-bit two's complement)
  if (exponent > 15) exponent |= 0xFFE0;

  // Extract Mantissa (Bottom 11 bits)
  int16_t mantissa = input_val & 0x07FF;
  // Sign extend mantissa if negative (11-bit two's complement)
  if (mantissa > 1023) mantissa |= 0xF800;

  return mantissa * pow(2, exponent);
}

// --- Helper: Decode Linear16 Format (VOUT only!) ---
// Used ONLY for: VOUT (requires VOUT_MODE exponent read at startup)
// Value = Unsigned_Integer * 2^Exponent
float pmbusLinear16ToFloat(uint16_t input_val) {
  return input_val * pow(2.0, vout_exponent);
}

// --- Helper: Read a 2-byte Word (Sensor Data) ---
float readPMBusSensor(byte cmd, bool isLinear16 = false) {
  delay(5); // Small delay before command
  
  Wire.beginTransmission(PSU_I2C_ADDR);
  Wire.write(cmd);
  if (Wire.endTransmission(false) != 0) {
    return -999.0; // Error indicator
  }

  delay(1); // Wait for PSU to prepare data
  
  // Request 2 bytes (Low Byte, High Byte)
  if (Wire.requestFrom(PSU_I2C_ADDR, 2) == 2) {
    byte low = Wire.read();
    byte high = Wire.read();
    uint16_t raw = (high << 8) | low;
    
    if (isLinear16) {
      return pmbusLinear16ToFloat(raw);
    } else {
      return pmbusLinear11ToFloat(raw);
    }
  }
  return -999.0; // Error indicator
}

// --- Helper: Read Status Word (2 bytes) ---
uint16_t readPMBusStatus(byte cmd) {
  delay(1);
  
  Wire.beginTransmission(PSU_I2C_ADDR);
  Wire.write(cmd);
  if (Wire.endTransmission(false) != 0) {
    return 0xFFFF;
  }

  delay(1);
  
  if (Wire.requestFrom(PSU_I2C_ADDR, 2) == 2) {
    byte low = Wire.read();
    byte high = Wire.read();
    return (high << 8) | low;
  }
  return 0xFFFF;
}

// --- Helper: Read Status Byte (1 byte) ---
uint8_t readPMBusStatusByte(byte cmd) {
  delay(5);
  
  Wire.beginTransmission(PSU_I2C_ADDR);
  Wire.write(cmd);
  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }

  delay(1);
  
  if (Wire.requestFrom(PSU_I2C_ADDR, 1) == 1) {
    return Wire.read();
  }
  return 0xFF;
}

// --- Helper: Read a String (Block Read) ---
String readPMBusString(byte cmd) {
  Wire.beginTransmission(PSU_I2C_ADDR);
  Wire.write(cmd);
  if (Wire.endTransmission(false) != 0) {
    return "N/A";
  }

  // PMBus Block Read: First we read 1 byte to get the length
  if (Wire.requestFrom(PSU_I2C_ADDR, 1) != 1) {
    return "N/A";
  }
  
  int len = Wire.read();
  if (len <= 0 || len > 32) {
    return "N/A";
  }

  // Now read the actual string characters
  Wire.requestFrom(PSU_I2C_ADDR, len);
  char buffer[64];
  int i = 0;
  while (Wire.available() && i < (sizeof(buffer) - 1)) {
    buffer[i++] = (char)Wire.read();
  }
  buffer[i] = '\0'; // Null terminate
  return String(buffer);
}

void setup() {
  // Initialize Serial (USB CDC)
  Serial.begin(115200);
  delay(2000); // Wait for USB to catch up
  
  Serial.println("\n\n╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║       PMBus PSU Monitor (ESP32-C6) - DPS-450SB             ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝");
  Serial.printf("Config: SDA=%d, SCL=%d, Addr=0x%02X, Speed=%d Hz\n\n", 
                PIN_SDA, PIN_SCL, PSU_I2C_ADDR, I2C_SPEED);

  // Initialize I2C
  if (!Wire.begin(PIN_SDA, PIN_SCL, I2C_SPEED)) {
    Serial.println("!!! I2C Wire.begin failed !!!");
    while(1) delay(100);
  }
  
  // Wire.setTimeOut(200);

  Serial.println("Reading PSU Information...\n");
  
  // Read VOUT_MODE to get the exponent for Linear16 decoding
  Wire.beginTransmission(PSU_I2C_ADDR);
  Wire.write(CMD_VOUT_MODE);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(PSU_I2C_ADDR, 1) == 1) {
    byte raw_mode = Wire.read();
    vout_exponent = raw_mode & 0x1F; 
    if (vout_exponent > 15) vout_exponent |= 0xE0; // Sign extend
    Serial.printf("  VOUT Mode Exponent: %d\n\n", vout_exponent);
  } else {
    Serial.println("  Warning: Could not read VOUT_MODE. Using default -9.\n");
    vout_exponent = -9;
  }
  
  delay(50);
  
  // Read static info once
  String model = readPMBusString(CMD_MFR_MODEL);
  delay(10);
  String mfr_id = readPMBusString(CMD_MFR_ID);
  delay(10);
  String revision = readPMBusString(CMD_MFR_REVISION);
  delay(10);
  String serial = readPMBusString(CMD_MFR_SERIAL);
  delay(10);
  String date = readPMBusString(CMD_MFR_DATE);
  
  Serial.println("────────────── PSU INFORMATION ──────────────");
  Serial.printf("  Model:        %s\n", model.c_str());
  Serial.printf("  Manufacturer: %s\n", mfr_id.c_str());
  Serial.printf("  Revision:     %s\n", revision.c_str());
  Serial.printf("  Serial:       %s\n", serial.c_str());
  Serial.printf("  Date:         %s\n", date.c_str());
  Serial.println("──────────────────────────────────────────────\n");
  
  delay(1000);
  
  // Clear screen and prepare for live data
  Serial.println("\nStarting live monitoring (refreshing display)...\n");
  delay(500);
}

void loop() {
  // Read all sensor data (silently, errors won't show)
  float v_in = readPMBusSensor(CMD_READ_VIN, false);   // Linear11
  float i_in = readPMBusSensor(CMD_READ_IIN, false);   // Linear11
  float p_in = readPMBusSensor(CMD_READ_PIN, false);   // Linear11
  
  float v_out = readPMBusSensor(CMD_READ_VOUT, true);  // Linear16!
  float i_out = readPMBusSensor(CMD_READ_IOUT, false); // Linear11
  float p_out = readPMBusSensor(CMD_READ_POUT, false); // Linear11
  
  float temp1 = readPMBusSensor(CMD_READ_TEMP_1, false); // Linear11
  float temp2 = readPMBusSensor(CMD_READ_TEMP_2, false); // Linear11
  
  float fan1 = readPMBusSensor(CMD_READ_FAN_SPEED_1, false); // Linear11
  float fan2 = readPMBusSensor(CMD_READ_FAN_SPEED_2, false); // Linear11
  
  // Read status registers
  uint16_t status_word = readPMBusStatus(CMD_STATUS_WORD);
  uint8_t status_vout = readPMBusStatusByte(CMD_STATUS_VOUT);
  uint8_t status_iout = readPMBusStatusByte(CMD_STATUS_IOUT);
  uint8_t status_input = readPMBusStatusByte(CMD_STATUS_INPUT);
  uint8_t status_temp = readPMBusStatusByte(CMD_STATUS_TEMP);
  
  // Calculate efficiency
  float efficiency = 0.0;
  if (p_in > 0.1 && p_out > 0) {
    efficiency = (p_out / p_in) * 100.0;
  }
  
  // Build complete display string
  char buffer[1024];
  int pos = 0;
  
  pos += sprintf(buffer + pos, "\r"); // Carriage return to start of line
  pos += sprintf(buffer + pos, "PSU: DPS-450SB │ ");
  
  // INPUT
  pos += sprintf(buffer + pos, "IN: ");
  if (v_in >= 0 && v_in < 500) {
    pos += sprintf(buffer + pos, "%.1fV ", v_in);
  } else {
    pos += sprintf(buffer + pos, "N/A ");
  }
  
  if (i_in >= 0 && i_in < 100) {
    pos += sprintf(buffer + pos, "%.2fA ", i_in);
  } else {
    pos += sprintf(buffer + pos, "N/A ");
  }
  
  if (p_in >= 0 && p_in < 10000) {
    pos += sprintf(buffer + pos, "%.1fW │ ", p_in);
  } else {
    pos += sprintf(buffer + pos, "N/A │ ");
  }
  
  // OUTPUT
  pos += sprintf(buffer + pos, "OUT: ");
  if (v_out >= 0 && v_out < 100) {
    pos += sprintf(buffer + pos, "%.1fV ", v_out);
  } else {
    pos += sprintf(buffer + pos, "N/A ");
  }
  
  if (i_out >= 0 && i_out < 200) {
    pos += sprintf(buffer + pos, "%.2fA ", i_out);
  } else {
    pos += sprintf(buffer + pos, "N/A ");
  }
  
  if (p_out >= 0 && p_out < 10000) {
    pos += sprintf(buffer + pos, "%.1fW │ ", p_out);
  } else {
    pos += sprintf(buffer + pos, "N/A │ ");
  }
  
  // THERMAL
  if (temp1 >= -50 && temp1 < 200) {
    pos += sprintf(buffer + pos, "T1:%.0f°C ", temp1);
  } else {
    pos += sprintf(buffer + pos, "T1:N/A ");
  }
  
  if (temp2 >= -50 && temp2 < 200) {
    pos += sprintf(buffer + pos, "T2:%.0f°C │ ", temp2);
  } else {
    pos += sprintf(buffer + pos, "T2:N/A │ ");
  }
  
  // FAN
  if (fan1 >= 0 && fan1 < 50000) {
    pos += sprintf(buffer + pos, "Fan:%dRPM │ ", (int)fan1);
  } else {
    pos += sprintf(buffer + pos, "Fan:N/A │ ");
  }
  
  // EFFICIENCY
  if (efficiency > 0) {
    pos += sprintf(buffer + pos, "Eff:%.1f%% │ ", efficiency);
  } else {
    pos += sprintf(buffer + pos, "Eff:N/A │ ");
  }
  
  // STATUS
  pos += sprintf(buffer + pos, "Status:0x%04X", status_word);
  
  // Pad with spaces to clear any leftover characters from previous line
  for (int i = pos; i < 200; i++) {
    buffer[i] = ' ';
  }
  buffer[200] = '\0';
  
  // Print the complete line
  Serial.print(buffer);
  Serial.flush();
  
  delay(100); // Update every 2 seconds - slower for stability
}