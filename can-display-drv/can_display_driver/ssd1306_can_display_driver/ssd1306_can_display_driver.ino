/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited. 
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino_CAN.h>
#include <Arduino_LED_Matrix.h>
#include <SPI.h>
#include <Wire.h>

// Used for software SPI
#define OLED_CLK 13
#define OLED_MOSI 11

// Used for software or hardware SPI
#define OLED_CS 10
#define OLED_DC 8

#define OLED_RESET 4

#define CAN_ID_STATISTIC 0x30
#define CAN_ID_ROUNDTRIP_RX 0x50
#define CAN_ID_ROUNDTRIP_TX 0x51

// New CAN IDs for digital IO tokens
#define CAN_ID_FPM_ASIL_TOKEN 0x41
#define CAN_ID_LKA_ASIL_TOKEN 0x42
#define CAN_ID_LKA_QM_TOKEN 0x43

// Digital IO pins
#define PIN_FPM_ASIL 1 // D1
#define PIN_LKA_ASIL 2 // D2
#define PIN_LKA_QM 3   // D3

int is_display = 0; // Default to no display until we find one
int display_address = 0; // Will store the detected display address (0x3C or 0x3D)
int render_delay = 0;
unsigned long last_flash_time = 0;
bool flash_state = false;

ArduinoLEDMatrix matrix;
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// Token variables
volatile uint8_t FPM_ASIL_token = 0;
volatile uint8_t LKA_ASIL_token = 0;
volatile uint8_t LKA_QM_token = 0;

// Debounce variables
unsigned long last_debounce_time = 0;
const unsigned long DEBOUNCE_DELAY = 100; // 100ms debounce time

// Last reading values for debouncing
volatile uint8_t last_FPM_ASIL_state = 0;
volatile uint8_t last_LKA_ASIL_state = 0;
volatile uint8_t last_LKA_QM_state = 0;

unsigned long last_token_send_time = 0; // Timer for periodic token message sending
const unsigned long TOKEN_SEND_INTERVAL = 250; // 750ms interval
volatile int8_t token_send_counter= 0; 
// Interrupt service routines with debouncing
void ISR_FPM_ASIL() {
    uint8_t reading = digitalRead(PIN_FPM_ASIL);
    if (reading != last_FPM_ASIL_state) {
        last_debounce_time = millis();
    }
    last_FPM_ASIL_state = reading;
}

void ISR_LKA_ASIL() {
    uint8_t reading = digitalRead(PIN_LKA_ASIL);
    if (reading != last_LKA_ASIL_state) {
        last_debounce_time = millis();
    }
    last_LKA_ASIL_state = reading;
}

void ISR_LKA_QM() {
    uint8_t reading = digitalRead(PIN_LKA_QM);
    if (reading != last_LKA_QM_state) {
        last_debounce_time = millis();
    }
    last_LKA_QM_state = reading;
}

void setup() {
    Serial.begin(115200);

    matrix.begin();    // Configure digital input pins with pull-up resistors
    pinMode(PIN_FPM_ASIL, INPUT_PULLUP);
    pinMode(PIN_LKA_ASIL, INPUT_PULLUP);
    pinMode(PIN_LKA_QM, INPUT_PULLUP);
    
    // Initialize token values based on current pin states
    FPM_ASIL_token = digitalRead(PIN_FPM_ASIL);
    LKA_ASIL_token = digitalRead(PIN_LKA_ASIL);
    LKA_QM_token = digitalRead(PIN_LKA_QM);
    
    // Initialize debounce variables
    last_FPM_ASIL_state = FPM_ASIL_token;
    last_LKA_ASIL_state = LKA_ASIL_token;
    last_LKA_QM_state = LKA_QM_token;

    // Attach interrupts for pin change detection
    attachInterrupt(digitalPinToInterrupt(PIN_FPM_ASIL), ISR_FPM_ASIL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_LKA_ASIL), ISR_LKA_ASIL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_LKA_QM), ISR_LKA_QM, CHANGE);

    // Set filter mask and ID to allow only messages with ID 0x20
    // CAN.setFilterMask_Standard(0x1FFC0000);  // Mask for standard 11-bit IDs
    Serial.println("CAN.begin(...) ");
    if (!CAN.begin(CanBitRate::BR_1000k)) {
        Serial.println("CAN.begin(...) failed.");
        delay(100);
    }
    
    // Initialize I2C
    Wire.begin();
    
    // Check specifically for OLED display at the two common addresses: 0x3C and 0x3D
    Serial.println("Checking for OLED display at addresses 0x3C and 0x3D only...");
    
    // Check for display at address 0x3C (common for 0.96" OLED)
    if (scanI2CAddress(0x3C)) {
        display_address = 0x3C;
        Serial.println("Found OLED display at address 0x3C");
    } 
    // Check for display at address 0x3D (common for 1.15" OLED)
    else if (scanI2CAddress(0x3D)) {
        display_address = 0x3D;
        Serial.println("Found OLED display at address 0x3D");
    } 
    else {
        Serial.println("No OLED display found at addresses 0x3C or 0x3D");
        display_address = 0;
    }
      // Initialize the display if one was found
    if (display_address != 0) {
        Serial.print("Initializing OLED display at address 0x");
        Serial.println(display_address, HEX);
        
        if (display.begin(SSD1306_SWITCHCAPVCC, display_address)) {
            is_display = 1;
            display.display(); // show splashscreen
            delay(1000);
            display.clearDisplay();
            
            // Display initialization message            display.setTextSize(1);
            display.setTextColor(WHITE);
            display.setCursor(0, 0);
            display.println("Display initialized");
            display.print("Address: 0x");
            display.println(display_address, HEX);
            display.println("");
            display.println("CAN Monitor Starting...");
            display.display();
            
            Serial.println("OLED display initialized successfully");
        } else {
            Serial.println("OLED display found but failed to initialize");
            is_display = 0;
        }
    } else {
        Serial.println("No OLED display detected");
        is_display = 0;
    }
      delay(1000); // Give time to read the display
    
    if (is_display) {
        display.clearDisplay();
    }
    
    Serial.println("Digital IO CAN Monitor initialized");
    Serial.print("Initial states - FPM_ASIL: ");
    Serial.print(FPM_ASIL_token);
    Serial.print(", LKA_ASIL: ");
    Serial.print(LKA_ASIL_token);
    Serial.print(", LKA_QM: ");
    Serial.println(LKA_QM_token);
    
    // Initialize LED matrix with token states
    updateTokenLEDs();
}

#define ROWS 8
#define COLS 12

uint8_t frame[ROWS][COLS] = {0};

// Function to check if an I2C device exists at a specific address
bool scanI2CAddress(byte address) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        return true; // Device found at this address
    }
    return false; // No device at this address
}

// Function to send CAN message with token value
void sendTokenMessage(uint16_t can_id, uint8_t token_value) {
    uint8_t data[1] = {token_value};
    CanMsg const msg_tx(CanStandardId(can_id), sizeof(data), data);

    if (int const rc = CAN.write(msg_tx); rc < 0) {
        Serial.print("CAN.write(...) failed with error_code ");
        Serial.println(rc);
    }

}

// Function to update the LEDs for token states
void updateTokenLEDs() {
    // Clear the first three LEDs in the first row
    frame[0][0] = 0;
    frame[0][1] = 0;
    frame[0][2] = 0;

    // Set LEDs based on token states
    // FPM_ASIL_token -> LED at (0,0) (first row, first LED)
    frame[0][0] = FPM_ASIL_token;

    // LKA_ASIL_token -> LED at (0,1) (first row, second LED)
    frame[0][1] = LKA_ASIL_token;

    // LKA_QM_token -> LED at (0,2) (first row, third LED)
    frame[0][2] = LKA_QM_token;
    
    // Render the LED matrix whenever token states change
    matrix.renderBitmap(frame, ROWS, COLS);
}

// Function to display latency statistics on OLED
void displayLatencyStats(uint16_t total_latency, uint16_t can_latency, 
                        uint16_t eth_latency, uint16_t os_jitter) {
    if (is_display) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("Measured Latencies");
        display.print("Roundtrip: ");
        display.print(total_latency);
        display.println(" us");
        display.print("CAN:       ");
        display.print(can_latency);
        display.println(" us");
        display.print("Eth:       ");
        display.print(eth_latency);
        display.println(" us");
        display.println("");
        display.print("OS Jitter: ");
        display.print(os_jitter);
        display.println(" us");
        display.display();
    }
}

// Function to display CAN error message on OLED
void displayCanError() {
    if (is_display) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("CAN BUS ERROR");
        display.println("");
        display.println("No CAN communication");
        display.println("Check connections");
        display.println("");
        display.println("Status: Waiting...");
        display.display();
    }
}

// Global variables for latency statistics
uint16_t total_latency = 0;
uint16_t can_latency = 0;
uint16_t eth_latency = 0;
uint16_t os_jitter = 0;

// Timeout tracking for roundtrip messages
unsigned long last_roundtrip_time = 0;
const unsigned long ROUNDTRIP_TIMEOUT = 5000; // 5 seconds in milliseconds

// CAN availability timeout variables
unsigned long last_can_available_time = 0;
const unsigned long CAN_AVAILABILITY_TIMEOUT = 1000; // 1 second timeout before considering CAN unavailable

// Flag to indicate new telemetry data is available
volatile bool update_telemetry_flag = false;
unsigned long last_telemetry_update = 0; // Timestamp of last telemetry update

void loop() {
    static bool was_can_available = false;
    static bool can_status = false;
    
    // Debounce logic for input pins
    unsigned long current_time = millis();
      // Check CAN availability status (performed once per loop)
    bool is_can_available = CAN.available();
    
    // Update the CAN status with debouncing logic
    if (is_can_available) {
        last_can_available_time = current_time;
        can_status = true; 
    } else if (current_time - last_can_available_time > CAN_AVAILABILITY_TIMEOUT) {
        // Only mark CAN as unavailable if it's been unavailable for the timeout period
        can_status = false;
    }

    if (can_status) {
        // If CAN just became available after being unavailable, update LEDs to show current state
        if (!was_can_available) {
            was_can_available = true;
            updateTokenLEDs();
            Serial.println("CAN connection established");        }
        
        // Process CAN messages if available (reuse the availability check from earlier)
        bool skip_token_send_this_cycle = false;
        if (is_can_available) {
            CanMsg const msg = CAN.read();
            const uint8_t *data = msg.data;
            if (msg.id == CAN_ID_ROUNDTRIP_RX) {
                CanMsg const msg_tx(CanStandardId(CAN_ID_ROUNDTRIP_TX),
                                    sizeof(msg.data),
                                    msg.data); // Set CAN ID to 0x51
                                    
                /* Transmit the CAN message */
                if (int const rc = CAN.write(msg_tx); rc < 0) {
                    Serial.print("CAN.write(...) failed with error_code ");
                    Serial.println(rc);
                    // for (;;) {} // Halt on failure
                }
                Serial.print("."); //some delay to avoid flooding the CAN bus

                // Update last roundtrip message time
                last_roundtrip_time = current_time;
                
                // Check if new telemetry data is available
                if (update_telemetry_flag) {
                    displayLatencyStats(total_latency, can_latency, eth_latency, os_jitter);
                    update_telemetry_flag = false; // Reset the flag after displaying
                }
                skip_token_send_this_cycle = true;
            } else if (msg.id == CAN_ID_STATISTIC) { // Update global latency variables from CAN message
                total_latency = *((uint16_t *)&data[0]); // data[0-1] contains total latency
                can_latency = *((uint16_t *)&data[2]); // data[2-3] contains CAN latency
                eth_latency = *((uint16_t *)&data[4]); // data[4-5] contains Ethernet latency            
                os_jitter = *((uint16_t *)&data[6]);   // data[6-7] contains OS jitter            
                
                // Set flag to indicate new telemetry data is available
                update_telemetry_flag = true;
                last_telemetry_update = current_time;

                if (current_time - last_roundtrip_time > ROUNDTRIP_TIMEOUT) {
                    // It's been more than 5 seconds since the last roundtrip message
                    // Display the current latency values
                    displayLatencyStats(total_latency, can_latency, eth_latency, os_jitter);
                }
                skip_token_send_this_cycle = true;
            }
        }
        // Periodically send token messages every 750ms, but skip if a relevant CAN message was processed
        if ((current_time - last_token_send_time) > TOKEN_SEND_INTERVAL) {
            if (token_send_counter == 0) {
                sendTokenMessage(CAN_ID_FPM_ASIL_TOKEN, last_FPM_ASIL_state);
            }

            if (token_send_counter == 1) {
                sendTokenMessage(CAN_ID_LKA_QM_TOKEN, last_LKA_QM_state);
                updateTokenLEDs();
            }

            if (token_send_counter == 2) {
                sendTokenMessage(CAN_ID_LKA_ASIL_TOKEN, last_LKA_ASIL_state) ;
                updateTokenLEDs();
                token_send_counter = -1; // Reset counter after sending all tokens
            }

            token_send_counter++;
            
            last_token_send_time = current_time;
            Serial.print("o"); //some delay to avoid flooding the CAN bus

        }

    } else {
        // If CAN just became unavailable after being available, log it
        if (was_can_available) {
            was_can_available = false;
            Serial.println("CAN connection lost");
        }
        
        // If CAN is not available, flash the LEDs with a 300ms period
        // to indicate no CAN communication
        if (current_time - last_flash_time >= 300) {
            last_flash_time = current_time;
            flash_state = !flash_state;
            
            // Clear the first three LEDs in the first row
            frame[0][0] = 0;
            frame[0][1] = 0;
            frame[0][2] = 0;
            
            if (flash_state) {
                // Set LEDs on
                frame[0][0] = 1;
                frame[0][1] = 1;
                frame[0][2] = 1;
            }
            
            // Render the LED matrix
            matrix.renderBitmap(frame, ROWS, COLS);
            
            // Display CAN unavailable message on OLED
            displayCanError();
        }
    }
}
