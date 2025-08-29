/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino_CAN.h>
#include <SimpleFOC.h>
#include <math.h>

// Comment out to reduce debug output on serial
// #define DEBUG
// #define DEBUG2

// Status LEDs pin mapping
#define LED_RED D0
#define LED_YELLOW D1
#define LED_GREEN D2

// Status LEDs helper macros
#define LED_OFF(pin) digitalWrite(pin, LOW)
#define LED_ON(pin) digitalWrite(pin, HIGH)

// IDD specific values
#define CAN_NOMINALTORQUE_FACTOR 1024
#define TORQUE_VOLTAGE_FACTOR 4
#define MAX_VOLTAGE 8
#define DEGREES_360 360
#define SENSOR_RESOLUTION 18
#define CAN_ANGLE_FACTOR 6
#define CAN_ID_RX 0x20
#define CAN_ID_TX 0x21

// magnetic sensor instance - SPI
// AS5048a -> use as5147 config
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 3, 6, 8);

// voltage set point variable
float target_voltage = 0;

void setup() {
    // initialize status leds
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    LED_ON(LED_RED);
    LED_ON(LED_YELLOW);
    LED_ON(LED_GREEN);

    // use monitoring with serial
    Serial.begin(115200);

    // enable more verbose output for debugging
    // comment out if not needed
    SimpleFOCDebug::enable(&Serial);

    // Set filter mask and ID to allow only messages with ID 0x20
    CAN.setFilterMask_Standard(0x1FFC0000); // Mask for standard 11-bit IDs

    // Set up mailbox to exclusively listen to one CAN ID
    for (int mailbox = 0; mailbox < R7FA4M1_CAN::CAN_MAX_NO_STANDARD_MAILBOXES;
         mailbox++) {
        CAN.setFilterId_Standard(mailbox, CAN_ID_RX);
    }

    // Initialize CAN
    if (!CAN.begin(CanBitRate::BR_1000k)) {
        Serial.println("CAN.begin(...) failed.");
        for (;;) {
        }
    }

    // initialise magnetic sensor hardware
    sensor.init();
    // link the motor to the sensor
    motor.linkSensor(&sensor);

    // power supply voltage
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);

    // aligning voltage
    motor.voltage_sensor_align = 5;
    // choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // set motion control loop to be used
    motor.controller = MotionControlType::torque;

    // comment out if not needed
    motor.useMonitoring(Serial);

    // basic initialization finished, switch off green status LEDs for
    // indication
    LED_OFF(LED_GREEN);

    // initialize motor
    int motorInitResult = 0;
    do {
        motorInitResult = motor.init();
        Serial.println("Motor init Result is: " + String(motorInitResult));
    } while (motorInitResult == 0);

    // switch off yellow status LEDs for indication of motor ready
    LED_OFF(LED_YELLOW);

    // align sensor and start FOC
    int focInitResult = 0;
    do {
        focInitResult = motor.initFOC();
        Serial.println("FOC init Result is: " + String(focInitResult));
    } while (focInitResult == 0);

    // switch off red status LEDs for indication of FOC ready
    LED_OFF(LED_RED);

    // switch on green status LEDs for indication of complete initialization
    // finished
    LED_ON(LED_GREEN);
    Serial.println(F("Motor ready."));

    _delay(1000);
}

void loop() {
    static unsigned long msLast;
    static uint32_t msgCnt = 0;
    static uint32_t lastMsgCnt = 0;
    static float lastValueReq = 0;
    static uint16_t lastValueRes = 0;
    static bool canLedIsOn = true;
    static bool opLedIsOn = false;
    static bool valueHasChanged = false;

    motor.loopFOC();

    unsigned long msNow = millis();
    if ((msNow - msLast) > 300) {
        msLast = msNow;

#ifdef DEBUG
        Serial.print(msgCnt);
        Serial.print(": Req ");
        Serial.print(lastValueReq);
        Serial.print(", Res ");
        Serial.println(lastValueRes);
#endif

        // toggle LED in case CAN message has been received
        if (lastMsgCnt != msgCnt) {
            if (canLedIsOn) {
                canLedIsOn = false;
                LED_OFF(LED_GREEN);
            } else {
                canLedIsOn = true;
                LED_ON(LED_GREEN);
            }
        }

        // turn on LED if value has changed within last second, otherwise off
        if (valueHasChanged) {
            valueHasChanged = false;
            LED_ON(LED_YELLOW);
        } else {
            LED_OFF(LED_YELLOW);
            motor.move(0);
        }
    }

    int err_code;
    if (CAN.isError(err_code)) {
        Serial.print(err_code);
        Serial.println(" Error occured!");
        CAN.clearError();
    }

    if (CAN.available()) {
        // Serial.println("Message received!");
        // Receive the message and extract data
        CanMsg const msg_rx = CAN.read();
        const uint8_t *data = msg_rx.data;

        // Only count messages with ID 0x20
        if (msg_rx.id == 0x20) {
            msgCnt++;

            float set_torque =
                *((float *)&data[0]); // Assuming data[0-3] contains the torque
                                      // value

            target_voltage = (float)(set_torque / TORQUE_VOLTAGE_FACTOR);
            if (target_voltage > MAX_VOLTAGE)
                target_voltage = MAX_VOLTAGE;
            // Serial.println(target_voltage);

            // Motion control function
            // velocity, position or voltage (defined in motor.controller)
            // this function can be run at much lower frequency than loopFOC()
            // function You can also use motor.move() and set the motor.target
            // in the code
            motor.move(target_voltage);

            // Read the current angle
            float angle = sensor.getAngle();

            float angle_value = (float)(-angle * 360 / (2 * M_PI));

#ifdef DEBUG2
            Serial.print("set_torque=");
            Serial.print(set_torque);
            Serial.print(",\ttarget_voltage=");
            Serial.print(target_voltage);
            Serial.print(",\tang=");
            Serial.print(angle);
            Serial.print(",\tang_snd=");
            Serial.println(angle_value);
#endif
            // delay w Serial.println  avoid CAN.write error
            Serial.println(angle_value);
            // Create a message to be sent
            uint8_t msg_data[5] = {0, 0, 0, 0, 0};

            // Convert angle_value to a float and pack it into the msg_data byte
            // array
            memcpy(&msg_data[0], &angle_value, sizeof(float));

            // Copy the 5th byte with the rolling counter from the received
            // message to the 5th byte of the transmit message
            if (msg_rx.data_length > 4) {
                msg_data[4] = data[4];
            }

            // Check if the value has changed for status led
            // if (lastValueReq != set_torque) {
            valueHasChanged = true;
            // }

            // Save the last values for status/debugging
            lastValueReq = set_torque;
            lastValueRes = angle_value;

            // Package the message with CAN_ID_TX
            CanMsg const msg_tx(CanStandardId(CAN_ID_TX), sizeof(msg_data),
                                msg_data); // Set CAN ID to 0x03

            /* Transmit the CAN message */
            if (int const rc = CAN.write(msg_tx); rc < 0) {
                Serial.print("CAN.write(...) failed with error_code ");
                Serial.println(rc);
                // for (;;) {} // Halt on failure
            }
        }
    }
}

