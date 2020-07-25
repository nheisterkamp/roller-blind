/**
 * @TODO
 * - [ ] add I2C LCD for debugging
 * - [ ] write calibration code
 * - [ ] save calibration to EEPROM
 * - [ ] read calibration from EEPROM
 * - [ ] implement endstop interrupt (in loop now, is ok?)
 * - [ ] connect to wifi
 * - [ ] listen on web server for commands
 */
#include <Arduino.h>
#include <DoubleResetDetect.h>
#include <ESP8266WiFi.h>
#include <AccelStepper.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

// Define pin connections
#define DIR_PIN D1
#define STEP_PIN D2
#define ENDSTOP_PIN D5
#define CALIBRATE_PIN D6
#define MICROSTEP_PIN D7
#define ENABLE_PIN D8

// Define motor interface type
#define STEPPER_INTERFACE 1

// Creates an instance
AccelStepper blinds(STEPPER_INTERFACE, STEP_PIN, DIR_PIN);

const int DIR = -1;
const int HIGH_SPEED_THRESHOLD = 20;
const int LOW_SPEED = 200;
const int LOW_SPEED_ACCELERATION = 100;
const int HIGH_SPEED = 300;
const int HIGH_SPEED_ACCELERATION = 150;

int POSITION = 100;
int MAX_POSITION_STEPS = 100;
int TARGET_POSITION = 100;

bool HOMING = false;
bool CALIBRATING = false;

const char* SSID = "Our House";
const char* PASSWORD = "oldskool";

DoubleResetDetect drd(5.0, 0x00);

// set up web server
ESP8266WebServer server(80);

void setLowPowerMode() {
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(ENABLE_PIN, 1);
}

void setHighPowerMode() {
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(ENABLE_PIN, 0);
}

void setLowSpeed() {
  blinds.setMaxSpeed(LOW_SPEED);
  blinds.setAcceleration(LOW_SPEED_ACCELERATION);
  blinds.setSpeed(LOW_SPEED);
}

void setHighSpeed() {
  blinds.setMaxSpeed(HIGH_SPEED);
  blinds.setAcceleration(HIGH_SPEED_ACCELERATION);
  blinds.setSpeed(HIGH_SPEED);
}

long eeprom_readLong(int address) {
  return ((long)EEPROM.read(address) << 24) +
        ((long)EEPROM.read(address + 1) << 16) +
        ((long)EEPROM.read(address + 2) << 8) +
        (long)EEPROM.read(address + 3);
}

long eeprom_writeLong(int address, long value) {
  EEPROM.write(address, (value >> 24) & 0xFF);
  EEPROM.write(address + 1, (value >> 16) & 0xFF);
  EEPROM.write(address + 2, (value >> 8) & 0xFF);
  EEPROM.write(address + 3, value & 0xFF);
}

void clearCurrentState() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void saveCurrentState() {
  Serial.println("save to EEPROM");
  EEPROM.write(0, POSITION);
  eeprom_writeLong(1, MAX_POSITION_STEPS);
  EEPROM.commit();
}

void loadCurrentState() {
  Serial.println("load from EEPRxOM");
  long position = EEPROM.read(0);
  Serial.println("read position: " + String(position));
  long maxPositionSteps = eeprom_readLong(1);
  Serial.println("read maxPositionSteps: " + String(maxPositionSteps));

  if (position >= 0 && position <= 100 && maxPositionSteps > 0) {
    Serial.println("Seems legit, load from memory");
    POSITION = position;
    MAX_POSITION_STEPS = maxPositionSteps;
  } else {
    Serial.println("Probably garbage data, ignore");
  }
}

void setupWifi() {
  WiFi.begin(SSID, PASSWORD);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  delay(2000);
}

// bool LED_ON = false;

void ICACHE_RAM_ATTR endStopInterrupt() {
  Serial.println("endStopInterrupt()");
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  Serial.println("ENDSTOP FALLING INTERRUPT " + String(interrupt_time));

  if (interrupt_time - last_interrupt_time > 200) {
    // LED_ON = !LED_ON;
    // digitalWrite(LED_BUILTIN, LED_ON);

    if (HOMING || CALIBRATING) {
      Serial.println("ENDSTOP FALLING INTERRUPT DEBOUNCED");
      POSITION = 100;
      HOMING = false;
      blinds.stop();
      blinds.setCurrentPosition(0);

      if (CALIBRATING) {
        // if we're calibrating we can move down now
        setHighPowerMode();
        setHighSpeed();
        blinds.move(100000 * DIR);
      } else {
        setLowPowerMode();
      }
    }
  }

  last_interrupt_time = interrupt_time;
}

void setupEndStopInterrupt() {
  attachInterrupt(digitalPinToInterrupt(ENDSTOP_PIN), endStopInterrupt, FALLING);
}

void homeBlinds() {
  if (digitalRead(ENDSTOP_PIN)) {
    HOMING = true;
    setHighPowerMode();
    setHighSpeed();
    blinds.move(-10000000 * DIR);
  } else {
    setLowPowerMode();
  }
}

// void calibrateBlinds() {
// 	CALIBRATING = true;
// 	homeBlinds();
// }

void setupBlinds() {
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  setHighSpeed();
}

void ICACHE_RAM_ATTR calibrationInterrupt() {
  Serial.println("calibrationInterrupt()");
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  Serial.println("CALIBRATION RISING INTERRUPT " + String(interrupt_time));

  if (interrupt_time - last_interrupt_time > 200) {
    Serial.println("CALIBRATION RISING INTERRUPT DEBOUNCED");

    if (CALIBRATING) {
      // if we're already calibrating, we are done now
      CALIBRATING = false;

      blinds.stop();
      POSITION = 0;
      MAX_POSITION_STEPS = blinds.currentPosition() * DIR;
      saveCurrentState();

      setLowPowerMode();
    } else {
      // else we start calibrating
      CALIBRATING = true;

      if (!digitalRead(ENDSTOP_PIN)) {
        // if endstop is pressed, move down to calibrate
        setHighPowerMode();
        setHighSpeed();
        blinds.move(100000 * DIR);
      } else {
        // else start with homing
        homeBlinds();
      }
    }
  }

  last_interrupt_time = interrupt_time;
}

void setupCalibrationInterrupt() {
  attachInterrupt(digitalPinToInterrupt(CALIBRATE_PIN), calibrationInterrupt, RISING);
}

void blindStopR() {
  Serial.println("blindStopR");
  server.send(200, "application/json", "{\"success\":true}");
}

void moveTo(int new_position) {
  TARGET_POSITION = new_position;
  if (new_position == 100) {
    homeBlinds();
  } else {
    setHighPowerMode();
    if (abs(TARGET_POSITION - POSITION) <= HIGH_SPEED_THRESHOLD) {
      setLowSpeed();
    } else {
      setHighSpeed();
    }
    blinds.moveTo(((long)((float)MAX_POSITION_STEPS * (1 - ((float)TARGET_POSITION / 100.0)))) * DIR);
  }
}

void blindSetPositionR() {
  int requested_position = server.arg("pos").toInt();

  if (requested_position >= 0 && requested_position <= 100) {
    // TARGET_POSITION = requested_position;
    moveTo(requested_position);
    Serial.print("TARGET_POSITION: ");
    Serial.println(TARGET_POSITION);
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(500, "application/json", "{\"success\":false}");
  }
}

void blindGetPositionR() {
  Serial.println("blindGetPositionR");
  server.send(200, "application/json", "{\"ShutterPosition1\":" + String((int)POSITION) + "}");
}

void setupServer() {
  server.on("/set_position", blindSetPositionR);
  server.on("/get_position", blindGetPositionR);
  server.on("/stop", blindStopR);

  server.begin();
}

void setup() {
  Serial.begin(9600);

  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(CALIBRATE_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(MICROSTEP_PIN, OUTPUT);
  // digitalWrite(MICROSTEP_PIN, HIGH);

  EEPROM.begin(512);
  loadCurrentState();

  setupBlinds();
  setHighPowerMode();
  setupWifi();
  setupEndStopInterrupt();
  setupCalibrationInterrupt();

  homeBlinds();
  setupServer();
}

void loop() {
  POSITION = 100 - (int)((float)(blinds.currentPosition() * DIR) / (float)MAX_POSITION_STEPS);

  if (blinds.distanceToGo() == 0 && !HOMING && !CALIBRATING) {
    delay(1000);
    setLowPowerMode();
  }

  blinds.run();
  server.handleClient();
}
