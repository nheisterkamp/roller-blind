#include <Arduino.h>
#include <DoubleResetDetect.h>
#include <ESP8266WiFi.h>
#include <AccelStepper.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

// Define pin connections
#define DIR_PIN D1
#define STEP_PIN D2
#define ENDSTOP_PIN D4
#define CALIBRATE_PIN D6
#define MICROSTEP_PIN D7
#define ENABLE_PIN D8

// Define motor interface type
#define STEPPER_INTERFACE 1

// Creates an instance
AccelStepper blinds(STEPPER_INTERFACE, STEP_PIN, DIR_PIN);

const int DIR = -1;

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

long eeprom_readLong(int address) {
  return ((long)EEPROM.read(address) << 24) +
        ((long)EEPROM.read(address + 1) << 16) +
        ((long)EEPROM.read(address + 2) << 8) +
        (long)EEPROM.read(address + 3);
}

void eeprom_writeLong(int address, long value) {
  EEPROM.write(address, (value >> 24) & 0xFF);
  EEPROM.write(address + 1, (value >> 16) & 0xFF);
  EEPROM.write(address + 2, (value >> 8) & 0xFF);
  EEPROM.write(address + 3, value & 0xFF);
}

void clearCurrentState() {
  for (unsigned int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void saveCurrentState() {
  EEPROM.write(0, POSITION);
  eeprom_writeLong(1, MAX_POSITION_STEPS);
  EEPROM.commit();
}

void loadCurrentState() {
  long position = EEPROM.read(0);
  long maxPositionSteps = eeprom_readLong(1);

  if (position >= 0 && position <= 100 && maxPositionSteps > 0) {
    POSITION = position;
    MAX_POSITION_STEPS = maxPositionSteps;
  } else {
  }
}

void setupWifi() {
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void ICACHE_RAM_ATTR endStopInterrupt() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > 200) {
    if (!HOMING && !CALIBRATING) {
      return;
    }
    
    if (HOMING) {
      POSITION = 100;
      blinds.stop();
      blinds.setCurrentPosition(0);

      if (!CALIBRATING) {
        setLowPowerMode();
      }
    }

    if (CALIBRATING) {
      // if we're calibrating
      if (HOMING) {
        // and just arrived home we can move down now
        setHighPowerMode();
        blinds.move(1000000 * DIR);
      } else {
        // we are done calibrating
        CALIBRATING = false;
        
        blinds.stop();

        POSITION = 0;
        MAX_POSITION_STEPS = blinds.currentPosition() * DIR;

        saveCurrentState();
        setLowPowerMode();
      }
    }

    HOMING = false;
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
    blinds.move(-10000000 * DIR);
  } else if (CALIBRATING) {
    setHighPowerMode();
    blinds.move(10000000 * DIR);
  }
}

void setupBlinds() {
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  blinds.setMaxSpeed(750.0);
  blinds.setAcceleration(1000.0);
  blinds.setSpeed(750.0);
}

void ICACHE_RAM_ATTR calibrationInterrupt() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > 200) {

    if (CALIBRATING) {
      // if we're already calibrating, we are done now
      blinds.stop();

      if (!HOMING) {
        POSITION = 0;
        MAX_POSITION_STEPS = blinds.currentPosition() * DIR;

        saveCurrentState();
      }

      setLowPowerMode();

      HOMING = false;
      CALIBRATING = false;
    } else {
      // else we start calibrating
      CALIBRATING = true;

      if (!digitalRead(ENDSTOP_PIN)) {
        // if endstop is pressed, move down to calibrate
        setHighPowerMode();
        blinds.move(10000000 * DIR);
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
  blinds.stop();
  delay(1000);
  setLowPowerMode();
  server.send(200, "application/json", "{\"success\":true}");
}

void blindCalibrateR() {
  CALIBRATING = true;
  homeBlinds();
  server.send(200, "application/json", "{\"success\":true}");
}

void blindStatusR() {
  server.send(200, "application/json", "{\"currentPosition\":" + String(blinds.currentPosition()) + "}");
}

void moveTo(int new_position) {
  TARGET_POSITION = new_position;
  POSITION = new_position;
  setHighPowerMode();
  if (new_position == 100) {
    homeBlinds();
  } else {
    blinds.moveTo((int)(MAX_POSITION_STEPS * (1 - (TARGET_POSITION / 100.0))) * DIR);
  }
}

void blindSetPositionR() {
  int requested_position = server.arg("pos").toInt();

  if (requested_position >= 0 && requested_position <= 100) {
    moveTo(requested_position);
    server.send(200, "application/json", "{\"success\":true}");
  } else {
    server.send(500, "application/json", "{\"success\":false}");
  }
}

void blindGetPositionR() {
  server.send(200, "application/json", "{\"ShutterPosition1\":" + String(POSITION) + "}");
}

void setupServer() {
  server.on("/set_position", blindSetPositionR);
  server.on("/get_position", blindGetPositionR);
  server.on("/stop", blindStopR);
  server.on("/calibrate", blindCalibrateR);
  server.on("/status", blindStatusR);

  server.begin();
}

void setup() {
  pinMode(ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(MICROSTEP_PIN, OUTPUT);
  digitalWrite(MICROSTEP_PIN, HIGH);

  EEPROM.begin(512);
  loadCurrentState();

  setupBlinds();
  setHighPowerMode();
  setupWifi();
  setupEndStopInterrupt();

  homeBlinds();
  setupServer();
}

void loop() {
  if (blinds.distanceToGo() <= 5 && !HOMING && !CALIBRATING) {
    blinds.setCurrentPosition(0);
    blinds.runToPosition();
    delay(1000);
    setLowPowerMode();
  }

  blinds.run();
  server.handleClient();
}
