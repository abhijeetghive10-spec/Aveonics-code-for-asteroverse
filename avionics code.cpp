#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <LoRa.h>

// ======================================================
// HARDWARE
// ======================================================

Adafruit_BMP280 bmp;
Servo ejectionServo;

// Servo + buzzer
const int SERVO_PIN  = 4;
const int BUZZER_PIN = 25;

// BMP280
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// RA-02 / SX1278
const int LORA_SCK  = 18;
const int LORA_MISO = 19;
const int LORA_MOSI = 23;
const int LORA_CS   = 5;
const int LORA_RST  = 14;
const int LORA_DIO0 = 26;


// ======================================================
// SERVO POSITIONS
// ======================================================

const int ANGLE_GROUND = 80;
const int ANGLE_EJECT  = 0;


// ======================================================
// TEST FLIGHT PARAMETERS
// ======================================================

const float LIFT_OFF_HEIGHT = 0.30;  // m
const float MIN_FLIGHT_HEIGHT = 1.50; // m
const float APOGEE_DROP_M = 0.30;     // m

const unsigned long APOGEE_TO_DEPLOY_MS = 1000;


// ======================================================
// TIMING
// ======================================================

const unsigned long SAMPLE_INTERVAL_MS = 10;   // 100 Hz
const unsigned long TELEMETRY_INTERVAL_MS = 500; // 2 Hz

unsigned long lastSampleTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long apogeeDetectTime = 0;


// ======================================================
// ALTITUDE VARIABLES
// ======================================================

float baselineAltitude = 0.0;
float currentAltitude = 0.0;
float filteredAltitude = 0.0;
float maxAltitude = 0.0;

const float ALPHA = 0.35;


// ======================================================
// FLIGHT STATE
// ======================================================

enum FlightState
{
  STATE_READY,
  STATE_ASCENT,
  STATE_APOGEE_WAIT,
  STATE_DEPLOYED
};

FlightState currentState = STATE_READY;


// ======================================================
// TELEMETRY
// ======================================================

// Example ID from competition document.
// CHANGE THIS to the organizer-assigned Rocket ID.
const uint8_t ROCKET_ID = 37;

uint16_t sequenceNumber = 0;


// ======================================================
// BUZZER
// ======================================================

void beepOnce(unsigned long duration)
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}


// 100 ms ON + 100 ms OFF
void soundBeeps(int count)
{
  for (int i = 0; i < count; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);

    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}


// ======================================================
// GROUND ZERO CALIBRATION
// ======================================================

void calibrateGround()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("CALIBRATING GROUND ZERO...");
  Serial.println("================================");

  delay(1000);

  float altitudeSum = 0.0;

  for (int i = 0; i < 50; i++)
  {
    altitudeSum += bmp.readAltitude(1013.25);
    delay(20);
  }

  baselineAltitude = altitudeSum / 50.0;

  currentAltitude = 0.0;
  filteredAltitude = 0.0;
  maxAltitude = 0.0;

  Serial.print("Ground baseline = ");
  Serial.print(baselineAltitude, 2);
  Serial.println(" m");

  // ONE LONG BEEP = ARMED
  beepOnce(400);

  Serial.println("BEEP: 1 LONG");
  Serial.println("SYSTEM ARMED");
  Serial.println("Waiting for lift-off...");
  Serial.println();
}


// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("ROCKET AVIONICS TEST FIRMWARE");
  Serial.println("==========================================");

  // -------------------------
  // I2C
  // -------------------------

  Wire.begin(SDA_PIN, SCL_PIN);


  // -------------------------
  // BUZZER
  // -------------------------

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);


  // -------------------------
  // SERVO
  // -------------------------

  ejectionServo.setPeriodHertz(50);

  ejectionServo.attach(
    SERVO_PIN,
    500,
    2400
  );

  ejectionServo.write(ANGLE_GROUND);

  Serial.println("Servo: OK");
  Serial.println("Servo position: 80 degrees");


  // -------------------------
  // BMP280
  // -------------------------

  if (!bmp.begin(0x76))
  {
    if (!bmp.begin(0x77))
    {
      Serial.println("CRITICAL: BMP280 NOT FOUND!");

      while (true)
      {
        beepOnce(100);
        delay(500);
      }
    }
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_1
  );

  Serial.println("BMP280: OK");


  // -------------------------
  // LORA
  // -------------------------

  SPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_CS
  );

  LoRa.setPins(
    LORA_CS,
    LORA_RST,
    LORA_DIO0
  );

  if (!LoRa.begin(434500000))
  {
    Serial.println("CRITICAL: LoRa FAILED!");

    while (true)
    {
      beepOnce(100);
      delay(500);
    }
  }

  // Competition radio configuration
  LoRa.setSignalBandwidth(125E3);
  LoRa.setSpreadingFactor(7);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  //LoRa.explicitHeaderMode();
  LoRa.enableCrc();

  // Bench/test setting from the original code.
  // Organizer controls final RF configuration.
  LoRa.setTxPower(10);

  Serial.println("LoRa: OK");
  Serial.println("Frequency: 434.500 MHz");
  Serial.println("Bandwidth: 125 kHz");
  Serial.println("SF: 7");
  Serial.println("CR: 4/5");
  Serial.println("Preamble: 8");
  Serial.println("CRC: ON");


  // -------------------------
  // GROUND CALIBRATION
  // -------------------------

  calibrateGround();
}


// ======================================================
// TELEMETRY
// ======================================================

void sendTelemetry()
{
  uint8_t packetType;

  float telemetryAltitude;


  // --------------------------------------------------
  // Normal flight
  // --------------------------------------------------

  if (currentState == STATE_READY ||
      currentState == STATE_ASCENT)
  {
    packetType = 0x01;
    telemetryAltitude = filteredAltitude;
  }

  // --------------------------------------------------
  // Apogee detected / deployed
  // --------------------------------------------------

  else
  {
    packetType = 0x02;
    telemetryAltitude = maxAltitude;
  }


  if (telemetryAltitude < 0.0)
    telemetryAltitude = 0.0;


  // Convert metres -> 0.1 metre units
  uint16_t altitudePayload =
    (uint16_t)(telemetryAltitude * 10.0);


  // ==================================================
  // REQUIRED 6-BYTE PACKET
  //
  // BYTE 0 = Rocket ID
  // BYTE 1 = TYPE
  // BYTE 2 = SEQ MSB
  // BYTE 3 = SEQ LSB
  // BYTE 4 = ALT MSB
  // BYTE 5 = ALT LSB
  // ==================================================

  uint8_t packet[6];

  packet[0] = ROCKET_ID;
  packet[1] = packetType;

  packet[2] = (sequenceNumber >> 8) & 0xFF;
  packet[3] = sequenceNumber & 0xFF;

  packet[4] = (altitudePayload >> 8) & 0xFF;
  packet[5] = altitudePayload & 0xFF;


  // Send
  LoRa.beginPacket();
  LoRa.write(packet, 6);
  LoRa.endPacket();


  // ==================================================
  // SERIAL DEBUG
  // ==================================================

  Serial.print("TX | ");

  Serial.print("ID=");
  Serial.print(ROCKET_ID);

  Serial.print(" TYPE=0x");

  if (packetType < 0x10)
    Serial.print("0");

  Serial.print(packetType, HEX);

  Serial.print(" SEQ=");
  Serial.print(sequenceNumber);

  Serial.print(" ALT=");
  Serial.print(telemetryAltitude, 1);

  Serial.print("m");

  Serial.print(" RAW=");
  Serial.print(altitudePayload);

  Serial.print(" STATE=");

  switch (currentState)
  {
    case STATE_READY:
      Serial.print("READY");
      break;

    case STATE_ASCENT:
      Serial.print("ASCENT");
      break;

    case STATE_APOGEE_WAIT:
      Serial.print("APOGEE_WAIT");
      break;

    case STATE_DEPLOYED:
      Serial.print("DEPLOYED");
      break;
  }

  Serial.println();

  sequenceNumber++;
}


// ======================================================
// MAIN LOOP
// ======================================================

void loop()
{
  // ==================================================
  // 100 Hz SENSOR / FLIGHT STATE ENGINE
  // ==================================================

  if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS)
  {
    lastSampleTime = millis();


    // ----------------------------------------------
    // Read relative altitude
    // ----------------------------------------------

    currentAltitude =
      bmp.readAltitude(1013.25) - baselineAltitude;


    // ----------------------------------------------
    // Low-pass filtering
    // ----------------------------------------------

    filteredAltitude =
      (ALPHA * currentAltitude) +
      ((1.0 - ALPHA) * filteredAltitude);


    // ==================================================
    // STATE MACHINE
    // ==================================================

    switch (currentState)
    {

      // ==============================================
      // READY
      // ==============================================

      case STATE_READY:

        if (filteredAltitude > LIFT_OFF_HEIGHT)
        {
          currentState = STATE_ASCENT;

          maxAltitude = filteredAltitude;

          Serial.println();
          Serial.println(">>> LIFT-OFF DETECTED <<<");
          Serial.println("Tracking ascent...");
        }

        break;


      // ==============================================
      // ASCENT
      // ==============================================

      case STATE_ASCENT:

        // Track maximum altitude
        if (filteredAltitude > maxAltitude)
        {
          maxAltitude = filteredAltitude;
        }


        // ----------------------------------------------
        // APOGEE CONDITION
        //
        // Peak must be >= 1.5 m
        // AND altitude must fall 0.30 m
        // from the peak.
        // ----------------------------------------------

        if (
          maxAltitude >= MIN_FLIGHT_HEIGHT &&
          filteredAltitude <
          (maxAltitude - APOGEE_DROP_M)
        )
        {
          currentState = STATE_APOGEE_WAIT;

          apogeeDetectTime = millis();


          // ONE SHORT BEEP
          beepOnce(80);


          Serial.println();
          Serial.println(">>> APOGEE / DESCENT DETECTED <<<");

          Serial.print("Maximum altitude = ");
          Serial.print(maxAltitude, 2);
          Serial.println(" m");

          Serial.println("BEEP: 1 SHORT");
          Serial.println("Waiting 1 second before ejection...");
        }

        break;


      // ==============================================
      // APOGEE WAIT
      // ==============================================

      case STATE_APOGEE_WAIT:

        if (
          millis() - apogeeDetectTime >=
          APOGEE_TO_DEPLOY_MS
        )
        {
          // ------------------------------------------
          // EJECT
          // ------------------------------------------

          ejectionServo.write(ANGLE_EJECT);

          currentState = STATE_DEPLOYED;


          Serial.println();
          Serial.println(">>> PARACHUTE EJECTION <<<");
          Serial.println("Servo: 80 deg -> 0 deg");


          // THREE BEEPS
          soundBeeps(3);

          Serial.println("BEEP: 3");
          Serial.println("SYSTEM LOCKED - DEPLOYED");
          Serial.println("POWER CYCLE REQUIRED FOR RESET");
          Serial.println();
        }

        break;


      // ==============================================
      // DEPLOYED
      // ==============================================

      case STATE_DEPLOYED:

        // NOTHING HAPPENS HERE.
        //
        // NO TOUCHDOWN STATE
        // NO 2 SECOND RE-ARM
        // NO SERVO RESET
        //
        // System remains permanently locked
        // until physical power is removed.

        break;
    }
  }


  // ==================================================
  // 2 Hz TELEMETRY
  // ==================================================

  if (
    millis() - lastTelemetryTime >=
    TELEMETRY_INTERVAL_MS
  )
  {
    lastTelemetryTime = millis();

    sendTelemetry();
  }
}