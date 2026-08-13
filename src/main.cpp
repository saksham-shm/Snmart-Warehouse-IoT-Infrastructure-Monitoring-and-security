#include <DHT22.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// RFID pins
#define ssPin   5
#define rstPin  4
#define dhtOne  33
#define dhtTwo  25
#define pirPIN  32
#define ledPIN  26   // NOTE: GPIO35 cannot be used here — it's input-only on ESP32

MFRC522 rfid(ssPin, rstPin);
DHT22 dht1(dhtOne);
DHT22 dht2(dhtTwo);

// Relay, Buzzer, Servo pins
const int relay1   = 27;
const int relay2   = 14;
const int buzzer   = 12;
const int servopin = 13;
const int gasPin   = 34;


// Gas thresholds
const int gasThreshold1 = 600;
const int gasThreshold2 = 800;

// Temperature and humidity safe ranges
const float TEMP_MIN  = -10.0;
const float TEMP_MAX  =  30.0;
const float HUM_MIN   =  60.0;
const float HUM_MAX   =  90.0;

Servo servo1;

String cardsAccepted[] = {"83 AC CD 27", "01 AC 03 04", "55 66 77 88"};
String employeeName[]  = {"Employee 1",  "Employee 2",  "Employee 3"};

// ── Timing ──────────────────────────────────────────────────────────────────
unsigned long lastSensorRead    = 0;
const unsigned long sensorInterval = 2000;

unsigned long lastGasRead       = 0;
const unsigned long gasInterval = 2000;

// ── 20-second alert buzzer state (non-blocking, alternates 1800/900 Hz) ──────
bool          alertBuzzerActive  = false;
unsigned long alertBuzzerStart   = 0;          // when the 20s window began
bool          alertToneOn        = false;       // is tone currently playing?
unsigned long alertLastToggle    = 0;           // last tone/silence flip
bool          alertToneHigh      = true;        // which frequency is next
const unsigned long ALERT_DURATION  = 20000;   // total buzzer time: 20 s
const unsigned long ALERT_TONE_ON   =    150;  // ms tone ON per beep
const unsigned long ALERT_TONE_OFF  =    150;  // ms silence between beeps

// ── Alert edge detection (fire buzzer only on NEW alert, not every read) ─────
bool prevSmokeAlert = false;
bool prevEnvAlert   = false;

// ── Averaged sensor values (global so loop can use them) ─────────────────────
float avgTemp        = 0;
float avgHumid       = 0;
int   lastGasReading = 0;   // persists between 2-second gas reads for LCD

// ── Alert state flags (set by sensor reads, consumed by updateLCD) ───────────
bool smokeAlert = false;   // true when gas >= gasThreshold1
bool envAlert   = false;   // true when temp or humidity out of range

// ── Non-blocking "door open" timer (replaces delay(3000) in RFIDAccepted) ────
bool          doorOpen      = false;
unsigned long doorOpenStart = 0;
const unsigned long DOOR_OPEN_DURATION = 3000;  // how long servo stays at 90°

// ── Non-blocking motion LED timer ─────────────────────────────────────────────
//    While motionLedOn is true, the PIR is NOT re-checked (LED window is
//    running). Once the 5 s window ends, motionLedOn goes false and the PIR
//    is polled every loop() again until it triggers HIGH.
bool          motionLedOn      = false;
unsigned long motionLedStart   = 0;
const unsigned long MOTION_LED_DURATION = 5000;  // LED stays on 5 s per trigger

// ── LCD alert-display hold timers (independent of each other, non-blocking) ──
//    Each alert message must stay on the LCD for the SAME 20 s that the
//    buzzer plays, even if the underlying sensor value returns to normal
//    sooner. Env and smoke alerts are tracked separately so neither one
//    cuts the other's display time short.
bool          envAlertDisplayActive   = false;
unsigned long envAlertDisplayStart    = 0;
bool          smokeAlertDisplayActive = false;
unsigned long smokeAlertDisplayStart  = 0;
const unsigned long ALERT_DISPLAY_DURATION = 20000;  // matches ALERT_DURATION


// ────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Systm Initiation");
  lcd.setCursor(0, 1);
  lcd.print("Please Wait !");

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);

  pinMode(buzzer, OUTPUT);

  servo1.attach(servopin);
  servo1.write(90);

  pinMode(gasPin, INPUT);
  pinMode(pirPIN, INPUT);
  pinMode(ledPIN, OUTPUT);
  digitalWrite(ledPIN, LOW);
  Serial.println("PIR warming up...");
  delay(30000);   // let PIR sensor stabilize
  Serial.println("PIR ready");

  Serial.println("MFRC522 Ready");
}


// ── LCD helper ───────────────────────────────────────────────────────────────
void writetoLCD(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}


// ── Buzzer helpers (blocking — only used for RFID feedback, very short) ──────
void buzzerAccept() {
  for (int i = 0; i < 2; i++) {
    tone(buzzer, 1800); delay(100);
    noTone(buzzer);     delay(100);
  }
}

void buzzerAlert(int times) {
  for (int i = 0; i < times; i++) {
    tone(buzzer, 1800); delay(150);
    noTone(buzzer);     delay(150);
  }
}


// ── Relay helpers ────────────────────────────────────────────────────────────
void relayControl_Access(boolean value) {
  digitalWrite(relay1, value ? HIGH : LOW);
}

void relayControl_light(boolean value) {
  digitalWrite(relay2, value ? HIGH : LOW);
}


// ── RFID handlers ─────────────────────────────────────────────────────────────
//    NOTE: no delay(3000) here anymore — door timing is handled non-blocking
//    by doorOpenRun(), called every loop() iteration.
void RFIDAccepted(String accessedby) {
  writetoLCD("Accessed by:", accessedby);
  buzzerAccept();
  servo1.write(0);
  doorOpen      = true;
  doorOpenStart = millis();
}

void RFIDDenied() {
  writetoLCD("Access Denied", "Try Again");
  buzzerAlert(5);
  servo1.write(90);
}

// ── Door timer tick: call every loop iteration, no delay() ───────────────────
//    Closes the servo automatically DOOR_OPEN_DURATION ms after RFIDAccepted().
void doorOpenRun() {
  if (!doorOpen) return;

  if (millis() - doorOpenStart >= DOOR_OPEN_DURATION) {
    servo1.write(90);
    doorOpen = false;
  }
}


// ── Motion LED tick: call every loop iteration, no delay() ───────────────────
//    - If the LED is currently off: poll the PIR every call. The instant it
//      reads HIGH, turn the LED on and start a 5 s window.
//    - If the LED is currently on: ignore the PIR and just wait for the 5 s
//      window to elapse, then turn the LED off (which re-enables polling on
//      the very next loop() call — i.e. continuous checking resumes).
void motionLedRun() {
  if (!motionLedOn) {
    if (digitalRead(pirPIN) == HIGH) {
      Serial.println("Motion detected");
      digitalWrite(ledPIN, HIGH);
      motionLedOn    = true;
      motionLedStart = millis();
    }
  } else {
    if (millis() - motionLedStart >= MOTION_LED_DURATION) {
      digitalWrite(ledPIN, LOW);
      motionLedOn = false;
    }
  }
}


// ── Env alert display tick: call every loop iteration, no delay() ────────────
//    Keeps the temp/humidity alert on the LCD for the full 20 s buzzer
//    window, independent of whether the reading itself has recovered.
void envAlertDisplayRun() {
  if (!envAlertDisplayActive) return;

  if (millis() - envAlertDisplayStart >= ALERT_DISPLAY_DURATION) {
    envAlertDisplayActive = false;
    updateLCD();   // refresh immediately instead of waiting for next sensor read
  }
}

// ── Smoke alert display tick: call every loop iteration, no delay() ──────────
//    Keeps the smoke alert on the LCD for the full 20 s buzzer window,
//    independent of whether the reading itself has recovered.
void smokeAlertDisplayRun() {
  if (!smokeAlertDisplayActive) return;

  if (millis() - smokeAlertDisplayStart >= ALERT_DISPLAY_DURATION) {
    smokeAlertDisplayActive = false;
    updateLCD();   // refresh immediately instead of waiting for next sensor read
  }
}


// ── 1. Read both DHTs and store AVERAGES in dhtData[] ────────────────────────
//    dhtData[0] = avgHumidity   dhtData[1] = avgTempC   dhtData[2] = avgTempF
void readDHT(float dhtData[]) {
  float humidOne = dht1.getHumidity();
  float humidTwo = dht2.getHumidity();
  float tempOne  = dht1.getTemperature();
  float tempTwo  = dht2.getTemperature();

  if (isnan(humidOne) || isnan(tempOne)) {
    Serial.println("DHT1 read failed");
    humidOne = 0; tempOne = 0;
  }
  if (isnan(humidTwo) || isnan(tempTwo)) {
    Serial.println("DHT2 read failed");
    humidTwo = 0; tempTwo = 0;
  }

  float avgH = (humidOne + humidTwo) / 2.0;
  float avgT = (tempOne  + tempTwo)  / 2.0;
  float avgF = avgT * 9.0 / 5.0 + 32.0;

  dhtData[0] = avgH;
  dhtData[1] = avgT;
  dhtData[2] = avgF;
}


// ── Line helpers (pad to 16 chars so no stale characters remain) ─────────────
void writeLine0(String msg) {
  lcd.setCursor(0, 0);
  lcd.print(msg);
  for (int i = msg.length(); i < 16; i++) lcd.print(" ");
}

void writeLine1(String msg) {
  lcd.setCursor(0, 1);
  lcd.print(msg);
  for (int i = msg.length(); i < 16; i++) lcd.print(" ");
}

// ── Unified LCD update — called after every sensor read ──────────────────────
//
//  Four cases based on smokeAlert / envAlert flags:
//
//  smokeAlert=F  envAlert=F  → L0: temp+humid normal   L1: smoke value OK
//  smokeAlert=T  envAlert=F  → L0: temp+humid normal   L1: !SMOKE ALERT!
//  smokeAlert=F  envAlert=T  → L0: !TEMP/HUM ALERT!    L1: smoke value OK
//  smokeAlert=T  envAlert=T  → L0: !TEMP/HUM ALERT!    L1: !SMOKE ALERT!
//
//  Buzzer controls are NOT touched here.
void updateLCD() {
  // ── Line 0: env alert overrides normal temp/humid display ────────────────
  //    Driven by envAlertDisplayActive (20 s hold), not the live envAlert
  //    flag, so the message stays up for the full buzzer duration.
  if (envAlertDisplayActive) {
    bool tempOK  = (avgTemp  >= TEMP_MIN && avgTemp  <= TEMP_MAX);
    bool humidOK = (avgHumid >= HUM_MIN  && avgHumid <= HUM_MAX);

    if (!tempOK && !humidOK) {
      writeLine0("!T+H OUT RANGE!");
    } else if (!tempOK) {
      writeLine0("!TEMP ALERT " + String(avgTemp, 1) + (char)223 + "!");
    } else {
      writeLine0("!HUM ALERT " + String(avgHumid, 1) + "%!");
    }
  } else {
    // Normal: show averaged readings
    writeLine0("H:" + String(avgHumid, 1) + "% T:" + String(avgTemp, 1) + (char)223 + "C");
  }

  // ── Line 1: smoke alert overrides normal smoke value display ─────────────
  //    Driven by smokeAlertDisplayActive (20 s hold), not the live
  //    smokeAlert flag, so the message stays up for the full buzzer duration.
  if (smokeAlertDisplayActive) {
    if (lastGasReading > gasThreshold2) {
      writeLine1("!SMOKE DANGER " + String(lastGasReading) + "!");
    } else {
      writeLine1("!SMOKE ALERT " + String(lastGasReading) + "!");
    }
  } else {
    writeLine1("Smoke:" + String(lastGasReading) + " OK");
  }
}


// ── Alert buzzer: arm for a fresh 20-second run ──────────────────────────────
//    Call once when a new alert is detected. Restarts the window even if
//    already running (second alert while first is playing → resets to 20 s).
void triggerAlertBuzzer() {
  alertBuzzerActive = true;
  alertBuzzerStart  = millis();
  alertToneHigh     = true;
  alertToneOn       = false;
  alertLastToggle   = millis();
  Serial.println("Alert buzzer armed: 20 s");
}

// ── Alert buzzer tick: call every loop iteration, no delay() ─────────────────
//    Alternates tone(1800) / tone(900) every ALERT_TONE_ON ms,
//    with ALERT_TONE_OFF ms silence between beeps.
//    Stops itself after ALERT_DURATION ms.
void alertBuzzerRun() {
  if (!alertBuzzerActive) return;

  unsigned long now = millis();

  // Auto-stop after 20 seconds
  if (now - alertBuzzerStart >= ALERT_DURATION) {
    alertBuzzerActive = false;
    noTone(buzzer);
    Serial.println("Alert buzzer finished 20 s");
    return;
  }

  if (alertToneOn) {
    // Currently playing — check if it's time to go silent
    if (now - alertLastToggle >= ALERT_TONE_ON) {
      noTone(buzzer);
      alertToneOn     = false;
      alertLastToggle = now;
    }
  } else {
    // Currently silent — check if it's time to play the next tone
    if (now - alertLastToggle >= ALERT_TONE_OFF) {
      int freq = alertToneHigh ? 1800 : 900;
      tone(buzzer, freq);
      alertToneHigh   = !alertToneHigh;   // alternate for next beep
      alertToneOn     = true;
      alertLastToggle = now;
    }
  }
}



// ────────────────────────────────────────────────────────────────────────────
void loop() {

  // ── A. Alert buzzer tick (runs every iteration, no delay) ────────────────
  alertBuzzerRun();

  // ── A2. Door-open timer tick (runs every iteration, no delay) ────────────
  doorOpenRun();

  // ── A3. Motion LED tick (runs every iteration, no delay) ─────────────────
  motionLedRun();

  // ── A4. LCD alert-display hold ticks (runs every iteration, no delay) ────
  envAlertDisplayRun();
  smokeAlertDisplayRun();

  // ── B. DHT sensor read (every sensorInterval ms) ─────────────────────────
  float reading[3];   // [0]=avgHumid  [1]=avgTempC  [2]=avgTempF
  if (millis() - lastSensorRead >= sensorInterval) {
    lastSensorRead = millis();
    readDHT(reading);

    avgTemp  = reading[1];
    avgHumid = reading[0];

    bool tempOK  = (avgTemp  >= TEMP_MIN && avgTemp  <= TEMP_MAX);
    bool humidOK = (avgHumid >= HUM_MIN  && avgHumid <= HUM_MAX);
    envAlert = (!tempOK || !humidOK);

    // Fire buzzer only on the rising edge (condition just became true)
    if (envAlert && !prevEnvAlert) {
      Serial.println("ALERT: Env out of range T=" + String(avgTemp) + " H=" + String(avgHumid));
      triggerAlertBuzzer();
      envAlertDisplayActive = true;
      envAlertDisplayStart  = millis();
    }
    prevEnvAlert = envAlert;

    updateLCD();
  }

  // ── C. Gas / smoke sensor (sampled every gasInterval ms) ─────────────────
  if (millis() - lastGasRead >= gasInterval) {
    lastGasRead    = millis();
    lastGasReading = analogRead(gasPin);
    Serial.println("Gas Value: " + String(lastGasReading));

    smokeAlert = (lastGasReading >= gasThreshold1);

    // Fire buzzer only on the rising edge (condition just became true)
    if (smokeAlert && !prevSmokeAlert) {
      Serial.println("ALERT: Smoke detected: " + String(lastGasReading));
      triggerAlertBuzzer();
      smokeAlertDisplayActive = true;
      smokeAlertDisplayStart  = millis();
    }
    prevSmokeAlert = smokeAlert;

    updateLCD();
  }

  // ── D. RFID scan ─────────────────────────────────────────────────────────
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  String content = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    content.concat(String(rfid.uid.uidByte[i] < 0x10 ? " 0" : " "));
    content.concat(String(rfid.uid.uidByte[i], HEX));
  }
  content.toUpperCase();
  String readUID = content.substring(1);

  Serial.print("UID: ");
  Serial.println(readUID);

  boolean allowed = false;
  const int NumCards = sizeof(cardsAccepted) / sizeof(cardsAccepted[0]);

  for (int i = 0; i < NumCards; i++) {
    if (readUID == cardsAccepted[i]) {
      RFIDAccepted(employeeName[i]);
      allowed = true;
      break;
    }
  }

  if (!allowed) RFIDDenied();

  Serial.println("End of line");
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();   // ← FIX: release crypto session so next card reliably detected
}
