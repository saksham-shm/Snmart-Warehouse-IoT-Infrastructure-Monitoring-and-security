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

MFRC522 rfid(ssPin, rstPin);
DHT22 dht1(dhtOne);
DHT22 dht2(dhtTwo);

// Relay, Buzzer, Servo pins
const int relay1   = 27;
const int relay2   = 14;
const int buzzer   = 12;
const int servopin = 13;
const int gasPin   = 34;

// Push button pin for silencing smoke alarm
const int buttonPin = 26;

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

// ── Non-blocking smoke buzzer state ─────────────────────────────────────────
bool     smokeAlarmActive    = false;
bool     buzzerToneOn        = false;
unsigned long lastBuzzerToggle = 0;
const unsigned long buzzerOnTime  = 300;   // ms tone ON
const unsigned long buzzerOffTime = 200;   // ms tone OFF

// ── Button edge detection ────────────────────────────────────────────────────
int buttonLastState = LOW;

// ── Averaged sensor values (global so loop can use them) ─────────────────────
float avgTemp        = 0;
float avgHumid       = 0;
int   lastGasReading = 0;   // persists between 2-second gas reads for LCD

// ── Alert state flags (set by sensor reads, consumed by updateLCD) ───────────
bool smokeAlert = false;   // true when gas >= gasThreshold1
bool envAlert   = false;   // true when temp or humidity out of range


// ────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("hello");
  lcd.setCursor(0, 1);
  lcd.print("Begin");

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);

  pinMode(buzzer, OUTPUT);
  pinMode(buttonPin, INPUT_PULLDOWN);   // uses ESP32 internal pull-down

  servo1.attach(servopin);
  servo1.write(0);

  pinMode(gasPin, INPUT);
  pinMode(pirPIN, INPUT);

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


// ── Buzzer helpers (blocking — only used for RFID feedback) ──────────────────
void buzzerAccept() {
  for (int i = 0; i < 2; i++) {
    tone(buzzer, 1800); delay(100);
    noTone(buzzer);     delay(100);
  }
  delay(500);
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
void RFIDAccepted(String accessedby) {
  writetoLCD("Accessed by:", accessedby);
  buzzerAccept();
  servo1.write(90);
  delay(3000);
  servo1.write(0);
}

void RFIDDenied() {
  writetoLCD("Access Denied", "Try Again");
  buzzerAlert(5);
  servo1.write(0);
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
  if (envAlert) {
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
  if (smokeAlert) {
    if (lastGasReading > gasThreshold2) {
      writeLine1("!SMOKE DANGER " + String(lastGasReading) + "!");
    } else {
      writeLine1("!SMOKE ALERT " + String(lastGasReading) + "!");
    }
  } else {
    writeLine1("Smoke:" + String(lastGasReading) + " OK");
  }
}


// ── 3. Non-blocking smoke alarm buzzer ──────────────────────────────────────
//    Call every loop iteration. Uses millis() — NO delay().
//    smokeAlarmActive must be set true before calling (done in loop).
//    Push button (edge: LOW→HIGH) silences the alarm.
void smokeAlertBuzzer() {
  if (!smokeAlarmActive) {
    noTone(buzzer);
    return;
  }

  unsigned long now = millis();

  if (buzzerToneOn) {
    if (now - lastBuzzerToggle >= buzzerOnTime) {
      noTone(buzzer);
      buzzerToneOn      = false;
      lastBuzzerToggle  = now;
    }
  } else {
    if (now - lastBuzzerToggle >= buzzerOffTime) {
      tone(buzzer, 1800);
      buzzerToneOn      = true;
      lastBuzzerToggle  = now;
    }
  }
}



// ────────────────────────────────────────────────────────────────────────────
void loop() {

  // ── A. Push-button edge detection (silences smoke alarm) ─────────────────
  int buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH && buttonLastState == LOW) {
    // Rising edge detected — silence the alarm
    if (smokeAlarmActive) {
      smokeAlarmActive = false;
      noTone(buzzer);
      Serial.println("Smoke alarm silenced by button");
      writetoLCD("Alarm silenced", "by operator");
      delay(1500);   // Brief acknowledgement display
    }
  }
  buttonLastState = buttonState;

  // ── B. Non-blocking smoke buzzer (runs every iteration) ──────────────────
  smokeAlertBuzzer();

  // ── C. DHT sensor read (every sensorInterval ms) ─────────────────────────
  float reading[3];   // [0]=avgHumid  [1]=avgTempC  [2]=avgTempF
  if (millis() - lastSensorRead >= sensorInterval) {
    lastSensorRead = millis();
    readDHT(reading);

    avgTemp  = reading[1];
    avgHumid = reading[0];

    // Set flag — clears itself automatically when readings return to safe range
    bool tempOK  = (avgTemp  >= TEMP_MIN && avgTemp  <= TEMP_MAX);
    bool humidOK = (avgHumid >= HUM_MIN  && avgHumid <= HUM_MAX);
    envAlert = (!tempOK || !humidOK);

    if (envAlert) Serial.println("ALERT: Env out of range T=" + String(avgTemp) + " H=" + String(avgHumid));

    updateLCD();
  }

  // ── D. Gas / smoke sensor (sampled every gasInterval ms) ────────────────
  if (millis() - lastGasRead >= gasInterval) {
    lastGasRead    = millis();
    lastGasReading = analogRead(gasPin);
    Serial.println("Gas Value: " + String(lastGasReading));

    // Set flag — clears itself when reading drops back below threshold
    smokeAlert = (lastGasReading >= gasThreshold1);

    if (smokeAlert) {
      smokeAlarmActive = true;       // Arm the non-blocking buzzer (unchanged)
      Serial.println("ALERT: Smoke detected: " + String(lastGasReading));
    }

    updateLCD();
  }

  // ── E. RFID scan ─────────────────────────────────────────────────────────
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
}
