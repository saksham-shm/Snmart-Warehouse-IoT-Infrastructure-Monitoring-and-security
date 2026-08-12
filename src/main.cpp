#include <DHT22.h>

#include <LiquidCrystal_I2C.h>
#include<Wire.h>
#include<SPI.h>
#include<MFRC522.h>
#include<ESP32Servo.h>
// #include<DHT.h>


LiquidCrystal_I2C lcd(0x27, 16, 2);

// RFID pins
#define ssPin 5
#define rstPin 4
#define dhtOne 33
#define dhtTwo 25
#define pirPIN 32


MFRC522 rfid(ssPin,rstPin);
DHT22 dht1(dhtOne);   
DHT22 dht2(dhtTwo);

// Relay Pins, Buzzer pins, ServoPins
const int relay1 = 27;   
const int relay2 = 14;
const int buzzer=12;
const int servopin=13;
const int gasPin=34;
const int gasThreshold1=400;
const int gasThreshold2=800;
Servo servo1;

//  08 2F BF 33
String cardsAccepted[] = {"83 AC CD 27","01 AC 03 04","55 66 77 88"};
String employeeName[] = {"Employee 1","Employee 2","Employee 3"};

unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 2000;



void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();
    // SDA = Serial Data -> 23 SCL = Serial Clock ->22
  //Initialize the lcd
  Wire.begin(21,22);  // LCD Screen 1    
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0,0);
  lcd.print("hello");
  lcd.setCursor(0,1);
  lcd.print("Begin");
  // Relay
  pinMode(relay1, OUTPUT);
  pinMode(relay2,OUTPUT);
  digitalWrite(relay1, LOW);   // ← add this
  digitalWrite(relay2, LOW);  
  // digitalWrite(relay1,LOW);
  // Buzzer
  pinMode(buzzer,OUTPUT);
  // RFID

  servo1.attach(servopin);
  servo1.write(0);

  pinMode(gasPin, INPUT);
  Serial.println("MFRC522 Ready");

  pinMode(pirPIN, INPUT);
}

// Function to write into lcd Screen with no return value
void writetoLCD(String line1, String line2){
  lcd.clear();
// Write new information
  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
}

void buzzerAccept(){
  for (int i=0;i<2;i++){
    tone(buzzer, 1800);
    delay(100);
    noTone(buzzer);
    delay(100);
  }
  delay(500);
}

void buzzerAlert (int time){
  for ( int i=0;i<time;i++){
    // plays the buzzer the required times.
    tone(buzzer,1800);
    delay(150);
    tone(buzzer,1800);
    delay(150);
    noTone(buzzer);
    delay(150);
  }
}

void relayControl_Access(boolean value){
  if (value == true){
    digitalWrite(relay1, HIGH);
   }
  else{
    digitalWrite(relay1, LOW);
  }
}

void relayControl_light(boolean value){
  if (value){
    digitalWrite(relay2, HIGH);
  }
  else{
    digitalWrite(relay2, LOW);
  }

}


void RFIDAccepted(String accessedby){
  writetoLCD("Accessed by: ",accessedby);
  buzzerAccept();
  // relayControl_Access(true);
  // servo1.attach(servopin);
  servo1.write(90);
  delay(3000);
  servo1.write(0);
  // relayControl_Access(false);
  
}

void RFIDDenied(){
  writetoLCD("Access Denied","Try Again");
  buzzerAlert(5);
  // relayControl_Access(false);
  servo1.write(0);
 

}

void readDHT(float dhtData[]){

  float humidOne = dht1.getHumidity();
  float humidTwo = dht2.getHumidity();
  float tempOne  = dht1.getTemperature();
  float tempTwo  = dht2.getTemperature();



  if (isnan(humidOne) || isnan(tempOne)) {
    Serial.println("DHT1 read failed");
    humidOne = 0; tempOne = 0;
  }
  if (isnan(humidTwo) || isnan(tempTwo)) {
    Serial.println("DHT2 read failed");  // ← This tells you which one fails
    humidTwo = 0; tempTwo = 0;
  }

  // This library doesn't support Fahrenheit directly; convert manually
  float ftempOne = tempOne * 9.0 / 5.0 + 32.0;
  float ftempTwo = tempTwo * 9.0 / 5.0 + 32.0;

  dhtData[0] = humidOne;
  dhtData[1] = humidTwo;
  dhtData[2] = tempOne;
  dhtData[3] = tempTwo;
  dhtData[4] = ftempOne;
  dhtData[5] = ftempTwo;

}

void gasMonitor(int reading){

  if(reading > gasThreshold2){
    writetoLCD("Gas Reading:"+ String(reading),"Highly toxic");
    delay(500);
  }
  else if (reading > gasThreshold1){
    writetoLCD("Gas Reading:"+String(reading),"Toxis gas");
    delay(500);
  }
  else{
    return;
  }
}

void loop() {
  // Relay test
  // delay(2000);
  // digitalWrite(relay1, HIGH);
  // delay(2000);
  // digitalWrite(relay1, LOW);

  float reading[6];
  if (millis() - lastSensorRead >= sensorInterval ){

    lastSensorRead = millis();
    readDHT(reading);
    String output1 = "H1:" + String(reading[0],1) + "%" +"T1:"+String(reading[2],1)+"°C" ;
    String output2 = "H2:" + String(reading[1],1) + "%"+"T2:"+String(reading[3],1)+"°C";
    writetoLCD(output1, output2);
  }

  int gasReading = analogRead(gasPin);

  String gasvalue = String(gasReading);
  Serial.println("Gas Value" + gasvalue);
  delay(2000);
  if ( gasReading >= 400){
    gasMonitor(gasReading);
    // delay(2000);
  }
  // writetoLCD(gasValue,"Hello");
  // delay(10000);
  //  if (gasReading > gasThreshold1){
  //     // gasMonitor(gasReading);
  //     writetoLCD("Gas reading",String(gasReading));
  // }

  // int pirRead = digitalRead(pirPIN);
  // if (pirRead == HIGH){
  //   // Serial.println("Motion Yes");
  //   relayControl_light(true);
  // }
  // else{
  //   relayControl_light(false);
  // }
  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

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

  for (int i=0;i<NumCards;i++){

    if (readUID == cardsAccepted[i]){
      RFIDAccepted(employeeName[i]);
      allowed = true;
      break;
    }
  }

  if (!allowed){
    RFIDDenied();
  }

  Serial.println("End of line");
  rfid.PICC_HaltA();

}
