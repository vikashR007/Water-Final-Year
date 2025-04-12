#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>
#define FLOW_SENSOR1_PIN 19  // GPIO for Flow Sensor 1
#define FLOW_SENSOR2_PIN 18  // GPIO for Flow Sensor 2
#define TURBIDITY1_SENSOR_PIN 34  // Analog pin for Turbidity Sensor
#define TURBIDITY2_SENSOR_PIN 35  // Analog pin for Turbidity Sensor
#define VALVE1 26       // Valve 1
#define VALVE2 27       // Valve 2
#define BUZZER 23       // Valve 2
volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;
HardwareSerial sim800l(1); // Use UART1 (Serial1) for SIM800L
#define RX2 17  // ESP32 RX (Connect to SIM800L TX)
#define TX2 16  // ESP32 TX (Connect to SIM800L RX)
String phoneNumber = "+916379894651"; 

float flowRate1 = 0;
float flowRate2 = 0 ;
float turbidity1 = 0;
float turbidity2 = 0;
float totalVolume1 = 0;
float totalVolume2 = 0;
unsigned long lastTime = 0;

// Initialize LCD with I2C address 0x27 (Change to 0x3F if needed)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void IRAM_ATTR pulseCounter1() {
  pulseCount1++;
}

void IRAM_ATTR pulseCounter2() {
  pulseCount2++;
}

void setup() {
  Serial.begin(115200);
  sim800l.begin(9600, SERIAL_8N1, RX2, TX2);
  Wire.begin(21, 22); // SDA -> GPIO 21, SCL -> GPIO 22
  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("WATER QUALITY");
  lcd.setCursor(0, 1);
  lcd.print("MONITORING SYSTEM");
  pinMode(FLOW_SENSOR1_PIN, INPUT_PULLUP);
  pinMode(FLOW_SENSOR2_PIN, INPUT_PULLUP);
  pinMode(TURBIDITY1_SENSOR_PIN, INPUT);
  pinMode(TURBIDITY2_SENSOR_PIN, INPUT);
  pinMode(VALVE1, OUTPUT);
  pinMode(VALVE2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(VALVE1,HIGH);
  digitalWrite(VALVE2,HIGH);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR1_PIN), pulseCounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR2_PIN), pulseCounter2, RISING);
  sendCommand("AT"); // Check if SIM800L is responding
  sendCommand("AT+CMGF=1"); // Set SMS mode to Text
  sendCommand("AT+CLIP=1"); // Enable caller ID
  delay(2000);
  lcd.clear();
}

void loop() {
  
  if (millis() - lastTime >= 1000) {  // Update every second
    detachInterrupt(FLOW_SENSOR1_PIN);
    detachInterrupt(FLOW_SENSOR2_PIN);
    // Flow Rate Calculation (L/min)
    flowRate1 = (pulseCount1 / 7.5);  
    flowRate2 = (pulseCount2 / 7.5);  
    
    // Update total volume
    totalVolume1 += flowRate1 / 60;  // Convert L/min to L/sec
    totalVolume2 += flowRate2 / 60;
        
    // Read Turbidity Sensor (0 - 3.3V mapped to 0-4095)
    int turbidityValue1 = analogRead(TURBIDITY1_SENSOR_PIN);
    turbidity1 = (turbidityValue1 / 4095.0) * 100.0; // Convert to percentage (0-100%)
    // Read Turbidity Sensor (0 - 3.3V mapped to 0-4095)
    int turbidityValue2 = analogRead(TURBIDITY2_SENSOR_PIN);
    turbidity2 = (turbidityValue2 / 4095.0) * 100.0; // Convert to percentage (0-100%)
    // Display values on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("V1:");
    lcd.print(totalVolume1);
    lcd.setCursor(9, 0);
    lcd.print("V2:");
    lcd.print(totalVolume2);
    lcd.setCursor(0, 1);
    lcd.print("T1:");
    lcd.print(turbidity1);
    lcd.setCursor(9, 1);
    lcd.print("T2:");
    lcd.print(turbidity2);
    delay(100);
    if((turbidity1<45)&&(turbidity2<45)){
    digitalWrite(VALVE1,LOW);
    digitalWrite(VALVE2,LOW); 
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WATER QUALITY LOW");
    lcd.setCursor(0, 1);
    lcd.print("BOTH VALVE OFF");
    digitalWrite(BUZZER,HIGH);
    checkWaterQuality();
    digitalWrite(BUZZER,LOW);
    lcd.clear();
    }
    if((turbidity1<45)&&(turbidity2>45)){
    digitalWrite(VALVE1,LOW);
    digitalWrite(VALVE2,HIGH); 
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WATER QUALITY LOW");
    lcd.setCursor(3, 1);
    lcd.print("VALVE1 OFF");
    digitalWrite(BUZZER,HIGH);
    checkWaterQuality();
    digitalWrite(BUZZER,LOW);
    lcd.clear();
    }
    if((turbidity1>45)&&(turbidity2<45)){
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,LOW); 
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WATER QUALITY LOW");
    lcd.setCursor(3, 1);
    lcd.print("VALVE2 OFF");
    digitalWrite(BUZZER,HIGH);
    checkWaterQuality();
    digitalWrite(BUZZER,LOW);
    lcd.clear();
    }
    if((turbidity1>45)&&(turbidity2>45)){
    digitalWrite(VALVE1,HIGH);
    digitalWrite(VALVE2,HIGH); 
    }
    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR1_PIN), pulseCounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR2_PIN), pulseCounter2, RISING);
  }
}


// Function to send an AT command and print response
void sendCommand(String command) {
    Serial.print("Sending: "); Serial.println(command);
    sim800l.println(command);
    delay(1000); // Wait for response
    while (sim800l.available()) {
        Serial.write(sim800l.read());
    }
}

// Function to send an SMS
void sendSMS(String number, String message) {
    sendCommand("AT+CMGS=\"" + number + "\""); // Set recipient number
    delay(1000);
    sim800l.print(message); // Send message
    delay(1000);
    sim800l.write(26); // Send Ctrl+Z to indicate end of message
    Serial.println("SMS Sent!");
}

// Function to make a call
void makeCall(String number) {
    sendCommand("ATD" + number + ";"); // Dial number
    Serial.println("Calling...");
    delay(30000);
}

void checkWaterQuality() {
    String message = "";
    if (turbidity1 > 45 || turbidity2 > 45) {  // Assuming >35% is low quality
        message = "ALERT: Water Quality Low!\n";
        message += "T1: " + String(turbidity1)   + "%\n";
        message += "T2: " + String(turbidity2)   + "%\n";
        message += "V1: " + String(totalVolume1) + " L\n";
        message += "V2: " + String(totalVolume2) + " L ";
        sendSMS(phoneNumber, message);
    }
}
