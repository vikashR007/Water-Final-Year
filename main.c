#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define FLOW_SENSOR1_PIN 18  // GPIO for Flow Sensor 1
#define FLOW_SENSOR2_PIN 19  // GPIO for Flow Sensor 2
#define TURBIDITY_SENSOR_PIN 34  // Analog pin for Turbidity Sensor

volatile int pulseCount1 = 0;
volatile int pulseCount2 = 0;

float flowRate1 = 0.0;
float flowRate2 = 0.0;
float turbidity = 0.0;
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

  // Initialize LCD
  Wire.begin(21, 22); // SDA -> GPIO 21, SCL -> GPIO 22
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Water Monitoring");

  // Flow sensor setup
  pinMode(FLOW_SENSOR1_PIN, INPUT_PULLUP);
  pinMode(FLOW_SENSOR2_PIN, INPUT_PULLUP);
  pinMode(TURBIDITY_SENSOR_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR1_PIN), pulseCounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR2_PIN), pulseCounter2, RISING);
}

void loop() {
  if (millis() - lastTime >= 1000) {  // Update every second
    detachInterrupt(FLOW_SENSOR1_PIN);
    detachInterrupt(FLOW_SENSOR2_PIN);

    // Flow Rate Calculation (L/min)
    flowRate1 = (pulseCount1 / 7.5);  
    flowRate2 = (pulseCount2 / 7.5);  

    // Read Turbidity Sensor (0 - 3.3V mapped to 0-4095)
    int turbidityValue = analogRead(TURBIDITY_SENSOR_PIN);
    turbidity = (turbidityValue / 4095.0) * 100.0; // Convert to percentage (0-100%)

    Serial.print("Flow Rate 1: ");
    Serial.print(flowRate1);
    Serial.println(" L/min");

    Serial.print("Flow Rate 2: ");
    Serial.print(flowRate2);
    Serial.println(" L/min");

    Serial.print("Turbidity: ");
    Serial.print(turbidity);
    Serial.println(" %");

    // Display values on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("F1:");
    lcd.print(flowRate1);
    lcd.setCursor(9, 0);
    lcd.print("F2:");
    lcd.print(flowRate2);
+    lcd.setCursor(0, 1);
    lcd.print("Turb:");
    lcd.print(turbidity);
    lcd.print(" %");

    pulseCount1 = 0;
    pulseCount2 = 0;
    lastTime = millis();

    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR1_PIN), pulseCounter1, RISING);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR2_PIN), pulseCounter2, RISING);
  }
}