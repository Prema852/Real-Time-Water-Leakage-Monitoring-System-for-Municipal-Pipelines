// AquaGuard - Leak detection with two flow sensors (NORMAL / CRITICAL only)
// Flow Sensor 1 (inlet)  -> GPIO 26
// Flow Sensor 2 (outlet) -> GPIO 27
// LCD SDA                -> GPIO 16
// LCD SCL                -> GPIO 17
// White LED -> GPIO 32 (Normal)     Red LED -> GPIO 25 (Critical)
// Buzzer (active)        -> GPIO 21
// Relay IN (valve)       -> GPIO 22
// Reset button           -> GPIO 4 and GND
// Each LED: GPIO -> 220R resistor -> LED anode, cathode -> GND

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define FLOW1_PIN 26
#define FLOW2_PIN 27
#define SDA_PIN   16
#define SCL_PIN   17

#define LED_WHITE 32   // normal
#define LED_RED   25   // critical

#define BUZZER_PIN 21
#define VALVE_PIN  22
#define RESET_PIN  4

// Most relay modules are active-LOW (LOW = relay ON). Set false if yours is active-HIGH.
#define RELAY_ACTIVE_LOW true

#define LCD_ADDR  0x27   // change to 0x3F if the LCD stays blank
#define LCD_COLS  16
#define LCD_ROWS  2

// ---------------- SETTINGS (tune these) ----------------
// Sensor 2 reads lower than sensor 1 even with a healthy pipe (your log showed
// S2 = about 0.77 x S1). This factor corrects it: S2_CAL = S1 / S2 on a healthy pipe.
#define S2_CAL         1.30

// Loss % = (difference between S1 and corrected S2) / (higher flow) x 100
//   below CRIT_PCT  -> NORMAL
//   CRIT_PCT or more -> CRITICAL (valve closes)
#define CRIT_PCT       15.0

#define MIN_FLOW        0.5    // L/min: below this there is no real flow, so no leak check
#define CRIT_CONFIRM    3      // seconds CRITICAL must last before the valve closes
#define STARTUP_IGNORE_MS 6000 // ignore readings for the first 6 s after power-on
#define AVG_SECONDS     5      // readings are averaged over the last 5 seconds

// Ignore noise pulses closer together than this (real pulses at 30 L/min are ~4.4 ms apart)
#define MIN_PULSE_GAP_US 3000
// -------------------------------------------------------

enum Status { NORMAL, CRITICAL };

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

volatile unsigned long pulseCount1 = 0;
volatile unsigned long pulseCount2 = 0;
volatile unsigned long lastPulseUs1 = 0;
volatile unsigned long lastPulseUs2 = 0;

unsigned long lastTime = 0;
Status prevStatus = NORMAL;
Status currentStatus = NORMAL;
bool criticalLatched = false;   // stays true after a leak until the reset button is pressed
int critCount = 0;

float hist1[AVG_SECONDS], hist2[AVG_SECONDS];   // pulses per second history
int histIdx = 0, histFilled = 0;

float flowRate1 = 0;
float flowRate2 = 0;   // corrected with S2_CAL

void IRAM_ATTR flow1Pulse() {
  unsigned long now = micros();
  if (now - lastPulseUs1 >= MIN_PULSE_GAP_US) {
    pulseCount1++;
    lastPulseUs1 = now;
  }
}

void IRAM_ATTR flow2Pulse() {
  unsigned long now = micros();
  if (now - lastPulseUs2 >= MIN_PULSE_GAP_US) {
    pulseCount2++;
    lastPulseUs2 = now;
  }
}

void setStatusLeds(Status s) {
  digitalWrite(LED_WHITE, s == NORMAL   ? HIGH : LOW);
  digitalWrite(LED_RED,   s == CRITICAL ? HIGH : LOW);
}

// Critical: continuous buzzer. Normal: silent.
void updateBuzzer() {
  digitalWrite(BUZZER_PIN, currentStatus == CRITICAL ? HIGH : LOW);
}

// Valve is normally-closed: relay ON (energised) = valve OPEN = water flows.
// CRITICAL -> relay OFF -> valve CLOSES.
void setValve(bool open) {
  if (RELAY_ACTIVE_LOW) {
    if (open) {
      pinMode(VALVE_PIN, OUTPUT);
      digitalWrite(VALVE_PIN, LOW);    // relay ON  -> valve OPEN
    } else {
      // Release the pin instead of driving HIGH: a 3.3V HIGH often can't
      // fully switch off a 5V relay module.
      pinMode(VALVE_PIN, INPUT);       // relay OFF -> valve CLOSED
    }
  } else {
    pinMode(VALVE_PIN, OUTPUT);
    digitalWrite(VALVE_PIN, open ? HIGH : LOW);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(FLOW1_PIN, INPUT_PULLUP);
  pinMode(FLOW2_PIN, INPUT_PULLUP);

  pinMode(LED_WHITE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);
  digitalWrite(BUZZER_PIN, LOW);
  setValve(true);   // valve open at start
  setStatusLeds(NORMAL);

  attachInterrupt(digitalPinToInterrupt(FLOW1_PIN), flow1Pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW2_PIN), flow2Pulse, RISING);

  // I2C on custom pins
  Wire.begin(SDA_PIN, SCL_PIN);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("AquaGuard");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(1500);
  lcd.clear();

  Serial.println("AquaGuard Leak Detector");
  Serial.println("--------------------------");

  lastTime = millis();
}

void loop() {

  updateBuzzer();

  // Reset button clears the CRITICAL latch
  if (digitalRead(RESET_PIN) == LOW) {
    criticalLatched = false;
    critCount = 0;
  }

  unsigned long elapsed = millis() - lastTime;
  if (elapsed >= 1000) {

    // Copy pulse counts
    noInterrupts();
    unsigned long pulses1 = pulseCount1;
    unsigned long pulses2 = pulseCount2;
    pulseCount1 = 0;
    pulseCount2 = 0;
    interrupts();

    // Pulses per second, using the real elapsed time
    hist1[histIdx] = pulses1 * 1000.0 / elapsed;
    hist2[histIdx] = pulses2 * 1000.0 / elapsed;
    histIdx = (histIdx + 1) % AVG_SECONDS;
    if (histFilled < AVG_SECONDS) histFilled++;

    float sum1 = 0, sum2 = 0;
    for (int i = 0; i < histFilled; i++) { sum1 += hist1[i]; sum2 += hist2[i]; }

    // YF-S201: Flow rate (L/min) = frequency / 7.5
    flowRate1 = (sum1 / histFilled) / 7.5;
    float flow2Raw = (sum2 / histFilled) / 7.5;
    flowRate2 = flow2Raw * S2_CAL;

    // Leak check: percentage loss between the two sensors
    float diff   = fabs(flowRate1 - flowRate2);
    float bigger = (flowRate1 > flowRate2) ? flowRate1 : flowRate2;
    float lossPct = (bigger >= MIN_FLOW) ? (diff / bigger * 100.0) : 0.0;

    bool leak = (millis() > STARTUP_IGNORE_MS) && (lossPct >= CRIT_PCT);

    // Leak must last CRIT_CONFIRM seconds in a row before the valve closes
    if (leak) critCount++;
    else      critCount = 0;
    if (critCount >= CRIT_CONFIRM) criticalLatched = true;

    Status status = criticalLatched ? CRITICAL : NORMAL;

    // Once latched, the valve stays closed. (Closing it drops both flows to 0,
    // which would otherwise look NORMAL and reopen the valve.)
    setValve(!criticalLatched);

    setStatusLeds(status);
    currentStatus = status;

    // Re-initialise the LCD on status change (and every cycle while CRITICAL)
    // so electrical glitches can never leave it showing garbage.
    if (status != prevStatus || status == CRITICAL) {
      delay(20);
      lcd.init();
      lcd.backlight();
      prevStatus = status;
    }

    const char* statusTxt = (status == NORMAL) ? "NORM" : "CRIT";

    // Serial output
    Serial.print("Raw pulses/s: ");  Serial.print(pulses1);  Serial.print(" , ");  Serial.println(pulses2);
    Serial.print("Flow 1: ");           Serial.print(flowRate1);  Serial.println(" L/min");
    Serial.print("Flow 2 (raw): ");     Serial.print(flow2Raw);   Serial.println(" L/min");
    Serial.print("Flow 2 (corrected): "); Serial.print(flowRate2); Serial.println(" L/min");
    Serial.print("Loss: ");  Serial.print(lossPct, 1);  Serial.println(" %");
    Serial.print("Status: ");
    Serial.println(status == NORMAL ? "NORMAL" : "CRITICAL");
    if (criticalLatched) Serial.println("LATCHED: valve closed. Press reset button (GPIO 4 to GND) or the EN button to clear.");
    Serial.println("--------------------------");

    // LCD output (exactly 16 chars per row)
    char line[17];

    // Row 0: "S1: 1.23L/m NORM"
    snprintf(line, sizeof(line), "S1:%5.2fL/m %4s", flowRate1, statusTxt);
    lcd.setCursor(0, 0);
    lcd.print(line);

    // Row 1: "S2: 1.10L/m  8%"  (loss percentage)
    snprintf(line, sizeof(line), "S2:%5.2fL/m %3.0f%%", flowRate2, lossPct);
    lcd.setCursor(0, 1);
    lcd.print(line);

    lastTime = millis();
  }
}
