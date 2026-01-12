// FlameEye-X FINAL with Blynk + LCD + Servo Lock + Manual Valve Open
// Hardware: MQ2, IR Flame sensor, DS18B20, Relay (12V fan), Buzzer, 3 LEDs, Servo valve, 16x2 LCD (JHD162A)

// ---------- BLYNK + WiFi ----------
#define BLYNK_TEMPLATE_ID "Your template id"
#define BLYNK_TEMPLATE_NAME "FlameEyeX"
#define BLYNK_AUTH_TOKEN "Your template token"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Your Wifi Name";      // your WiFi
char pass[] = "Your Wifi password";           // your WiFi pass

BlynkTimer timer;

// ---------- LIBRARIES ----------
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>
#include <LiquidCrystal.h>
#include <math.h>

// ---------- PIN DEFINITIONS ----------
// Sensors
const int gasPin      = 34;  // MQ2 analog output (A0 -> GPIO34)
const int flamePin    = 32;  // Flame sensor D0 (digital, LOW = fire)
const int ds18b20Pin  = 4;   // DS18B20 data (D4)

// Actuators
const int relayPin     = 5;  // Relay input (fan) - board is wired so current logic works
const int buzzerPin    = 2;  // Buzzer TMB12A05
const int ledSafePin   = 27; // Green LED (SAFE)
const int ledPrePin    = 25; // Yellow LED (PRE)
const int ledDangerPin = 33; // Red LED (EMERGENCY)
const int servoPin     = 13; // Servo signal (demo gas valve)

// LCD pins (JHD162A, 4-bit mode)
const int lcdRs = 16;
const int lcdEn = 17;
const int lcdD4 = 22;
const int lcdD5 = 21;
const int lcdD6 = 19;
const int lcdD7 = 23;

// ---------- BLYNK VIRTUAL PINS ----------
// V0 = Risk (%)
// V1 = State text (SAFE / PRE / EMG / WARM)
// V2 = Gas ADC
// V3 = Reset emergency (switch button)
// V4 = SAFE LED
// V5 = PRE LED
// V6 = EMG LED
// V7 = Manual "Open Valve" command button  <-- NEW
// V8 = Valve status text (OPEN/PARTIAL/CLOSED)  <-- NEW

// ---------- LCD OBJECT ----------
LiquidCrystal lcd(lcdRs, lcdEn, lcdD4, lcdD5, lcdD6, lcdD7);

// ---------- DS18B20 SETUP ----------
OneWire oneWire(ds18b20Pin);
DallasTemperature tempSensor(&oneWire);

// ---------- SERVO ----------
Servo valveServo;
bool servoAttached = false;

// ---------- MODEL PARAMETERS ----------
// P = 1 / (1 + exp( -(α*Ctilde + β*Ttilde + η*Ipos + γ*Dtilde - δ*V + b) ))

const float kAlpha  = 1.0f;   // weight for Ctilde
const float kBeta   = 0.5f;   // weight for Ttilde
const float kEta    = 0.25f;  // interaction weight Ipos
const float kGammaW = 0.25f;  // weight for Dtilde (renamed from gamma)
const float kDelta  = 0.35f;  // ventilation weight
const float kBias   = -2.2f;  // baseline bias

// Thresholds
const float P_PRE   = 0.60f;  // preemptive threshold
const float P_EMG   = 0.92f;  // emergency threshold

// Normalization & duration
const float gasSigma    = 80.0f;  // spread for gas z-score (tunable)
const float dTdtSigma   = 0.5f;   // spread for dT/dt
const float lnD_Sigma   = 1.0f;   // spread for ln(duration)
const float D50         = 30.0f;  // half-saturation duration for Deff

// Duration threshold in ADC units (~ "high gas")
const float GAS_THRESH_DURATION = 700.0f; // start counting duration when gasEma above this

// Clipping for z-scores
const float C_CLIP_MIN = -3.0f, C_CLIP_MAX = 6.0f;
const float T_CLIP_MIN = -4.0f, T_CLIP_MAX = 8.0f;
const float D_CLIP_MIN = -3.0f, D_CLIP_MAX = 6.0f;

// EMA
const float GAS_EMA_ALPHA  = 0.2f;  // for gas
const float TEMP_EMA_ALPHA = 0.2f;  // for temp

// Baseline adaptation
const float BASELINE_GAS_MIN = 500.0f;   // manually chosen safe kitchen baseline
float baselineGas   = BASELINE_GAS_MIN;  // will only go UP from here
float baselineTemp  = 25.0f;
const float BASELINE_ALPHA = 0.001f;     // very small learning rate

// Warmup
const unsigned long WARMUP_MS = 120000UL; // 120 s
unsigned long startMs = 0;

// --------- STATE VARIABLES ----------
float gasEma = 0.0f;
float tempEma = 25.0f;
float lastTempEma = 25.0f;
float dTdt = 0.0f;

float durationExp = 0.0f;   // exposure duration (seconds)
unsigned long lastSampleMs = 0;
unsigned long lastPrintMs  = 0;

bool fanOn         = false;
bool valvePartial  = false;
bool valveClosed   = false;

// EMERGENCY latch + valve lock
bool emergencyLatched      = false; // once true, EMG until reset in Blynk
bool lastEmergencyLatched  = false;
bool valveLock             = false; // once true, code never auto-opens valve

// ---------- HELPERS ----------
float fmax2(float a, float b) { return (a > b) ? a : b; }
float fconstrain(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

// Servo helpers
void setValveOpen() {
  if (!servoAttached) return;
  if (valveLock) return;     // safety: if locked, never auto-open
  valveServo.write(0);       // fully open
  valvePartial = false;
  valveClosed  = false;
}
void setValvePartial() {
  if (!servoAttached) return;
  if (valveLock) return;
  valveServo.write(45);      // half close
  valvePartial = true;
  valveClosed  = false;
}
void setValveClosed() {
  if (!servoAttached) return;
  valveServo.write(90);      // fully closed (demo)
  valvePartial = false;
  valveClosed  = true;
}

// Fan + buzzer helpers
void setFan(bool on) {
  // Board is wired so this logic keeps your earlier behaviour
  // (relayPin LOW = fan ON, HIGH = OFF) — don't touch if hardware OK.
  digitalWrite(relayPin, on ? LOW : HIGH);
  fanOn = on;
}

void beepPatternPreemptive() {
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
  delay(150);
}

void beepPatternEmergency() {
  for (int i = 0; i < 8; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    delay(80);
  }
}

// LCD helper: print state + P on first line, gas on second
void lcdShowState(const char* stateLabel, float P, float gas) {
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(stateLabel);
  lcd.print(" P=");
  lcd.print(P, 2);

  lcd.setCursor(0, 1);
  lcd.print("Gas=");
  lcd.print((int)gas);
  lcd.print(" ADC    ");
}

// ---------- BLYNK VIRTUAL HANDLERS ----------

// V3 = Reset Emergency (switch button)
BLYNK_WRITE(V3) {
  int v = param.asInt();
  if (v == 1) {
    emergencyLatched = false;   // bot can go back to SAFE/PRE logic
    // valveLock stays TRUE -> user must manually open gas (via physical valve or servo button)
    digitalWrite(buzzerPin, LOW);
    durationExp = 0.0f;
    Blynk.virtualWrite(V3, 0);  // switch back to OFF
  }
}

// V7 = Manual "Open Valve" command  <-- NEW
BLYNK_WRITE(V7) {
  int v = param.asInt();
  if (v == 1) {
    // Only allow servo-open when NOT in emergency
    if (!emergencyLatched) {
      valveLock = false;   // unlock in software
      setValveOpen(); 
      Blynk.virtualWrite(V7, 1);     
    }
    // always release button back to OFF
    Blynk.virtualWrite(V7, 0);
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(500);

  // WiFi + Blynk
  Blynk.begin(auth, ssid, pass);

  startMs = millis();

  pinMode(gasPin, INPUT);
  pinMode(flamePin, INPUT_PULLUP); // D0 often has pull-up, LOW = flame

  pinMode(relayPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledSafePin, OUTPUT);
  pinMode(ledPrePin, OUTPUT);
  pinMode(ledDangerPin, OUTPUT);

  setFan(false);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledSafePin, LOW);
  digitalWrite(ledPrePin, LOW);
  digitalWrite(ledDangerPin, LOW);

  // DS18B20
  tempSensor.begin();

  // Servo
  valveServo.attach(servoPin);
  servoAttached = true;
  setValveOpen();

  // LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("FlameEye-X");
  lcd.setCursor(0, 1);
  lcd.print("Warming up...");

  // Initial baselines
  baselineGas  = BASELINE_GAS_MIN;
  baselineTemp = 25.0f;

  int rawGasInit = analogRead(gasPin);
  gasEma       = rawGasInit;
  tempEma      = baselineTemp;
  lastTempEma  = tempEma;

  lastSampleMs = millis();
  lastPrintMs  = millis();

  Serial.println(F("FlameEye-X starting..."));
  Serial.println(F("time_ms  STATE  rawGas  gasEma  flameRaw  tempC  tempEma  dTdt  duration  Ctilde  Ttilde  Dtilde  Ipos  P  fanOn  vPart  vClos  flameDet  baselineGas  baselineTemp  emgLatched  valveLock"));
}

// ---------- LOOP ----------
void loop() {
  Blynk.run();
  timer.run();

  unsigned long nowMs = millis();
  float dtSec = (nowMs - lastSampleMs) / 1000.0f;
  if (dtSec < 1.0f) {
    return; // ~1 Hz sensing
  }
  lastSampleMs = nowMs;

  bool inWarmup = (nowMs - startMs < WARMUP_MS);

  // ---------- 1) Read sensors ----------
  int rawGas   = analogRead(gasPin);
  int flameRaw = digitalRead(flamePin);

  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);
  bool tempValid = true;
  if (tempC < -40.0f || tempC > 125.0f || tempC == -127.0f) {
    tempValid = false;
    tempC = tempEma;
  }

  // ---------- 2) EMA for gas ----------
  gasEma = GAS_EMA_ALPHA * rawGas + (1.0f - GAS_EMA_ALPHA) * gasEma;

  // ---------- 3) EMA for temperature ----------
  if (tempValid) {
    lastTempEma = tempEma;
    tempEma = TEMP_EMA_ALPHA * tempC + (1.0f - TEMP_EMA_ALPHA) * tempEma;
  }

  // ---------- 4) dT/dt ----------
  dTdt = (tempEma - lastTempEma) / fmax2(dtSec, 1e-6f);

  // ---------- 5) Duration exposure ----------
  if (gasEma >= GAS_THRESH_DURATION) {
    durationExp += dtSec;
  } else {
    durationExp -= 0.5f * dtSec;
    if (durationExp < 0.0f) durationExp = 0.0f;
  }

  // ---------- 6) Normalize ----------
  float Ctilde = (gasEma - baselineGas) / (gasSigma + 1e-6f);
  if (Ctilde < C_CLIP_MIN) Ctilde = C_CLIP_MIN;
  if (Ctilde > C_CLIP_MAX) Ctilde = C_CLIP_MAX;

  float Ttilde = (dTdt - 0.0f) / (dTdtSigma + 1e-6f);
  if (Ttilde < T_CLIP_MIN) Ttilde = T_CLIP_MIN;
  if (Ttilde > T_CLIP_MAX) Ttilde = T_CLIP_MAX;

  float Deff   = durationExp / (1.0f + durationExp / D50);
  float lnD    = logf(1.0f + Deff);
  float Dtilde = (lnD - 0.0f) / (lnD_Sigma + 1e-6f);
  if (Dtilde < D_CLIP_MIN) Dtilde = D_CLIP_MIN;
  if (Dtilde > D_CLIP_MAX) Dtilde = D_CLIP_MAX;

  float Cpos = fmax2(0.0f, Ctilde);
  float Tpos = fmax2(0.0f, Ttilde);
  float Ipos = Cpos * Tpos;

  float V = fanOn ? 1.0f : 0.0f;

  // ---------- 7) Risk score & probability ----------
  float S = kAlpha * Ctilde
          + kBeta  * Ttilde
          + kEta   * Ipos
          + kGammaW * Dtilde
          - kDelta * V
          + kBias;
  float P = 1.0f / (1.0f + expf(-S));

  // ---------- 8) Flame detection ----------
  bool flameDet = (flameRaw == LOW); // LOW = fire

  // ---------- 9) Emergency latch ----------
  if (!inWarmup) {
    if (flameDet || P >= P_EMG) {
      emergencyLatched = true;
    }
  }

  // ---------- 10) Decide state ----------
  enum State { STATE_WARM, STATE_SAFE, STATE_PRE, STATE_EMG };
  State state;

  if (inWarmup) {
    state = STATE_WARM;
  } else if (emergencyLatched) {
    state = STATE_EMG;
  } else if (P >= P_PRE) {
    state = STATE_PRE;
  } else {
    state = STATE_SAFE;
  }

  // ---------- 11) Actions ----------
  switch (state) {
    case STATE_WARM:
      setFan(false);
      setValveOpen();  // before any EMG, lock = false
      digitalWrite(ledDangerPin, LOW);
      digitalWrite(ledPrePin, LOW);
      digitalWrite(ledSafePin, LOW);
      digitalWrite(buzzerPin, LOW);
      break;

    case STATE_SAFE:
      setFan(false);
      setValveOpen();
      digitalWrite(ledDangerPin, LOW);
      digitalWrite(ledPrePin, LOW);
      digitalWrite(ledSafePin, HIGH);
      digitalWrite(buzzerPin, LOW);
      break;

    case STATE_PRE:
      setFan(true);
      setValvePartial();
      digitalWrite(ledDangerPin, LOW);
      digitalWrite(ledPrePin, HIGH);
      digitalWrite(ledSafePin, LOW);
      beepPatternPreemptive();
      break;

    case STATE_EMG:
      setFan(true);
      setValveClosed();
      valveLock = true;   // from now on, code never auto-opens valve
      digitalWrite(ledDangerPin, HIGH);
      digitalWrite(ledPrePin, LOW);
      digitalWrite(ledSafePin, LOW);
      beepPatternEmergency();
      break;
  }

  // ---------- 12) Adaptive baseline ----------
  if (state == STATE_SAFE && !flameDet && durationExp < 5.0f) {
    float candidate = (1.0f - BASELINE_ALPHA) * baselineGas + BASELINE_ALPHA * gasEma;
    if (candidate > baselineGas) baselineGas = candidate;
    if (baselineGas < BASELINE_GAS_MIN) baselineGas = BASELINE_GAS_MIN;

    baselineTemp = (1.0f - BASELINE_ALPHA) * baselineTemp + BASELINE_ALPHA * tempEma;
  }

  // ---------- 13) LCD update ----------
  if (state == STATE_WARM) {
    unsigned long remain = (WARMUP_MS - (nowMs - startMs)) / 1000;
    if ((long)remain < 0) remain = 0;
    lcd.setCursor(0, 0);
    lcd.print("WARMUP ");
    lcd.print((int)remain);
    lcd.print("s     ");
    lcd.setCursor(0, 1);
    lcd.print("Gas=");
    lcd.print((int)gasEma);
    lcd.print(" ADC   ");
  } else if (state == STATE_SAFE) {
    lcdShowState("SAFE", P, gasEma);
  } else if (state == STATE_PRE) {
    lcdShowState("PRE ", P, gasEma);
  } else {
    lcdShowState("EMG ", P, gasEma);
  }

  // ---------- 14) Blynk UI updates ----------
  const char* stateStr =
    (state == STATE_WARM) ? "WARMUP" :
    (state == STATE_SAFE) ? "SAFE" :
    (state == STATE_PRE ) ? "PRE"  :
                            "EMG";

  // risk & gas & state text
  Blynk.virtualWrite(V0, P * 100.0f);
  Blynk.virtualWrite(V2, gasEma);
  Blynk.virtualWrite(V1, stateStr);

  // FIXED: state LEDs (0/255) instead of writing pin numbers
  Blynk.virtualWrite(V4, (state == STATE_SAFE) ? 255 : 0); // SAFE LED
  Blynk.virtualWrite(V5, (state == STATE_PRE)  ? 255 : 0); // PRE LED
  Blynk.virtualWrite(V6, (state == STATE_EMG)  ? 255 : 0); // EMG LED

  // NEW: valve status text on V8
  const char* valveStr =
    valveClosed  ? "CLOSED"  :
    valvePartial ? "PARTIAL" :
                   "OPEN";
  Blynk.virtualWrite(V8, valveStr);

  // One-time emergency notification
  if (emergencyLatched && !lastEmergencyLatched) {
    Blynk.logEvent("emergency", "FlameEye-X: EMERGENCY gas/heat/flame detected");
  }
  lastEmergencyLatched = emergencyLatched;

  // ---------- 15) Serial debug (1 Hz) ----------
  if (nowMs - lastPrintMs >= 1000) {
    lastPrintMs = nowMs;

    Serial.print(nowMs);              Serial.print("  ");
    Serial.print(stateStr);           Serial.print("  ");
    Serial.print("rawGas=");          Serial.print(rawGas);
    Serial.print("  gasEma=");        Serial.print(gasEma, 1);
    Serial.print("  flameRaw=");      Serial.print(flameRaw);
    Serial.print("  tempC=");         Serial.print(tempC, 2);
    Serial.print("  tempEma=");       Serial.print(tempEma, 2);
    Serial.print("  dTdt=");          Serial.print(dTdt, 3);
    Serial.print("  duration=");      Serial.print(durationExp, 1);
    Serial.print("  Ctilde=");        Serial.print(Ctilde, 2);
    Serial.print("  Ttilde=");        Serial.print(Ttilde, 2);
    Serial.print("  Dtilde=");        Serial.print(Dtilde, 2);
    Serial.print("  Ipos=");          Serial.print(Ipos, 2);
    Serial.print("  P=");             Serial.print(P, 3);
    Serial.print("  fanOn=");         Serial.print(fanOn ? 1 : 0);
    Serial.print("  vPart=");         Serial.print(valvePartial ? 1 : 0);
    Serial.print("  vClos=");         Serial.print(valveClosed ? 1 : 0);
    Serial.print("  flameDet=");      Serial.print(flameDet ? 1 : 0);
    Serial.print("  baselineGas=");   Serial.print(baselineGas, 1);
    Serial.print("  baselineTemp=");  Serial.print(baselineTemp, 1);
    Serial.print("  emgLatched=");    Serial.print(emergencyLatched ? 1 : 0);
    Serial.print("  valveLock=");     Serial.print(valveLock ? 1 : 0);
    Serial.println();
  }
}

