#include <Wire.h>
#include "RTClib.h"
#include <TOTP.h>
#include <sha1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Keypad.h>
#include "driver/rtc_io.h"
#include <esp_sleep.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>

// ---------------- OLED ----------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  32
#define OLED_RESET     -1
#define I2C_ADDRESS    0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- RTC ----------------
RTC_DS3231 rtc;

// ---------------- Keypad ----------------
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {32, 33, 25, 26};
byte colPins[COLS] = {27, 14, 12};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ---------------- TOTP ----------------
uint8_t hmacKey[] = {
  0x31,0x32,0x33,0x34,0x35,
  0x36,0x37,0x38,0x39,0x30,
  0x31,0x32,0x33,0x34,0x35,
  0x36,0x37,0x38,0x39,0x30
};
TOTP totp(hmacKey, sizeof(hmacKey));
char currentCode[7];
unsigned long lastStep = (unsigned long)-1;
const int timeOffset = 9;

// ---------------- Pins ----------------
const int ledPin = 5;
const int relayPin = 2;
const int buzzerPin = 19;
const int unlockButtonPin = 4;

// ---------------- State ----------------
unsigned long ledOnUntil = 0;
bool isUnlocked = false;
unsigned long wakeStart = 0;
const unsigned long idleTimeout = 10000UL;
String enteredCode = "";
unsigned long lastCountdownUpdate = 0;

// ---------------- Wake pins ----------------
const gpio_num_t WAKE_PIN     = GPIO_NUM_26;
const gpio_num_t HOLD_LOW_PIN = GPIO_NUM_12;

// ---------------- Bluetooth ----------------
BluetoothSerial SerialBT;
bool btEnabled = false;
const unsigned long btPressDuration = 2000; // 2s long press

// ---------------- Stable Lock (EEPROM) ----------------
const int EEPROM_SIZE = 64;
const int EEPROM_ADDR = 0;
bool stableLockMode = false;

// ---------------- '*' press detection (single/double) ----------------
const unsigned long DOUBLE_PRESS_TIME = 400; // 400ms window for double press
bool starPressed = false;
unsigned long lastStarRelease = 0;
bool waitingForDouble = false;

// ---------------- Function declarations ----------------
void enterDeepSleep();
void printWakeupReason();
void updateTOTP(unsigned long nowEpoch);
void checkPassword(const String &entered, unsigned long nowEpoch);
bool isCurrentCodeValid(const String &input, unsigned long nowEpoch);
bool isPreviousCodeValid(const String &input, unsigned long nowEpoch);
void playRisingToneSweep(int pin, int startFreq, int endFreq, int step, int delayTime);
void unlockSystem();
void lockSystem();
void showBluetoothSplash();
void showLockfeeSplash();
void saveStableLockToEEPROM();
void loadStableLockFromEEPROM();
void setStableLockMode(bool val);

// ==================================================
// ==================== SETUP =======================
void setup() {
  Serial.begin(115200);
  delay(200);

  EEPROM.begin(EEPROM_SIZE);
  loadStableLockFromEEPROM();

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  pinMode(unlockButtonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  printWakeupReason();
  rtc_gpio_hold_dis(HOLD_LOW_PIN);

  if (!rtc.begin()) while (1) delay(10);
  if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDRESS)) while (1) delay(10);

  showLockfeeSplash();

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 10);
  display.println("Password");
  display.display();

  wakeStart = millis();
}

// ==================================================
// ===================== LOOP =======================
void loop() {
  unsigned long nowEpoch = rtc.now().unixtime() + timeOffset;
  unsigned long currentStep = nowEpoch / 30;
  unsigned long currentMillis = millis();

  // ---------------- Unlock / BLE Button ----------------
  static bool buttonPreviouslyPressed = false;
  static unsigned long pressStartTime = 0;
  bool buttonPressed = (digitalRead(unlockButtonPin) == LOW);

  if (!stableLockMode) {
    if (buttonPressed) {
      if (!buttonPreviouslyPressed) {
        pressStartTime = millis();
        buttonPreviouslyPressed = true;
      }
      if (!btEnabled && (millis() - pressStartTime >= btPressDuration)) {
        SerialBT.begin("ESP32-BT-LOCK");
        btEnabled = true;
        Serial.println("[BT] Bluetooth enabled (long press)");
        showBluetoothSplash();
      }
    } else {
      if (buttonPreviouslyPressed) {
        unsigned long heldTime = millis() - pressStartTime;
        if (heldTime < btPressDuration) {
          if (!isUnlocked) {
            unlockSystem();
            Serial.println("[INFO] Button -> System Unlocked");
          } else {
            lockSystem();
            Serial.println("[INFO] Button -> System Locked");
          }
        }
        buttonPreviouslyPressed = false;
        wakeStart = millis();
      }
    }
  } else {
    if (buttonPressed && !buttonPreviouslyPressed) {
      Serial.println("[INFO] Unlock button pressed but STABLE LOCK active - ignored");
      tone(buzzerPin, 1000, 80);
      buttonPreviouslyPressed = true;
    }
    if (!buttonPressed) buttonPreviouslyPressed = false;
  }

  // ---------------- Keypad Input ----------------
  char key = keypad.getKey();
  wakeStart = (wakeStart == 0) ? millis() : wakeStart;

  if (key == '*') {
    if (!starPressed) {
      starPressed = true;
    }
  } else {
    if (starPressed) {
      if (waitingForDouble && (currentMillis - lastStarRelease <= DOUBLE_PRESS_TIME)) {
        Serial.println("[DOUBLE PRESS] '*' detected -> Stable Lock ON");
        setStableLockMode(true);
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
         display.setCursor(20, 10);
         display.println("LOCK*");
        display.display();
        tone(buzzerPin, 1200, 150);
        delay(200);
        noTone(buzzerPin);
        waitingForDouble = false;
      } else {
        waitingForDouble = true;
        lastStarRelease = currentMillis;
      }
      starPressed = false;
    }
  }

  if (waitingForDouble && (currentMillis - lastStarRelease > DOUBLE_PRESS_TIME)) {
    Serial.println("[SINGLE PRESS] '*' detected -> Clear Entry");
    enteredCode = "";
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 10);
    display.println("Password");
    display.display();
    waitingForDouble = false;
  }

  if (key && key != '*') {
    wakeStart = millis();
    tone(buzzerPin, 2000, 50);
    if (key >= '0' && key <= '9') {
      if (enteredCode.length() < 6) enteredCode += key;
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(25, 10);
      display.print(enteredCode);
      display.display();
    }
    if (key == '#') {
      // reserved
    }
    if (enteredCode.length() == 6) {
      checkPassword(enteredCode, nowEpoch);
      enteredCode = "";
    }
  }

  // ---------------- TOTP update ----------------
  if (currentStep != lastStep) {
    updateTOTP(nowEpoch);
    lastStep = currentStep;
  }

  // ---------------- Countdown & Auto Lock ----------------
  if (isUnlocked) {
    if (currentMillis - lastCountdownUpdate >= 1000) {
      lastCountdownUpdate = currentMillis;
      long remaining = (ledOnUntil - currentMillis) / 1000;
      remaining = max(0L, remaining);
      if (remaining <= 0) {
        lockSystem();
        Serial.println("[INFO] Relay OFF (Auto-Locked)");
        wakeStart = millis();
      } else {
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(20, 0);
        display.println("Unlocked");
        display.setTextSize(1);
        display.setCursor(35, 20);
        display.print("Left: ");
        display.print(remaining);
        display.println("s");
        display.display();
      }
    }
  }

  // ---------------- Idle Sleep ----------------
  if (wakeStart > 0 && (millis() - wakeStart > idleTimeout)) {
    if (!isUnlocked && enteredCode.length() == 0) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(20, 10);
      display.println("Sleeping...");
      display.display();
      delay(1000);
      enterDeepSleep();
    }
  }

  // ---------------- Bluetooth Commands ----------------
  if (btEnabled && SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      if (input == "A") {
        unsigned long epochNow = rtc.now().unixtime() + timeOffset;
        unsigned long stepNow  = epochNow / 30;
        if (stepNow != lastStep) {
          updateTOTP(epochNow);
          lastStep = stepNow;
        }
        SerialBT.print("[BT] TOTP: ");
        SerialBT.println(currentCode);
      } else if (input.toInt() > 100000) {
        unsigned long newEpoch = input.toInt();
        rtc.adjust(DateTime(newEpoch));
        SerialBT.print("[BT] RTC updated to: ");
        SerialBT.println(newEpoch);
      } else {
        SerialBT.println("[BT] Invalid input");
      }
    }
  }

  delay(50);
}

// ==================================================
// ================= HELPER FUNCTIONS ===============
void showLockfeeSplash() {
  display.clearDisplay();
  display.fillScreen(SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(2);
  display.setCursor(20, 10);
  display.println("LOCKFEE");
  display.display();
  delay(2000);
}

void unlockSystem() {
  isUnlocked = true;
  ledOnUntil = millis() + 30000UL;
  digitalWrite(relayPin, HIGH);
  digitalWrite(ledPin, LOW);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20, 0);
  display.println("Unlocked");
  display.setTextSize(1);
  display.setCursor(35, 20);
  display.print("Left: 30s");
  display.display();
}

void lockSystem() {
  isUnlocked = false;
  digitalWrite(relayPin, LOW);
  digitalWrite(ledPin, HIGH);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(25, 10);
  display.println("Locked");
  display.display();
}

void showBluetoothSplash() {
  display.clearDisplay();
  int logoX = (SCREEN_WIDTH - 16) / 2;
  static const unsigned char PROGMEM bluetooth_bmp_16x16[] = {
    0b00000110, 0b00000000,
    0b00000101, 0b00000000,
    0b00000100, 0b10000000,
    0b01100100, 0b01000000,
    0b00110100, 0b00100000,
    0b00011111, 0b11100000,
    0b00001111, 0b11000000,
    0b00000111, 0b10000000,
    0b00001111, 0b11000000,
    0b00011111, 0b11100000,
    0b00110100, 0b00100000,
    0b01100100, 0b01000000,
    0b00000100, 0b10000000,
    0b00000101, 0b00000000,
    0b00000110, 0b00000000,
    0b00000000, 0b00000000
  };
  display.drawBitmap(logoX, 0, bluetooth_bmp_16x16, 16, 16, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 22);
  display.print("Bluetooth");
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20, 10);
  display.println("Password");
  display.display();
}

void enterDeepSleep() {
  display.clearDisplay();
  display.display();
  pinMode((int)WAKE_PIN, INPUT_PULLUP);
  rtc_gpio_init(HOLD_LOW_PIN);
  rtc_gpio_set_direction(HOLD_LOW_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(HOLD_LOW_PIN, 0);
  rtc_gpio_hold_en(HOLD_LOW_PIN);
  esp_sleep_enable_ext0_wakeup(WAKE_PIN, 0);
  delay(100);
  esp_deep_sleep_start();
}

void printWakeupReason() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  switch (reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup by EXT0"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup by EXT1"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup by TIMER"); break;
    default: Serial.println("Wakeup reason unknown"); break;
  }
}

void updateTOTP(unsigned long nowEpoch) {
  char *code = totp.getCode(nowEpoch);
  strncpy(currentCode, code, 6);
  currentCode[6] = '\0';
  Serial.print("[TOTP] ");
  Serial.println(currentCode);
}

long lastTriggeredStep = -999999;
void checkPassword(const String &entered, unsigned long nowEpoch) {
  bool matchedCurrent  = isCurrentCodeValid(entered, nowEpoch);
  bool matchedPrevious = isPreviousCodeValid(entered, nowEpoch);
  long currentStep = nowEpoch / 30;
  long matchedStep = -1;
  if (matchedCurrent) matchedStep = currentStep;
  else if (matchedPrevious) matchedStep = currentStep - 1;
  if (matchedStep != -1) {
    if (matchedStep != lastTriggeredStep) {
      lastTriggeredStep = matchedStep;
      unlockSystem();
      wakeStart = millis();
      playRisingToneSweep(buzzerPin, 500, 4000, 50, 20);
      noTone(buzzerPin);        
      Serial.println("✅ TOTP valid! Relay ON for 30s");
      if (stableLockMode) {
        setStableLockMode(false);
        Serial.println("[STABLE LOCK] cleared after successful passcode");
      }
    }
  } else {
    Serial.println("❌ TOTP invalid");
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(20, 10);
    display.println("Invalid!");
    display.display();
     playRisingToneSweep(buzzerPin, 500, 2000, 50, 20);
    noTone(buzzerPin);
    delay(2000);
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(20, 10);
    display.println("Password");
    display.display();
  }
}

bool isCurrentCodeValid(const String &input, unsigned long nowEpoch) {
  return (input == currentCode);
}

bool isPreviousCodeValid(const String &input, unsigned long nowEpoch) {
  unsigned long prevStep = (nowEpoch / 30 - 1) * 30;
  char *prevCode = totp.getCode(prevStep);
  return (input == prevCode);
}

void playRisingToneSweep(int pin, int startFreq, int endFreq, int step, int delayTime) {
  for (int freq = startFreq; freq <= endFreq; freq += step) {
    tone(pin, freq);
    delay(delayTime);
  }
  noTone(pin);
}

// ---------------- Stable Lock EEPROM helpers ----------------
void saveStableLockToEEPROM() {
  EEPROM.write(EEPROM_ADDR, stableLockMode ? 1 : 0);
  EEPROM.commit();
}

void loadStableLockFromEEPROM() {
  byte v = EEPROM.read(EEPROM_ADDR);
  stableLockMode = (v == 1);
  Serial.print("[EEPROM] StableLock loaded: ");
  Serial.println(stableLockMode ? "YES" : "NO");
}

void setStableLockMode(bool val) {
  stableLockMode = val;
  saveStableLockToEEPROM();
  Serial.print("[STABLE LOCK] set to: ");
  Serial.println(stableLockMode ? "ENABLED" : "DISABLED");
}
