/*
  =====================================================================
  CYD Servo/Frequentie Tuner
  =====================================================================
  Copyright (c) 2026 [HF-Tech]
  Permission is hereby granted to any person obtaining a copy of this software and associated 
  documentation files to view, download, and use the software for personal, non-commercial educational purposes only.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
  STRICT RESTRICTIONS:
    You may NOT sell, rent, lease, or license this software.
    You may NOT use this software for any commercial purposes or financial gain.
    You may NOT distribute or redistribute this software for commercial purposes.
  
  Platform:
    - Cheap Yellow Display (CYD, board ESP32-2432S028R) with ESP-WROOM-32
    - TFT display ILI9341, 2.8", 240x320, resistief touch XPT2046
    - 8120MG digital servo with 270-degree rotation, controlled directly
      from ESP32 GPIO27 via the built-in LEDC hardware PWM. Via a 1:3 gear ratio, the full
      270-degree servo travel translates to a 0–100% range on the
      output shaft (the capacitor butterfly valve); consequently, all
      positions in this sketch ("Servo pos", MIN/MAX, stored positions, MAN mode) 
      refer to the OUTPUT (butterfly) percentage, 0–100%.
    - The synthetic toothed belt also ensures good insulation between the high voltage of the 
      capacitor and the servo.
    - Ensure there is no backlash in the butterfly valve shaft or between the gears.
      Backlash will cause hysteresis and poor repeatability.
    - Presets must always be set from the lowest rotation value to the highest to account 
      for hysteresis correction. If a preset is selected later, the servo will first move to 
      a position 4.4% below the preset value before moving to the preset value itself. 
      This largely eliminates hysteresis.    
    - Servo signal wire -> GPIO27 (SERVO_PIN)
      Most 5V servos accept a 3.3V control signal without issue, but check this for your specific servo.
    - Potentiometer (MAN mode, second method to move the servo):
      middle pin (wiper) -> GPIO35 (POT_PIN), outer pins -> 3V3 and GND.
      GPIO35 is an input-only ADC1 pin that is freely available on most CYD boards
      (often on the "P3" header) – check this for your specific
      unit and adjust POT_PIN if necessary.

  Needed libraries (Library Manager):
    - TFT_eSPI                    (Bodmer)
    - XPT2046_Touchscreen         (Paul Stoffregen)
    - Preferences                 (ingebouwd in ESP32 core)
  Important - ESP32-Arduino core versie (LEDC-API):
    This sketch uses the LEDC-API of ESP32-Arduino core 3.x:
    ledcAttach(pin, freq, resolutie) / ledcWrite(pin, duty).

  Important - TFT_eSPI configuration:
    In the TFT_eSPI library folder, the User_Setup.h (or
    User_Setup_Select.h) file must be configured so that the following pins
    are active (standard CYD wiring):

      #define ILI9341_2_DRIVER
      #define TFT_WIDTH  240
      #define TFT_HEIGHT 320
      #define TFT_MISO 12
      #define TFT_MOSI 13
      #define TFT_SCLK 14
      #define TFT_CS   15
      #define TFT_DC    2
      #define TFT_RST  -1
      #define TFT_BL   21
      #define TFT_BACKLIGHT_ON HIGH
      #define SPI_FREQUENCY  40000000

  Startup:
    - The last servo position (key "lastpos") and the last speed setting
      (key "speed") are saved to NVM (non volatile memory) 2 seconds after they stop changing—
      and restored upon startup instead of defaulting to 50%.
      If memory is empty (first run), everything starts at 50%.
    - The speed slider is non-linear: the first half of the slider covers 1–25%
      (fine, slow control), while the second half covers 25–100%.

  Operation (summary):
    - "mem pos" (1–180) is a preset address in NVM is NOT
      equivalent to the servo percentage. Two values ​​are stored at this address:
      the Freq value and the current servo/cap position (Cap position, in %)
      at the time of saving.
    - Screen 2: manually set the servo/cap to the desired
      physical position (via screen 1), go to screen 2, select an available "mem pos" number
      (or overwrite an existing position), enter the corresponding "Freq",
      and press "set". The Freq and the current servo/cap position
      are saved together under that "mem pos" address.
    - Screen 1: using "Freq (kHz)" and "set", the entire table is
      searched for the (nearest) stored frequency, and the servo
      moves to the corresponding stored servo/cap percentage (not to the
      memory position value itself).
    - "MAN" button (between MIN and MAX): enables/disables the potentiometer as a
      control input. Upon activation, the current servo position is
      recorded as the reference point (ensuring no sudden jump), provided the potentiometer is centered.
      While MAN is active (indicated by a yellow button), the potentiometer position directly determines
      the servo/butterfly valve percentage (proportional control, not speed control) relative to
      that reference point: pot centered = servo at the reference position,
      pot turned right (CW) = servo/butterfly moves further clockwise, pot turned left (CCW) =
      servo/butterfly moves further counter-clockwise—and the servo holds its position once the
      potentiometer stops moving. The extent of the servo/butterfly movement from the
      reference position at full potentiometer deflection is
      determined by the speed slider. A dead zone around the center prevents the servo from 
      "hunting" if the potentiometer is not perfectly centered.
  =====================================================================
*/

#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Preferences.h>

// --------------------------------------------------------------------
// Pin definitions
// --------------------------------------------------------------------
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

// Servo - directly connected to an ESP32 GPIO via the LEDC hardware PWM
#define SERVO_PIN 22   // Servo 5V + GND powered externally; GND shared with ESP32!

// Raw touch calibration values ​​– adjust for your specific unit if necessary
#define TS_MINX 200
#define TS_MAXX 3900
#define TS_MINY 200
#define TS_MAXY 3900

// Potentiometer for MAN mode (second way to move the servo)
#define POT_PIN            35    // ADC1_CH7, input-only - check/adjust for your specific CYD unit
#define POT_DEADBAND        0.06f // dead zone around the center (fraction of the full stroke) – prevents
                                   // that the servo keeps "hunting" around the center due to slight noise

// --------------------------------------------------------------------
// Objects
// --------------------------------------------------------------------
SPIClass touchSPI = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
TFT_eSPI tft = TFT_eSPI();
Preferences prefs;

// --------------------------------------------------------------------
// Status / variables
// --------------------------------------------------------------------
enum Screen { SCREEN_P1, SCREEN_P2 };
Screen currentScreen = SCREEN_P1;

int servoPos      = 500;    // current servo position (butterfly) in 0.1 PERCENT STEPS (0–1000 = 0–100%; 500 = 50.0% = center)
int lastFoundMemPos = -1;   // mem pos associated with the most recently found Freq match (-1 = no match yet)
int memPosValue   = 0;      // P2: "mem pos" (1-180) - 0 = empty (starts empty, just like after saving)
int freqValueP2   = 0;      // P2: "Freq" (10000-30000) -> is stored - 0 = still empty
int freqValueP1   = 10000;  // P1: "Freq (kHz)" (10000–30000) -> look up
float speedPercent = 50.0f; // P1: "speed" (1–100%, in 0.1% increments)

int lastKnobX = -1;         // for clearing the old slider indication

bool prevTouched = false;
bool manMode = false;       // P1: MAN button active -> potentiometer controls the servo
int manRefStep = 500;       // servo position (in steps) serving as the "center" for the potentiometer -
                             // set to the current servoPos when MAN is enabled,
                             // so the potentiometer acts as an offset relative to the position at that moment.

// --------------------------------------------------------------------
// UI helper structure
// --------------------------------------------------------------------
struct Btn {
  int x, y, w, h;
  const char* label;
};

bool hit(Btn b, int tx, int ty) {
  return (tx >= b.x && tx <= b.x + b.w && ty >= b.y && ty <= b.y + b.h);
}

void drawButton(Btn b, uint16_t fillColor, uint16_t txtColor = TFT_WHITE, int textSize = 2) {
  tft.fillRoundRect(b.x, b.y, b.w, b.h, 6, fillColor);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 6, TFT_WHITE);
  tft.setTextColor(txtColor, fillColor);
  tft.setTextSize(textSize);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(b.label, b.x + b.w / 2, b.y + b.h / 2);
  tft.setTextDatum(TL_DATUM);
}

// --------------------------------------------------------------------
// Touch reading
// --------------------------------------------------------------------
bool getTouchPoint(int &x, int &y) {
  if (!ts.touched()) return false;
  TS_Point p = ts.getPoint();
  x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
  y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
  x = constrain(x, 0, tft.width() - 1);
  y = constrain(y, 0, tft.height() - 1);
  return true;
}

// Forward declaration (the function itself appears later, under screen 2)
bool memPosInUse(int memPos, int &outFreq, int &outPercentStep);

// ======================================================================
// NUMERIC KEYPAD (modal input screen)
// ======================================================================
Btn kp_digits[12] = {
  {47,  45, 70, 35, "1"}, {125, 45, 70, 35, "2"}, {203, 45, 70, 35, "3"},
  {47,  88, 70, 35, "4"}, {125, 88, 70, 35, "5"}, {203, 88, 70, 35, "6"},
  {47, 131, 70, 35, "7"}, {125,131, 70, 35, "8"}, {203,131, 70, 35, "9"},
  {47, 174, 70, 35, "C"}, {125,174, 70, 35, "0"}, {203,174, 70, 35, "<-"}
};
Btn kp_cancel = {47, 214, 110, 24, "Cancel"};
Btn kp_enter  = {163,214, 110, 24, "OK"};

void drawKeypadEntry(String entry) {
  tft.fillRect(20, 5, 280, 34, TFT_NAVY);
  tft.drawRect(20, 5, 280, 34, TFT_WHITE);
  tft.setTextColor(TFT_YELLOW, TFT_NAVY);
  tft.setTextSize(3);
  tft.setTextDatum(MR_DATUM);
  tft.drawString(entry, 292, 22);
  tft.setTextDatum(TL_DATUM);
}

void drawKeypadBase(const char* title) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(title, 20, 2);

  for (int i = 0; i < 12; i++) {
    drawButton(kp_digits[i], TFT_DARKGREY, TFT_WHITE, 2);
  }
  drawButton(kp_cancel, TFT_RED, TFT_WHITE, 2);
  drawButton(kp_enter,  TFT_DARKGREEN, TFT_WHITE, 2);
}

void drawKeypadError() {
  tft.fillRect(20, 5, 280, 34, TFT_RED);
  tft.drawRect(20, 5, 280, 34, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_RED);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Invalid value!", 160, 22);
  tft.setTextDatum(TL_DATUM);
}

// Blocking modal keyboard. Returns the entered (validated) value,
// or the original value 'initVal' if cancelled.
long numericKeypad(const char* title, long minVal, long maxVal, long initVal) {
  String entry = ""; // always start empty, regardless of the field's current value.
  bool done = false;
  long result = initVal;

  drawKeypadBase(title);
  drawKeypadEntry(entry);

  // The touch that activated the field might still be active on the screen
  // (finger not yet lifted) exactly where a numeric key on this keyboard
  // happens to be drawn. Therefore, initialize `wasTouched` based on the
  // CURRENT touch state instead of always setting it to 'false', so that
  // the ongoing touch isn't immediately registered as a key press—only
  // a new touch (after releasing) counts as a key press.
  int txInit, tyInit;
  bool wasTouched = getTouchPoint(txInit, tyInit);
  while (!done) {
    int tx, ty;
    bool touched = getTouchPoint(tx, ty);
    bool edge = touched && !wasTouched;

    if (edge) {
      bool handled = false;
      for (int i = 0; i < 12 && !handled; i++) {
        if (hit(kp_digits[i], tx, ty)) {
          String lbl = kp_digits[i].label;
          if (lbl == "C") {
            entry = "";
          } else if (lbl == "<-") {
            if (entry.length() > 0) entry.remove(entry.length() - 1);
          } else {
            if (entry.length() < 6) entry += lbl;
          }
          drawKeypadEntry(entry);
          handled = true;
        }
      }
      if (!handled && hit(kp_cancel, tx, ty)) {
        result = initVal;
        done = true;
      }
      if (!handled && hit(kp_enter, tx, ty)) {
        long v = entry.toInt();
        if (entry.length() > 0 && v >= minVal && v <= maxVal) {
          result = v;
          done = true;
        } else {
          drawKeypadError();
          delay(700);
          drawKeypadEntry(entry);
        }
      }
    }
    wasTouched = touched;
    delay(15);
  }
  return result;
}

// ======================================================================
// LIST OF SAVED FREQ VALUES (modal selection screen)
// ======================================================================
struct FreqEntry { int memPos; int freq; int percentStep; };
FreqEntry freqListEntries[180];
int freqListCount = 0;
int freqListPage = 0;
char freqListRowText[12][16];
Btn freqListRowBtn[12];
Btn freqListBtnPrev  = {10,  205, 90, 30, "Prev"};
Btn freqListBtnNext  = {115, 205, 90, 30, "Next"};
Btn freqListBtnClose = {220, 205, 90, 30, "Close"};

void drawFreqListPage(long currentFreq, int totalPages) {
  const int rowsPerPage = 12;
  const int rowsPerCol = 6;
  const int colW = 150, rowH = 26;
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(TC_DATUM);
  char hdr[24];
  snprintf(hdr, sizeof(hdr), "Select Frequency (%d/%d)", freqListPage + 1, totalPages);
  tft.drawString(hdr, 160, 2);
  tft.setTextDatum(TL_DATUM);

  int startIdx = freqListPage * rowsPerPage;
  for (int i = 0; i < rowsPerPage; i++) {
    int idx = startIdx + i;
    int col = i / rowsPerCol;      // 0 = links, 1 = rechts
    int row = i % rowsPerCol;
    freqListRowBtn[i].x = 7 + col * (colW + 6);
    freqListRowBtn[i].y = 28 + row * 30;
    freqListRowBtn[i].w = colW;
    freqListRowBtn[i].h = rowH;
    freqListRowBtn[i].label = "";
    if (idx < freqListCount) {
      snprintf(freqListRowText[i], sizeof(freqListRowText[i]), "%d: %d",
               freqListEntries[idx].memPos, freqListEntries[idx].freq);
      freqListRowBtn[i].label = freqListRowText[i];
      bool highlight = (freqListEntries[idx].freq == currentFreq);
      drawButton(freqListRowBtn[i], highlight ? TFT_DARKGREEN : TFT_NAVY, TFT_WHITE, 2);
    }
  }
  drawButton(freqListBtnPrev,  (freqListPage > 0)              ? TFT_BLUE : TFT_DARKGREY, TFT_WHITE, 1);
  drawButton(freqListBtnNext,  (freqListPage < totalPages - 1)  ? TFT_BLUE : TFT_DARKGREY, TFT_WHITE, 1);
  drawButton(freqListBtnClose, TFT_RED, TFT_WHITE, 1);
}

// Modal selection screen with all saved frequency values ​​(sorted by memory position).
// Returns the selected frequency, or 'currentFreq' if cancelled or nothing is saved.
long chooseStoredFreq(long currentFreq) {
  freqListCount = 0;
  for (int p = 1; p <= 180; p++) {
    int freq, percentStep;
    if (memPosInUse(p, freq, percentStep)) {
      freqListEntries[freqListCount].memPos = p;
      freqListEntries[freqListCount].freq = freq;
      freqListEntries[freqListCount].percentStep = percentStep;
      freqListCount++;
    }
  }

  if (freqListCount == 0) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("No stored", 160, 100);
    tft.drawString("Freq-values", 160, 130);
    tft.setTextDatum(TL_DATUM);
    delay(1200);
    return currentFreq;
  }

  const int rowsPerPage = 12;
  int totalPages = (freqListCount + rowsPerPage - 1) / rowsPerPage;
  freqListPage = 0;
  long result = currentFreq;
  bool done = false;

  drawFreqListPage(currentFreq, totalPages);

  // Same reason as for numericKeypad(): the touch that opened "List" might
  // coincide with a row/button on this selection screen. Start
  // wasTouched based on the CURRENT touch state, not on 'false'.
  int txInit, tyInit;
  bool wasTouched = getTouchPoint(txInit, tyInit);
  while (!done) {
    int tx, ty;
    bool touched = getTouchPoint(tx, ty);
    bool edge = touched && !wasTouched;

    if (edge) {
      if (hit(freqListBtnClose, tx, ty)) {
        done = true;
      } else if (hit(freqListBtnPrev, tx, ty) && freqListPage > 0) {
        freqListPage--;
        drawFreqListPage(currentFreq, totalPages);
      } else if (hit(freqListBtnNext, tx, ty) && freqListPage < totalPages - 1) {
        freqListPage++;
        drawFreqListPage(currentFreq, totalPages);
      } else {
        int startIdx = freqListPage * rowsPerPage;
        for (int i = 0; i < rowsPerPage; i++) {
          int idx = startIdx + i;
          if (idx < freqListCount && hit(freqListRowBtn[i], tx, ty)) {
            result = freqListEntries[idx].freq;
            done = true;
            break;
          }
        }
      }
    }
    wasTouched = touched;
    delay(15);
  }
  return result;
}

// ======================================================================
// NON-VOLATILE MEMORY (Preferences / NVM)
// ======================================================================
// mem pos is a separate preset address (1–180), NOT equal to the servo percentage.
// TWO values ​​are stored at that address: the Freq and the actual servo
// percentage displayed in "Servo pos" at the time of saving.
void saveFreqAtMemPos(int memPos, int freq, int actualPercentStep) {
  char keyF[10], keyA[10];
  snprintf(keyF, sizeof(keyF), "p%df", memPos);
  snprintf(keyA, sizeof(keyA), "p%da", memPos);
  prefs.putInt(keyF, freq);
  prefs.putInt(keyA, actualPercentStep);
  Serial.printf("[Save] mem pos %d -> Freq %d, %.1f%%\n", memPos, freq, actualPercentStep / 10.0f);
}

// Searches all saved presets for the frequency that best matches 'freq'
// and returns the servo percentage stored with it (not the memory position value itself).
// Returns -1 if nothing has been saved yet.
int findPercentForFreq(int freq, int &foundMemPos) {
  int bestPercentStep = -1;
  int bestMemPos = -1;
  long bestDiff = 2147483647;

  Serial.printf("[Search] gevraagde Freq = %d\n", freq);
  for (int p = 1; p <= 180; p++) {
    char keyF[10], keyA[10];
    snprintf(keyF, sizeof(keyF), "p%df", p);
    snprintf(keyA, sizeof(keyA), "p%da", p);
    if (prefs.isKey(keyF) && prefs.isKey(keyA)) {
      int storedFreq        = prefs.getInt(keyF, -1);
      int storedPercentStep = prefs.getInt(keyA, -1);
      Serial.printf("  mem pos %d -> Freq %d, %.1f%%\n", p, storedFreq, storedPercentStep / 10.0f);
      if (storedFreq == freq) {
        Serial.printf("  -> exact match: mem pos %d, %.1f%%\n", p, storedPercentStep / 10.0f);
        foundMemPos = p;
        return storedPercentStep; // exact match
      }
      long diff = abs((long)storedFreq - (long)freq);
      if (diff < bestDiff) {
        bestDiff = diff;
        bestPercentStep = storedPercentStep;
        bestMemPos = p;
      }
    }
  }
  Serial.printf("[Search] best (not-exact) match: mem pos %d, %.1f%%\n", bestMemPos, bestPercentStep / 10.0f);
  foundMemPos = bestMemPos;
  return bestPercentStep;
}

// ======================================================================
// SERVO STEERING
// ======================================================================
// The servo is controlled using microseconds instead of whole percentages, allowing
// for a resolution of 0.1% per step.
//
// This servo has a physical travel range of 270 degrees (pulse width 500–2500 µs
// over the FULL range). A 1:3 timing belt drive translates this into
// 0–100% movement at the output shaft (the butterfly capacitor): the servo
// therefore rotates three times as far as the butterfly. Since the full
// 270-degree servo travel corresponds exactly to the full 500–2500 µs
// pulse width range, the pulse remains directly proportional to the
// OUTPUT (butterfly) percentage. The range is divided into 1000 steps
// of 0.1% (0–100%): step 0 = 0.0% (butterfly), step 1000 = 100.0% (butterfly).
// Because the total pulse width (2000 µs) is distributed across these
// 1000 steps, each step corresponds to a pulse width difference of exactly
// 2000 / 1000 = 2 µs—meaning the servo is controlled in 2 µs increments.
//
// NOTE – achievable resolution via the ESP32 LEDC hardware PWM:
// LEDC divides the PWM period (20,000 µs at 50 Hz) into 2^LEDC_RESOLUTION_BITS
// count steps. With LEDC_RESOLUTION_BITS=16, that is 65,536 steps over 20,000 µs,
// resulting in approx. 0.31 µs per count step — far finer than the 2000/1000 = 2 µs
// required for a true 0.1% step. Consequently, every software step here
// actually results in a different physical pulse width.

#define SERVO_MIN_US   500   // pulse width at 0% throttle (= 0-degree servo)
#define SERVO_MAX_US   2500  // pulse width at 100% throttle (= 270-degree servo, full travel)
#define SERVO_MAX_STEP 1000  // 1,000 steps = 0.1% (butterfly) per step, over 0–100% -> 2 µs per step
#define SERVO_PWM_FREQ 50           // PWM frequency for the servo (Hz) – standard for analog/digital RC servos
#define LEDC_RESOLUTION_BITS 16     // duty cycle resolution of the ESP32 LEDC timer (16-bit = 65,536 steps/period)
#define LEDC_MAX_DUTY ((1UL << LEDC_RESOLUTION_BITS) - 1)
/* Adjusting for a different servo travel range or gear ratio:
The full output range is always represented as 0–100%, in 1,000 steps of 0.1%.
SERVO_MIN_US and SERVO_MAX_US represent the pulse widths at 0% and 100%
of the full physical servo travel, respectively (in this case, 500 µs and 2,500 µs
for a 0–270° servo). If the servo travel range differs, only SERVO_MIN_US
and SERVO_MAX_US need to be adjusted; SERVO_MAX_STEP remains 1,000
provided a resolution of 0.1% is desired.
*/
// servoPercent() returns the OUTPUT (butterfly) percentage (0–100%),
// not the physical servo angle—which is three times larger due to the 1:3 gear ratio.
float servoPercent() {
  return servoPos / 10.0f;
}

// Sends the servo to the current servoPos (in 0.1% increments) via the
// ESP32 LEDC hardware PWM. First converts the servo step into a pulse width
// in microseconds (same as before), and then into an
// LEDC duty value (0 .. LEDC_MAX_DUTY) corresponding to SERVO_PWM_FREQ.
void servoWriteCurrent() {
  int step = constrain(servoPos, 0, SERVO_MAX_STEP);
  long us = SERVO_MIN_US + (long)(SERVO_MAX_US - SERVO_MIN_US) * step / SERVO_MAX_STEP;

  float periodUs = 1000000.0f / SERVO_PWM_FREQ;                 // 20000 us bij 50 Hz
  uint32_t duty = (uint32_t)((us / periodUs) * LEDC_MAX_DUTY + 0.5f);

  ledcWrite(SERVO_PIN, duty);
}

// --------------------------------------------------------------------
// Remember last servo position (across power cycles)
// --------------------------------------------------------------------
// The current servoPos is stored in NVM under the key "lastpos" so that
// the servo returns to its last position after startup instead of 50%.
// To minimize flash memory wear, writing does NOT occur at every step,
// but only after the servo has been stationary for SERVO_SAVE_DELAY_MS
// and the position differs from the last stored value.
#define SERVO_SAVE_DELAY_MS 2000UL
int lastSavedServoPos = -1;           // position recently written up in NVM
int lastSeenServoPos  = -1;           // reading at the previous check
unsigned long servoLastChangeMs = 0;  // time of the last modification
int lastSavedSpeedX10 = -1;           // speed last recorded in NVM (in 0.1%)
int lastSeenSpeedX10  = -1;           // speed at the previous check
unsigned long speedLastChangeMs = 0;  // time of the last speed change

void loadLastServoPos() {
  int stored = prefs.getInt("lastpos", 500);        // 500 = 50,0% if nothing has been saved yet
  servoPos = constrain(stored, 0, SERVO_MAX_STEP);
  lastSavedServoPos = servoPos;
  lastSeenServoPos  = servoPos;
  manRefStep = servoPos;

  int storedSpeed = prefs.getInt("speed", 500);      // 500 = 50,0% if nothing has been saved yet
  speedPercent = constrain(storedSpeed, 10, 1000) / 10.0f;
  lastSavedSpeedX10 = lastSeenSpeedX10 = (int)(speedPercent * 10.0f + 0.5f);
  Serial.printf("[opstart] laatste snelheid geladen: %.1f%%\n", speedPercent);
  Serial.printf("[opstart] laatste servostand geladen: stap %d (%.1f%%)\n", servoPos, servoPos / 10.0f);
}

void saveServoPosIfSettled() {
  if (servoPos != lastSeenServoPos) {               // Servo is (still) moving.
    lastSeenServoPos = servoPos;
    servoLastChangeMs = millis();
  } else if (servoPos != lastSavedServoPos && millis() - servoLastChangeMs >= SERVO_SAVE_DELAY_MS) {
    prefs.putInt("lastpos", servoPos);
    lastSavedServoPos = servoPos;
    Serial.printf("[Save] last servoposition: stap %d (%.1f%%)\n", servoPos, servoPos / 10.0f);
  }

  // Save speed (slider) in the same way
  int speedX10 = (int)(speedPercent * 10.0f + 0.5f);
  if (speedX10 != lastSeenSpeedX10) {
    lastSeenSpeedX10 = speedX10;
    speedLastChangeMs = millis();
  } else if (speedX10 != lastSavedSpeedX10 && millis() - speedLastChangeMs >= SERVO_SAVE_DELAY_MS) {
    prefs.putInt("speed", speedX10);
    lastSavedSpeedX10 = speedX10;
    Serial.printf("[Save] last speed: %.1f%%\n", speedPercent);
  }
}

// --------------------------------------------------------------------
// Speed ​​model: fixed tick, variable chunk size
// --------------------------------------------------------------------
// Due to the low PWM frequency (50 Hz → 20 ms period), every `servoWriteCurrent()` 
//or `ledcWrite()` call takes around 20–30 ms anyway, regardless of any software delay 
//we might add ourselves. A model that translates "speed" into the delay between fixed 0.1% 
//steps therefore always hits this hardware-imposed limit: with 1,000 steps, a full stroke takes 
//at least 1,000 × ~25 ms, no matter the setting.
//
// Instead, the SIZE of each step is varied, here at a
// fixed tick rate (TICK_MS, close to the hardware's lower limit):
// a tiny increment (0.1%, for precise fine-tuning) at low speeds,
// and a large increment (4.4%, for rapid movement) at high speeds—
// with a linear progression in between. This ensures the time
// between updates remains constant (and thus always achievable
// by the hardware), while the increment size alone determines
// the servo's rate of progress. This approach is used by the
// preset movement (moveServoTo), the MIN/MAX hold function
// (handleMinMaxHold), and the potentiometer in MAN mode
// (handleManualPotentiometer), ensuring all three share the
// same predictable speed curve.
// MAX_CHUNK_STEPS is selected (44 steps = 4.4% of the 0–100% range)
// so that the PHYSICAL (butterfly valve) speed remains exactly
// equivalent to the old 0.1-degree resolution (where 40 steps
// equaled 4.0 degrees within the old 0–90 degree range:
// also 40/900 = 4.4% of the range per tick).
const unsigned long TICK_MS = 25;      // fixed interval between updates, based on the underlying LEDC hardware
const int MIN_CHUNK_STEPS   = 1;       // 0.1% per tick at 1% speed (finest control)
const int MAX_CHUNK_STEPS   = 44;      // 4.4% per tick at 100% speed (fastest control, physically identical to before)

int chunkStepsFromSpeed(float percent) {
  float frac = (percent - 1.0f) / 99.0f;  // 0.0 by 1%, 1.0 by 100%
  frac = constrain(frac, 0.0f, 1.0f);
  int chunk = (int)(MIN_CHUNK_STEPS + frac * (MAX_CHUNK_STEPS - MIN_CHUNK_STEPS) + 0.5f);
  return constrain(chunk, MIN_CHUNK_STEPS, MAX_CHUNK_STEPS);
}

// Moves the servo incrementally (in increments of chunkStepsFromSpeed() steps,
// at a fixed tick interval of TICK_MS) toward the target position, with a live update of the
// "Pos" indicator. 'target' is expressed in steps (0–1000, 0.1% per step).
void moveServoTo(int target) {
  target = constrain(target, 0, SERVO_MAX_STEP);
  int chunk = chunkStepsFromSpeed(speedPercent);
  Serial.printf("[servo] move from step %d (%.1f%%) to step %d (%.1f%%), chunk %d steps (%.1f%%)\n",
                servoPos, servoPercent(), target, target / 10.0f, chunk, chunk / 10.0f);

  int dir = (target > servoPos) ? 1 : -1;

  // Rewriting the TFT display via SPI takes a few milliseconds each time. Refreshing
  // on every tick would cause this to add up; therefore, the display update is
  // limited to a maximum of once every DRAW_INTERVAL_MS, regardless of the tick frequency.
  const unsigned long DRAW_INTERVAL_MS = 40;
  unsigned long lastDrawMs = 0;

  while (servoPos != target) {
    int remaining = abs(target - servoPos);
    int step = min(chunk, remaining);   // don't shoot the last chunk past the target
    servoPos += dir * step;
    servoWriteCurrent();

    unsigned long now = millis();
    if (now - lastDrawMs >= DRAW_INTERVAL_MS || servoPos == target) {
      drawPosValue();
      lastDrawMs = now;
    }

    delay(TICK_MS);
  }
}

// --------------------------------------------------------------------
// Hysteresis compensation when recalling a preset
// --------------------------------------------------------------------
// Due to backlash in the gear train, the butterfly valve does not
// end up in exactly the same position if the preceding movement was from "high to
// low" rather than "low to high" (or vice versa)—the backlash is
// only "taken up" at the end of the movement, and this occurs on a
// different side of the gears depending on the direction of movement.
// To compensate for this, when recalling a preset, the
// final approach is always performed from the SAME direction: the servo first
// moves to an intermediate stop HYSTERESIS_COMP_PERCENT % below the target value,
// and only then moves to the final target value. This ensures the final
// partial movement is always upwards (low -> high), regardless of whether the servo
// was previously positioned higher or lower than the target, and the backlash is
// consistently "taken up" in the same way -> resulting in better repeatability.
// the value for HYSTERESIS_COMP_PERCENT is set to 4.4%, change the value if needed.
#define HYSTERESIS_COMP_PERCENT 4.4f                                // % below the target value for the intermediate stop
#define HYSTERESIS_COMP_STEPS  ((int)(HYSTERESIS_COMP_PERCENT * 10)) // converted to steps (0.1% per step)

// Moves the servo to 'target' (in steps) WITH hysteresis compensation:
// first to (target - HYSTERESIS_COMP_STEPS), then to target. Used
// when recalling a preset (Screen 1, "set"); for other
// movements (MIN/MAX, MAN mode), the standard moveServoTo() remains in use.
void moveServoToPreset(int target) {
  target = constrain(target, 0, SERVO_MAX_STEP);

  // Compensate only if the new target is LOWER than the current position:
  // then the final partial movement approaches the target from below,
  // and backlash is consistently taken up in the same way. If the
  // servo is already below (or at) the target, the final
  // partial movement is already an upward one, so the intermediate stop is unnecessary.
  if (target < servoPos) {
    int approachFrom = constrain(target - HYSTERESIS_COMP_STEPS, 0, SERVO_MAX_STEP);
    Serial.printf("[servo] preset-nadering (van hoog naar laag): eerst naar stap %d (%.1f%%), dan naar stap %d (%.1f%%)\n",
                  approachFrom, approachFrom / 10.0f, target, target / 10.0f);
    moveServoTo(approachFrom);
    moveServoTo(target);
  } else {
    Serial.printf("[servo] preset-nadering (van laag naar hoog): direct naar stap %d (%.1f%%), geen tussenstop nodig\n",
                  target, target / 10.0f);
    moveServoTo(target);
  }
}

// ======================================================================
// POTENTIOMETER (MAN mode) – second way to control the servo,
// alongside the MIN/MAX buttons.
// ======================================================================

// Reads the potentiometer and returns a centered fraction:
// -1.0 = fully counter-clockwise, 0.0 = center, +1.0 = fully clockwise.
// Sampling a few times dampens ADC noise.
float readPotCentered() {
  const int samples = 4;
  long total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogRead(POT_PIN);
    delayMicroseconds(200);
  }
  int raw = (int)(total / samples);          // 0-4095 (12-bit ADC)
  float frac = (raw - 2048) / 2048.0f;       // -1.0 .. +1.0
  return constrain(frac, -1.0f, 1.0f);
}

// Called during every loop iteration while MAN mode is active (screen 1).
// Here, the potentiometer acts as a PROPORTIONAL POSITION control (no longer
// a speed control): the pot's position directly determines a target value,
// and the servo moves to that position and holds it—no continuous movement
// occurs as long as the pot remains in the same position.
//   - Pot centered (within the deadband)           -> target = manRefStep (the
//     servo position at the moment MAN was activated—so NO jump
//     to 50% if the servo was positioned elsewhere)
//   - Pot turned fully clockwise (CW, MAX side)    -> target = manRefStep +
//     maximum deflection, where the MAXIMUM deflection is determined
//     by the speed slider: at 100% this is 50% (half of
//     the full 0-100% throttle range), at e.g. 50% it is only 25%
//   - Pot turned fully counter-clockwise (CCW, MIN side) -> mirror image of the above
// There is a deadband (POT_DEADBAND) around the center so the servo does not
// "hunt" if the pot is not perfectly centered.
// The servo moves in steps (chunks) toward the new target value, at
// the rate indicated by the speed slider (chunkStepsFromSpeed()).
void handleManualPotentiometer() {
  float frac = readPotCentered(); // -1.0 (far left) .. 0.0 (middle) .. +1.0 (far right)

  // Maximum deflection (in 0.1% increments) from manRefStep, determined
  // by the slider: 100% = full 50% deflection allowed in each direction
  // (half of the 0–100% butterfly range).
  int maxOffsetSteps = (int)((speedPercent / 100.0f) * (SERVO_MAX_STEP / 2));

  int targetStep;
  if (fabs(frac) < POT_DEADBAND) {
    targetStep = manRefStep; // within the dead zone -> target is the reference position
  } else {
    // Rescale the usable part of the stroke (outside the dead zone) to 0.0–1.0.
    float travel = (fabs(frac) - POT_DEADBAND) / (1.0f - POT_DEADBAND);
    travel = constrain(travel, 0.0f, 1.0f);
    int dir = (frac > 0) ? 1 : -1; // right = clockwise (CW/MAX side), left = counter-clockwise (CCW/MIN side)
    targetStep = manRefStep + dir * (int)(travel * maxOffsetSteps);
  }
  targetStep = constrain(targetStep, 0, SERVO_MAX_STEP);

  if (servoPos == targetStep) {
    return; // already at the desired position -> nothing to do
  }

  int chunk = chunkStepsFromSpeed(speedPercent); // how quickly the servo moves towards the target position
  int dir = (targetStep > servoPos) ? 1 : -1;
  int remaining = abs(targetStep - servoPos);
  int step = min(chunk, remaining);   // not overshoot the mark
  servoPos += dir * step;
  servoWriteCurrent();
  drawPosValue();
  delay(TICK_MS);
}

// ======================================================================
// Screen 1 - lay-out
// ======================================================================
Btn btnP2      = {15,   5,  290, 30, "SAVE SERVO POSITION"};
Btn btnMin     = {15,  45, 90, 50, "MIN"};   // Button MIN (servo moving CCW)
Btn btnMan     = {113, 45, 90, 50, "MAN"};   // Button MAN (potentiometer on/off)
Btn btnMax     = {211, 45, 90, 50, "MAX"};   // Button MAX (servo moving CW)
Btn posFieldP1   = {15,  144, 80, 36, ""};   // percentage readout field, directly above the Freq input field
Btn memPosFieldP1= {145, 144, 50, 36, ""};   // mem POS readout field
Btn freqFieldP1  = {15, 185, 80, 36, ""};    // frequency readout field
Btn btnListP1    = {145,185, 60, 36, "LIST"};// Memory list display
Btn btnSetP1   = {235,185, 80, 36, "SET"};   // Set choosen frequency

const int sliderX = 15, sliderY = 122, sliderW = 290, sliderH = 16;   // Slider

void drawSliderTrack() {
  tft.fillRect(sliderX, sliderY, sliderW, sliderH, TFT_DARKGREY);
  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);
}

// Non-linear slider: the first 50% of the slider's travel covers only the
// 1–25% speed range, allowing for much finer adjustment
// in the low (slow) range. The second half of the slider goes from 25% to 100%.
#define SLIDER_KNEE_FRAC   0.5f    // slider position (0–1) where the breakpoint is located
#define SLIDER_KNEE_SPEED  25.0f   // speed (%) at the breakpoint

float speedFromSliderFrac(float f) {
  f = constrain(f, 0.0f, 1.0f);
  if (f <= SLIDER_KNEE_FRAC) {
    return 1.0f + (f / SLIDER_KNEE_FRAC) * (SLIDER_KNEE_SPEED - 1.0f);
  }
  return SLIDER_KNEE_SPEED + ((f - SLIDER_KNEE_FRAC) / (1.0f - SLIDER_KNEE_FRAC)) * (100.0f - SLIDER_KNEE_SPEED);
}

float sliderFracFromSpeed(float sp) {
  sp = constrain(sp, 1.0f, 100.0f);
  if (sp <= SLIDER_KNEE_SPEED) {
    return (sp - 1.0f) / (SLIDER_KNEE_SPEED - 1.0f) * SLIDER_KNEE_FRAC;
  }
  return SLIDER_KNEE_FRAC + (sp - SLIDER_KNEE_SPEED) / (100.0f - SLIDER_KNEE_SPEED) * (1.0f - SLIDER_KNEE_FRAC);
}

void drawSliderKnob() {
  float frac = sliderFracFromSpeed(speedPercent); // 0.0 .. 1.0
  int knobX = sliderX + 6 + (int)(frac * (sliderW - 12) + 0.5f);
  // clear old indication (redraw local track segment instead of reloading the entire slider)
  if (lastKnobX >= 0) {
    tft.fillRect(lastKnobX - 8, sliderY - 4, 16, sliderH + 8, TFT_BLACK);
    tft.fillRect(sliderX, sliderY, sliderW, sliderH, TFT_DARKGREY);
    tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);
  }
  tft.fillCircle(knobX, sliderY + sliderH / 2, 9, TFT_ORANGE);
  tft.drawCircle(knobX, sliderY + sliderH / 2, 9, TFT_WHITE);
  lastKnobX = knobX;

  // show percentage (1 decimal place, 0.5% increments)
  tft.fillRect(sliderX, sliderY - 20, 120, 16, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  char buf[20];
  snprintf(buf, sizeof(buf), "servo speed: %.1f%%", speedPercent);
  tft.drawString(buf, sliderX, sliderY - 20);
}

// Draws the frame and the "%" label for the percentage readout field. Drawn
// once when screen 1 is built (just like the Freq field below it);
// the value itself is updated via drawPosValue().
void drawPosFieldP1() {
  tft.fillRoundRect(posFieldP1.x, posFieldP1.y, posFieldP1.w, posFieldP1.h, 6, TFT_NAVY);
  tft.drawRoundRect(posFieldP1.x, posFieldP1.y, posFieldP1.w, posFieldP1.h, 6, TFT_WHITE);

  // Place the "%" label after the value, at the same x-position as "kHz" for the
  // frequency field below, so that both fields are neatly aligned.
  int labelX = posFieldP1.x + posFieldP1.w + 6;
  tft.fillRect(labelX, posFieldP1.y, btnListP1.x - labelX - 4, posFieldP1.h, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("%", labelX, posFieldP1.y + posFieldP1.h / 2);
  tft.setTextDatum(TL_DATUM);

  drawPosValue();
}

// Refreshes only the value *inside* the percentage display field (the frame remains).
// This is called frequently (at every step during a servo movement), so
// fillRoundRect/drawRoundRect is deliberately avoided here—that would cause unnecessary flickering.
void drawPosValue() {
  tft.fillRect(posFieldP1.x + 4, posFieldP1.y + 4, posFieldP1.w - 8, posFieldP1.h - 8, TFT_NAVY);
  tft.setTextColor(TFT_GREEN, TFT_NAVY);
  tft.setTextSize(2);
  tft.setTextDatum(ML_DATUM);
  char buf[16];
  snprintf(buf, sizeof(buf), "%5.1f", servoPercent());
  tft.drawString(buf, posFieldP1.x + 6, posFieldP1.y + posFieldP1.h / 2);
  tft.setTextDatum(TL_DATUM);
}

// Displays (or clears) the "mem pos" match value IN the mem-pos readout field
// (the frame remains; only the value is updated). Replaces the old
// blue "Freq:" text, which was redundant given the Freq input field.

void drawMemMatchInfo(int memPos) {
  tft.fillRect(memPosFieldP1.x + 4, memPosFieldP1.y + 4, memPosFieldP1.w - 8, memPosFieldP1.h - 8, TFT_NAVY);
  tft.setTextColor(TFT_GREEN, TFT_NAVY);
  tft.setTextSize(2);
  tft.setTextDatum(ML_DATUM);
  char buf[8];
  if (memPos > 0) {
    snprintf(buf, sizeof(buf), "%d", memPos);
  } else {
    snprintf(buf, sizeof(buf), "--");
  }
  tft.drawString(buf, memPosFieldP1.x + 6, memPosFieldP1.y + memPosFieldP1.h / 2);
  tft.setTextDatum(TL_DATUM);
}

// Draws the frame and the "mem pos" label for the mem-pos readout field. Drawn
// once when screen 1 is built, next to the percentage field.
void drawMemPosFieldP1() {
  tft.fillRoundRect(memPosFieldP1.x, memPosFieldP1.y, memPosFieldP1.w, memPosFieldP1.h, 6, TFT_NAVY);
  tft.drawRoundRect(memPosFieldP1.x, memPosFieldP1.y, memPosFieldP1.w, memPosFieldP1.h, 6, TFT_WHITE);

  int labelX = memPosFieldP1.x + memPosFieldP1.w + 6;
  tft.fillRect(labelX, memPosFieldP1.y, 315 - labelX, memPosFieldP1.h, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("mem pos", labelX, memPosFieldP1.y + memPosFieldP1.h / 2);
  tft.setTextDatum(TL_DATUM);

  drawMemMatchInfo(lastFoundMemPos);
}

void drawFreqFieldP1() {
  tft.fillRoundRect(freqFieldP1.x, freqFieldP1.y, freqFieldP1.w, freqFieldP1.h, 6, TFT_NAVY);
  tft.drawRoundRect(freqFieldP1.x, freqFieldP1.y, freqFieldP1.w, freqFieldP1.h, 6, TFT_WHITE);
  tft.setTextColor(TFT_YELLOW, TFT_NAVY);
  tft.setTextSize(2);
  tft.setTextDatum(ML_DATUM);
  char buf[16];
  snprintf(buf, sizeof(buf), "%d", freqValueP1);
  tft.drawString(buf, freqFieldP1.x + 6, freqFieldP1.y + freqFieldP1.h / 2);
  tft.setTextDatum(TL_DATUM);

  // Label "kHz" 
  int labelX = freqFieldP1.x + freqFieldP1.w + 6;
  tft.fillRect(labelX, freqFieldP1.y, btnListP1.x - labelX - 4, freqFieldP1.h, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("kHz", labelX, freqFieldP1.y + freqFieldP1.h / 2);
  tft.setTextDatum(TL_DATUM);
}

void drawScreenP1() {
  tft.fillScreen(TFT_BLACK);
  lastKnobX = -1;

  drawButton(btnP2, TFT_BLUE, TFT_WHITE, 2);

  drawButton(btnMin, TFT_DARKGREEN, TFT_WHITE, 3);
  drawButton(btnMan, manMode ? TFT_YELLOW : TFT_DARKGREEN, manMode ? TFT_BLACK : TFT_WHITE, 3);
  drawButton(btnMax, TFT_DARKGREEN, TFT_WHITE, 3);

  drawSliderTrack();
  drawSliderKnob();

  drawPosFieldP1();
  drawMemPosFieldP1();

  drawFreqFieldP1();
  drawButton(btnListP1, TFT_PURPLE, TFT_WHITE, 1);
  drawButton(btnSetP1, TFT_ORANGE, TFT_BLACK, 2);
}

// Holding down MIN/MAX moves the servo in increments of
// chunkStepsFromSpeed() steps at a fixed tick rate (TICK_MS) as long as the button
// remains pressed. At low speeds, the increment is small (fine adjustment); at
// high speeds, it is large (rapid movement)—using a single formula for the entire
// slider range, with no special cases. Since the button state is checked on every
// tick, releasing it always takes effect immediately (within one increment).
void handleMinMaxHold(int dir, Btn btn) {
  int chunk = chunkStepsFromSpeed(speedPercent);
  Serial.printf("[MIN/MAX] start on step %d, speed %.1f%%, chunk %d steps (%.1f%%)\n",
                servoPos, speedPercent, chunk, chunk / 10.0f);

  const unsigned long DRAW_INTERVAL_MS = 40;
  unsigned long lastDrawMs = 0;
  bool stillTouched = true;
  int startPos = servoPos;
  unsigned long startMs = millis();

  while (stillTouched &&
         ((dir < 0 && servoPos > 0) || (dir > 0 && servoPos < SERVO_MAX_STEP))) {
    int remaining = (dir < 0) ? servoPos : (SERVO_MAX_STEP - servoPos);
    int step = min(chunk, remaining);
    servoPos += dir * step;
    servoWriteCurrent();

    unsigned long now = millis();
    if (now - lastDrawMs >= DRAW_INTERVAL_MS) {
      drawPosValue();
      lastDrawMs = now;
    }

    delay(TICK_MS);

    int tx2, ty2;
    stillTouched = getTouchPoint(tx2, ty2) && hit(btn, tx2, ty2);
  }
  drawPosValue(); // always show the final position

  unsigned long elapsed = millis() - startMs;
  int stepsMoved = abs(servoPos - startPos);
  Serial.printf("[MIN/MAX] stopped at step %d, %d steps in %lu ms\n", servoPos, stepsMoved, elapsed);
}

void handleScreenP1(bool touched, int tx, int ty, bool wasTouched) {
  bool edge = touched && !wasTouched;

  // MAN mode: potentiometer controls the servo. This operates independently of the
  // touchscreen, so every time this screen is processed and MAN is
  // enabled, the potentiometer is read (even if nothing is being touched).


  if (manMode) {
    handleManualPotentiometer();
  }

  // navigation to screen 2 knob
  if (edge && hit(btnP2, tx, ty)) {
    currentScreen = SCREEN_P2;
    drawScreenP2();
    return;
  }

  // MAN – switch on/off (potentiometer as a second control method)
  if (edge && hit(btnMan, tx, ty)) {
    manMode = !manMode;
    if (manMode) {
      manRefStep = servoPos; // Current position becomes the "middle" for the pot – no jump upon switching on.
    }
    drawButton(btnMan, manMode ? TFT_YELLOW : TFT_DARKGREEN, manMode ? TFT_BLACK : TFT_WHITE, 3);
    return;
  }

  // MIN – step-by-step counter-clockwise as long as pressed (0.1%)
  if (touched && hit(btnMin, tx, ty)) {
    handleMinMaxHold(-1, btnMin);
    return;
  }

  // MAX – step-by-step clockwise rotation (0.1%) as long as pressed
  if (touched && hit(btnMax, tx, ty)) {
    handleMinMaxHold(1, btnMax);
    return;
  }

  // Slider speed (drag control, 0.5% increments)
  if (touched && tx >= sliderX && tx <= sliderX + sliderW &&
      ty >= sliderY - 15 && ty <= sliderY + sliderH + 15) {
    float raw = speedFromSliderFrac((float)(tx - sliderX) / (float)sliderW);
    float newSpeed = round(raw * 2.0f) / 2.0f;   // afronden op 0,5%
    newSpeed = constrain(newSpeed, 1.0f, 100.0f);
    if (fabs(newSpeed - speedPercent) >= 0.25f) {
      speedPercent = newSpeed;   // drawSliderKnob() clears the old indicator itself.
      drawSliderKnob();
      Serial.printf("[Speed] changed to %.1f%% (chunk %d steps / %.1f%% per tick)\n", speedPercent, chunkStepsFromSpeed(speedPercent), chunkStepsFromSpeed(speedPercent) / 10.0f);
    }
    return;
  }

  // Freq (kHz) input field
  if (edge && hit(freqFieldP1, tx, ty)) {
    freqValueP1 = (int)numericKeypad("Freq (Khz) invoeren (10000-30000)", 10000, 30000, freqValueP1);
    lastFoundMemPos = -1; // New goal entered; old match no longer valid.
    drawScreenP1();
    return;
  }

  // List – select a frequency from the previously saved values.
  if (edge && hit(btnListP1, tx, ty)) {
    long chosen = chooseStoredFreq(freqValueP1);
    if (chosen != freqValueP1) {
      freqValueP1 = (int)chosen;
      lastFoundMemPos = -1; // New goal selected; old match no longer valid.
    }
    drawScreenP1();
    return;
  }

  // SET – move the servo to the saved percentage corresponding to this frequency
  if (edge && hit(btnSetP1, tx, ty)) {
    int foundMemPos = -1;
    int percentStep = findPercentForFreq(freqValueP1, foundMemPos);
    if (percentStep >= 0) {
      moveServoToPreset(percentStep);
      lastFoundMemPos = foundMemPos;
      drawMemMatchInfo(lastFoundMemPos);
    } else {
      lastFoundMemPos = -1;
      tft.fillRect(15, freqFieldP1.y + freqFieldP1.h + 4, 290, 16, TFT_BLACK);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setTextSize(1);
      tft.drawString("No Calibration found for this frequency!", 15, freqFieldP1.y + freqFieldP1.h + 4);
      delay(1200);
      tft.fillRect(15, freqFieldP1.y + freqFieldP1.h + 4, 290, 16, TFT_BLACK);
    }
    return;
  }
}

// ======================================================================
// SCREEN 2 - lay-out
// ======================================================================
Btn memPosField  = {205,  42, 100, 32, ""};
Btn servoPosBox  = {205,  92, 100, 32, ""};
Btn freqFieldP2  = {205, 140, 100, 32, ""};
Btn btnSetP2     = {15,  182,  90, 36, "SET"};
Btn btnP1        = {15,   5, 290, 30, "MAIN SCREEN"};

// Draws a label at the start of a row, with an input/display box at the far
// right of the same line. valueText is the pre-formatted text (e.g., "123"
// or "90.0") that appears in the box. labelSize/valueSize determine the
// text size (2 = same size as the text on the "MAIN SCREEN" button).
void drawLabelFieldRow(const char* label, Btn b, uint16_t boxBg, uint16_t textColor, const char* valueText, int labelSize = 2, int valueSize = 2) {
  tft.fillRect(15, b.y, b.x - 15 - 4, b.h, TFT_BLACK); // erase the label area before the box
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(labelSize);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(label, 15, b.y + b.h / 2);

  tft.fillRoundRect(b.x, b.y, b.w, b.h, 6, boxBg);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 6, TFT_WHITE);
  tft.setTextColor(textColor, boxBg);
  tft.setTextSize(valueSize);
  tft.drawString(valueText, b.x + 8, b.y + b.h / 2);
  tft.setTextDatum(TL_DATUM);
}

// Checks whether a MEM POS address already contains data and, if desired, returns the stored frequency and percentage.
bool memPosInUse(int memPos, int &outFreq, int &outPercentStep) {
  char keyF[10], keyA[10];
  snprintf(keyF, sizeof(keyF), "p%df", memPos);
  snprintf(keyA, sizeof(keyA), "p%da", memPos);
  if (prefs.isKey(keyF) && prefs.isKey(keyA)) {
    outFreq = prefs.getInt(keyF, -1);
    outPercentStep = prefs.getInt(keyA, -1);
    return true;
  }
  return false;
}

// "mem pos" row: the slot itself indicates its status via its color (red = in
// use, green = free, navy blue = not filled in). If the address is in use,
// the stored frequency/percentage appears in small letters below the label.
void drawMemPosRow() {
  char buf[32];
  if (memPosValue > 0) {
    snprintf(buf, sizeof(buf), "%d", memPosValue);
  } else {
    buf[0] = '\0';
  }

  int freq, percentStep;
  uint16_t boxBg;
  bool inUse = false;
  if (memPosValue <= 0) {
    boxBg = TFT_NAVY; // nothing filled in yet
  } else if (memPosInUse(memPosValue, freq, percentStep)) {
    boxBg = TFT_RED;  // occupied
    inUse = true;
  } else {
    boxBg = TFT_DARKGREEN; // free
  }

  drawLabelFieldRow("MEM POSITION", memPosField, boxBg, TFT_WHITE, buf);

  // Detail line below the label: show only when occupied.
  
  const int dx = 15, dy = memPosField.y + memPosField.h + 6, dw = 290, dh = 12;
  tft.fillRect(dx, dy, dw, dh, TFT_BLACK);
  if (inUse) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    snprintf(buf, sizeof(buf), "Freq %d, %.1f%%", freq, percentStep / 10.0f);
    tft.drawString(buf, dx, dy);
  }
}

void drawServoPosRow() {
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", servoPercent());
  drawLabelFieldRow("CAP POSITION", servoPosBox, TFT_DARKGREY, TFT_WHITE, buf);
}

void drawFreqRow() {
  char buf[16];
  if (freqValueP2 > 0) {
    snprintf(buf, sizeof(buf), "%d", freqValueP2);
  } else {
    buf[0] = '\0';
  }
  drawLabelFieldRow("FREQUENCY", freqFieldP2, TFT_NAVY, TFT_YELLOW, buf, 2, 2);
}

void drawScreenP2() {
  tft.fillScreen(TFT_BLACK);

  drawMemPosRow();
  drawServoPosRow();
  drawFreqRow();

  drawButton(btnSetP2, TFT_ORANGE, TFT_BLACK, 2);
  drawButton(btnP1,    TFT_BLUE,   TFT_WHITE, 2);
}

void handleScreenP2(bool touched, int tx, int ty, bool wasTouched) {
  bool edge = touched && !wasTouched;

  if (edge && hit(memPosField, tx, ty)) {
    memPosValue = (int)numericKeypad("Insert mem position (1-180)", 1, 180, memPosValue);
    drawScreenP2();
    return;
  }

  if (edge && hit(freqFieldP2, tx, ty)) {
    freqValueP2 = (int)numericKeypad("Insert frequency (10000-30000)", 10000, 30000, freqValueP2);
    drawScreenP2();
    return;
  }

  if (edge && hit(btnSetP2, tx, ty)) {
    if (memPosValue <= 0 || freqValueP2 <= 0) {
      tft.fillRect(15, 228, 290, 10, TFT_BLACK);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setTextSize(1);
      tft.drawString("Enter mem pos and freq first!", 15, 228);
      delay(900);
      tft.fillRect(15, 228, 290, 10, TFT_BLACK);
      return;
    }

    saveFreqAtMemPos(memPosValue, freqValueP2, servoPos);

    // Clear fields after saving (0 = displayed as empty)
    memPosValue = 0;
    freqValueP2 = 0;
    drawMemPosRow();
    drawFreqRow();

    tft.fillRect(15, 228, 290, 10, TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(1);
    tft.drawString("Saved!", 15, 228);
    delay(500);
    tft.fillRect(15, 228, 290, 10, TFT_BLACK);
    return;
  }

  if (edge && hit(btnP1, tx, ty)) {
    currentScreen = SCREEN_P1;
    drawScreenP1();
    return;
  }
}

// ======================================================================
// SETUP / LOOP
// ======================================================================
void setup() {
  Serial.begin(115200);

  // Touch (separate SPI bus from the display, standard CYD wiring)
  touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  // Display
  tft.init();
  tft.setRotation(1);   // landscape, 320x240
  // NOTE: If the colors remain incorrect (white background, strange hues), check
  // the driver/inversion settings of the TFT_eSPI library (User_Setup.h).
  tft.fillScreen(TFT_BLACK);

  // Servo - directly via ESP32 LEDC hardware PWM on SERVO_PIN
  // Non-volatile memory (open first: the last servo position must be loaded
  // before the servo is driven)
  prefs.begin("freqtab", false);
  loadLastServoPos();

  ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, LEDC_RESOLUTION_BITS);
  servoWriteCurrent();   // servo directly to the last saved position

  // Potentiometer for MAN-mode
  analogReadResolution(12);                    // 0-4095
  analogSetPinAttenuation(POT_PIN, ADC_11db);   // full 0–3.3V range
  pinMode(POT_PIN, INPUT);

  drawScreenP1();
}

void loop() {
  int tx, ty;
  bool touched = getTouchPoint(tx, ty);

  if (currentScreen == SCREEN_P1) {
    handleScreenP1(touched, tx, ty, prevTouched);
  } else {
    handleScreenP2(touched, tx, ty, prevTouched);
  }

  prevTouched = touched;

  saveServoPosIfSettled();   // Save the last servo position as soon as the servo stops moving.

  // A fixed pause occurs only when nothing is being touched or moved –
  // during active servo movement, the speed setting alone determines the pace.
  if (!touched) {
    delay(15);
  }
}
