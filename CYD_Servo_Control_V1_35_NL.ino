/*
  =====================================================================
  CYD Servo/Frequentie Tuner
  =====================================================================
  Platform:
    - Cheap Yellow Display (CYD, board ESP32-2432S028R) met ESP-WROOM-32
    - TFT display ILI9341, 2.8", 240x320, resistief touch XPT2046
    - 8120MG digital servo met 270 graden slag, RECHTSTREEKS aangestuurd
      vanaf ESP32 GPIO27 via de ingebouwde LEDC-hardware-PWM (geen
      externe I2C-driver - servo zelf extern gevoed met 5V, GND
      gemeenschappelijk met de ESP32!). Via een 1:3 tandwielover-
      brenging levert de volledige 270 graden servoslag 0-100% op de
      uitgaande as (de condensator-vlinder) - alle posities in deze sketch
      ("Servo pos", MIN/MAX, mem pos-opslag, MAN-mode) zijn dan ook het
      UITGAANDE (vlinder-)percentage, 0-100%.
    - De kunstof tandriem zorgt tevens voor een goede isolatie tussen de hoge spanning
      van de condensator en de servo.
    - Zorg ervoor dat er geen speling aanwezig is op de vlinder as en tussen de tandwielen.
      Speling zal zorgen voor hysteresis en slechte repeatability.
    - Het instellen van de presets moet altijd van de laagste rotatie waarde naar de hoogste
      rotatie waarde geschieden i.v.m. hysteresis correctie. Indien later een preset wordt gekozen, 
      zal de servo eerst 4,4% onder de preset waarde aannemen om vervolgens naar 
      de preset waarde te gaan. Hiermee wordt de hysteresis voor een groot deel voorkomen.    
    - Servo-signaaldraad -> GPIO27 (SERVO_PIN)
      De meeste 5V-servo's accepteren een 3,3V-stuursignaal prima, maar controleer dit voor jouw servo.
    - Potentiometer (MAN-mode, tweede manier om de servo te bewegen):
      middenpin (wiper) -> GPIO35 (POT_PIN), buitenste pinnen -> 3V3 en GND.
      GPIO35 is een input-only ADC1-pin die op de meeste CYD-borden vrij
      beschikbaar is (vaak op de "P3"-header) - controleer dit voor jouw
      exemplaar en pas POT_PIN aan indien nodig.

  Benodigde libraries (Library Manager):
    - TFT_eSPI                    (Bodmer)
    - XPT2046_Touchscreen         (Paul Stoffregen)
    - Preferences                 (ingebouwd in ESP32 core)
  BELANGRIJK - ESP32-Arduino core versie (LEDC-API):
    Deze sketch gebruikt de LEDC-API van ESP32-Arduino core 3.x:
    ledcAttach(pin, freq, resolutie) / ledcWrite(pin, duty).

  BELANGRIJK - TFT_eSPI configuratie:
    In de library-map van TFT_eSPI moet het bestand User_Setup.h (of
    User_Setup_Select.h) zo ingesteld zijn dat onderstaande pinnen
    actief zijn (standaard CYD-bedrading):

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

  Opstarten:
    - De laatste servostand (sleutel "lastpos") en de laatste snelheidsinstelling
      (sleutel "speed") worden in NVS bewaard, pas 2 s nadat ze niet meer veranderen,
      en bij het opstarten teruggezet i.p.v. de standaard 50%.
      Bij een leeg geheugen (eerste keer) start alles op 50%.
    - Snelheidsslider is niet-lineair: de eerste helft van de slider dekt 1-25%
      (fijne, trage regeling), de tweede helft 25-100%.

  Werking (samengevat):
    - "mem pos" (1-180) is een presetadres in NVS - het is NIET
      gelijk aan het servopercentage. Onder dat adres worden twee waarden
      bewaard: de Freq-waarde en de actuele servostand/vlinder (Cap positie, in %) op
      het moment van opslaan.
    - Scherm 2: zet de servo/vlinder handmatig (via scherm 1) in de gewenste
      fysieke stand, ga naar scherm 2, kies een vrij "mem pos" nummer (of overschrijf een bestaande positie), 
      vul de bijbehorende "Freq" in en druk op "set". Freq en de op dat
      moment actuele Servo/vlinder positie worden samen opgeslagen onder dat
      mem pos adres.
    - Scherm 1: via "Freq (Khz)" en "set" wordt de volledige tabel
      doorzocht op de (dichtstbijzijnde) opgeslagen Freq, en de servo
      beweegt naar het daarbij opgeslagen servo/vlinder percentage (niet naar het
      mem pos-getal zelf).
    - Knop "MAN" (tussen MIN en MAX): schakelt de potentiometer als
      besturing in/uit. Bij het inschakelen wordt de actuele servostand
      vastgelegd als referentiepunt (dus geen sprong) zolang de potentiometer in het midden staat. 
      Zolang MAN actief is (rood), bepaalt de stand van de potentiometer rechtstreeks het
      servo/vlinder percentage (proportionele besturing, geen snelheidsbesturing) t.o.v.
      dat referentiepunt: pot in het midden = servo op de referentiestand,
      pot naar rechts (CW) = servo/vlinder verder rechtsom, pot naar links (CCW) =
      servo/vlinder verder linksom - en de servo blijft daar staan zodra de potmeter
      niet meer verder wordt gedraaid. Hoe ver de servo/vlinder vanaf de
      referentiestand kan komen bij volledige uitslag van de pot, wordt
      bepaald door de snelheids-slider (bij 100% tot 50% naar elke
      kant, bij een lagere instelling proportioneel minder). Rond het midden zit een dode
      zone zodat de servo niet "zoekt" als de pot niet perfect gecentreerd
      staat.
  =====================================================================
*/

#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Preferences.h>

// --------------------------------------------------------------------
// Pin definities
// --------------------------------------------------------------------
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

// Servo - rechtstreeks op een ESP32 GPIO via de LEDC-hardware-PWM
#define SERVO_PIN 22   // Servo 5V + GND EXTERN gevoed, GND gemeenschappelijk met ESP32!

// Ruwe touch-kalibratiewaarden - pas eventueel aan voor jouw exemplaar
#define TS_MINX 200
#define TS_MAXX 3900
#define TS_MINY 200
#define TS_MAXY 3900

// Potentiometer voor MAN-mode (tweede manier om de servo te bewegen)
#define POT_PIN            35    // ADC1_CH7, input-only - controleer/pas aan voor jouw CYD-exemplaar
#define POT_DEADBAND        0.06f // dode zone rond het midden (fractie van de volledige slag) - voorkomt
                                   // dat de servo rond het midden blijft "zoeken" door kleine ruis

// --------------------------------------------------------------------
// Objecten
// --------------------------------------------------------------------
SPIClass touchSPI = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
TFT_eSPI tft = TFT_eSPI();
Preferences prefs;

// --------------------------------------------------------------------
// Status / variabelen
// --------------------------------------------------------------------
enum Screen { SCREEN_P1, SCREEN_P2 };
Screen currentScreen = SCREEN_P1;

int servoPos      = 500;    // actuele servostand (vlinder) in STAPPEN VAN 0,1 PROCENT (0-1000 = 0-100%; 500 = 50,0% = midden)
int lastFoundMemPos = -1;   // mem pos die hoorde bij de laatst gevonden Freq-match (-1 = nog geen match)
int memPosValue   = 0;      // P2: "mem pos" (1-180) - 0 = nog leeg (start leeg, net als na het opslaan)
int freqValueP2   = 0;      // P2: "Freq" (10000-30000) -> wordt opgeslagen - 0 = nog leeg
int freqValueP1   = 10000;  // P1: "Freq (Khz)" (10000-30000) -> opzoeken
float speedPercent = 50.0f; // P1: "snelheid" (1-100%, in stappen van 0,1%)

int lastKnobX = -1;         // voor het wissen van de oude slider-indicatie

bool prevTouched = false;
bool manMode = false;       // P1: MAN-knop actief -> potentiometer bestuurt de servo
int manRefStep = 500;       // servostand (in stappen) die als "midden" geldt voor de pot -
                             // wordt bij het INSCHAKELEN van MAN gelijk gezet aan de actuele
                             // servoPos, zodat de pot een OFFSET is t.o.v. de stand van dat moment

// --------------------------------------------------------------------
// UI helper structuur
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
// Touch inlezen
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

// Voorwaartse declaratie (functie zelf staat verderop, bij scherm 2)
bool memPosInUse(int memPos, int &outFreq, int &outPercentStep);

// ======================================================================
// NUMERIEK TOETSENBORD (modaal invoerscherm)
// ======================================================================
Btn kp_digits[12] = {
  {47,  45, 70, 35, "1"}, {125, 45, 70, 35, "2"}, {203, 45, 70, 35, "3"},
  {47,  88, 70, 35, "4"}, {125, 88, 70, 35, "5"}, {203, 88, 70, 35, "6"},
  {47, 131, 70, 35, "7"}, {125,131, 70, 35, "8"}, {203,131, 70, 35, "9"},
  {47, 174, 70, 35, "C"}, {125,174, 70, 35, "0"}, {203,174, 70, 35, "<-"}
};
Btn kp_cancel = {47, 214, 110, 24, "Annuleer"};
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
  tft.drawString("Ongeldige waarde!", 160, 22);
  tft.setTextDatum(TL_DATUM);
}

// Blokkerend modaal toetsenbord. Geeft de ingevoerde (gevalideerde) waarde terug,
// of de oorspronkelijke waarde 'initVal' als er geannuleerd wordt.
long numericKeypad(const char* title, long minVal, long maxVal, long initVal) {
  String entry = ""; // altijd leeg starten, ongeacht de huidige waarde van het veld
  bool done = false;
  long result = initVal;

  drawKeypadBase(title);
  drawKeypadEntry(entry);

  // De aanraking die het veld opende, kan nog vastzitten op het scherm
  // (vinger nog niet losgelaten) precies op de plek waar toevallig een
  // cijfertoets van dit toetsenbord staat getekend. Start wasTouched daarom
  // op de HUIDIGE aanraakstatus i.p.v. altijd 'false', zodat die ene lopende
  // aanraking niet meteen als druk op die toets wordt gezien - pas een
  // nieuwe aanraking (na loslaten) telt als toetsdruk.
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
// LIJST MET OPGESLAGEN FREQ-WAARDEN (modaal keuzescherm)
// ======================================================================
struct FreqEntry { int memPos; int freq; int percentStep; };
FreqEntry freqListEntries[180];
int freqListCount = 0;
int freqListPage = 0;
char freqListRowText[12][16];
Btn freqListRowBtn[12];
Btn freqListBtnPrev  = {10,  205, 90, 30, "Vorige"};
Btn freqListBtnNext  = {115, 205, 90, 30, "Volgende"};
Btn freqListBtnClose = {220, 205, 90, 30, "Sluiten"};

void drawFreqListPage(long currentFreq, int totalPages) {
  const int rowsPerPage = 12;
  const int rowsPerCol = 6;
  const int colW = 150, rowH = 26;
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(TC_DATUM);
  char hdr[24];
  snprintf(hdr, sizeof(hdr), "Kies Freq (%d/%d)", freqListPage + 1, totalPages);
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

// Blokkerend keuzescherm met alle opgeslagen Freq-waarden (gesorteerd op mem pos).
// Geeft de gekozen Freq terug, of 'currentFreq' als er geannuleerd wordt / niets is opgeslagen.
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
    tft.drawString("Geen opgeslagen", 160, 100);
    tft.drawString("Freq-waarden", 160, 130);
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

  // Zelfde reden als bij numericKeypad(): de aanraking die "Lijst" opende kan
  // toevallig samenvallen met een rij/knop van dit keuzescherm. Start
  // wasTouched op de HUIDIGE aanraakstatus, niet op 'false'.
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
// NIET VLUCHTIG GEHEUGEN (Preferences / NVS)
// ======================================================================
// mem pos is een los presetadres (1-180), NIET gelijk aan het servopercentage.
// Onder dat adres worden TWEE waarden bewaard: de Freq en het actuele servo-
// percentage dat op het moment van opslaan werd weergegeven in "Servo pos".
void saveFreqAtMemPos(int memPos, int freq, int actualPercentStep) {
  char keyF[10], keyA[10];
  snprintf(keyF, sizeof(keyF), "p%df", memPos);
  snprintf(keyA, sizeof(keyA), "p%da", memPos);
  prefs.putInt(keyF, freq);
  prefs.putInt(keyA, actualPercentStep);
  Serial.printf("[opslaan] mem pos %d -> Freq %d, %.1f%%\n", memPos, freq, actualPercentStep / 10.0f);
}

// Zoekt in alle opgeslagen presets naar de Freq die het best overeenkomt met 'freq'
// en geeft het DAARBIJ OPGESLAGEN SERVOPERCENTAGE terug (niet het mem pos-getal zelf).
// Geeft -1 terug als er nog niets is opgeslagen.
int findPercentForFreq(int freq, int &foundMemPos) {
  int bestPercentStep = -1;
  int bestMemPos = -1;
  long bestDiff = 2147483647;

  Serial.printf("[zoeken] gevraagde Freq = %d\n", freq);
  for (int p = 1; p <= 180; p++) {
    char keyF[10], keyA[10];
    snprintf(keyF, sizeof(keyF), "p%df", p);
    snprintf(keyA, sizeof(keyA), "p%da", p);
    if (prefs.isKey(keyF) && prefs.isKey(keyA)) {
      int storedFreq        = prefs.getInt(keyF, -1);
      int storedPercentStep = prefs.getInt(keyA, -1);
      Serial.printf("  mem pos %d -> Freq %d, %.1f%%\n", p, storedFreq, storedPercentStep / 10.0f);
      if (storedFreq == freq) {
        Serial.printf("  -> exacte match: mem pos %d, %.1f%%\n", p, storedPercentStep / 10.0f);
        foundMemPos = p;
        return storedPercentStep; // exacte match
      }
      long diff = abs((long)storedFreq - (long)freq);
      if (diff < bestDiff) {
        bestDiff = diff;
        bestPercentStep = storedPercentStep;
        bestMemPos = p;
      }
    }
  }
  Serial.printf("[zoeken] beste (niet-exacte) match: mem pos %d, %.1f%%\n", bestMemPos, bestPercentStep / 10.0f);
  foundMemPos = bestMemPos;
  return bestPercentStep;
}

// ======================================================================
// SERVO BESTURING
// ======================================================================
// De servo wordt aangestuurd via microseconden i.p.v. hele procenten, zodat
// een resolutie van 0,1% per stap mogelijk is.
//
// Deze servo heeft een fysieke slag van 270 graden (pulsbreedte 500-2500 us
// over het VOLLEDIGE bereik). Via een 1:3 tandriem overbrenging levert dat
// 0-100% op de uitgaande as (de condensator-vlinder): de servo draait dus
// 3x zo ver als de vlinder. Omdat de volledige 270 graden servoslag exact
// overeenkomt met de volledige 500-2500 us pulsbreedte, is de puls nog
// steeds recht evenredig met het UITGAANDE (vlinder-)percentage. Het bereik
// is verdeeld in 1000 stappen van 0,1% (0-100%): stap 0 = 0,0% (vlinder),
// stap 1000 = 100,0% (vlinder). Omdat de volledige pulsbreedte (2000 us)
// over die 1000 stappen wordt verdeeld, komt elke stap overeen met exact
// 2000 / 1000 = 2 us pulsbreedteverschil - de servo wordt dus per 2 us
// aangestuurd.
//
// LET OP - haalbare resolutie via de ESP32 LEDC-hardware-PWM:
// LEDC verdeelt de PWM-periode (bij 50 Hz dus 20000 us) in 2^LEDC_RESOLUTION_BITS
// telstappen. Met LEDC_RESOLUTION_BITS=16 is dat 65536 stappen over 20000 us,
// dus ca. 0,31 us per telstap - ruim fijner dan de 2000/1000 = 2 us die je
// nodig hebt voor een echte 0,1%-stap. Elke software-stap geeft hier dus
// ook daadwerkelijk een andere fysieke pulsbreedte.

#define SERVO_MIN_US   500   // pulsbreedte bij 0% vlinder (= 0 graden servo)
#define SERVO_MAX_US   2500  // pulsbreedte bij 100% vlinder (= 270 graden servo, volledige slag)
#define SERVO_MAX_STEP 1000  // 1000 stappen = 0,1% (vlinder) per stap, over 0-100% -> 2 us per stap
#define SERVO_PWM_FREQ 50           // PWM-frequentie voor de servo (Hz) - standaard voor analoge/digitale RC-servo's
#define LEDC_RESOLUTION_BITS 16     // duty-resolutie van de ESP32 LEDC-timer (16 bit = 65536 stappen/periode)
#define LEDC_MAX_DUTY ((1UL << LEDC_RESOLUTION_BITS) - 1)
/* Aanpassen bij een andere servo-slag of overbrengingsverhouding:
Het volledige uitgaande bereik wordt altijd als 0-100% weergegeven, in 1000
stappen van 0,1%. SERVO_MIN_US/SERVO_MAX_US blijven de pulsbreedte bij resp.
0% en 100% van de volledige fysieke servo-slag (hier dus 500 en 2500 us voor
0-270 graden servo). Bij een andere servo-slag hoeven alleen SERVO_MIN_US/
SERVO_MAX_US aangepast te worden - SERVO_MAX_STEP blijft 1000 zolang een
resolutie van 0,1% gewenst is.
*/
// servoPercent() geeft het UITGAANDE (vlinder-)percentage terug (0-100%),
// niet de fysieke servohoek - die is 3x zo groot door de 1:3 overbrenging.
float servoPercent() {
  return servoPos / 10.0f;
}

// Stuurt de servo naar de huidige servoPos (in stappen van 0,1%), via de
// ESP32 LEDC-hardware-PWM. Rekent eerst de servostap om naar een pulsbreedte
// in microseconden (zelfde als voorheen), en die vervolgens naar een
// LEDC-duty-waarde (0 .. LEDC_MAX_DUTY) passend bij SERVO_PWM_FREQ.
void servoWriteCurrent() {
  int step = constrain(servoPos, 0, SERVO_MAX_STEP);
  long us = SERVO_MIN_US + (long)(SERVO_MAX_US - SERVO_MIN_US) * step / SERVO_MAX_STEP;

  float periodUs = 1000000.0f / SERVO_PWM_FREQ;                 // 20000 us bij 50 Hz
  uint32_t duty = (uint32_t)((us / periodUs) * LEDC_MAX_DUTY + 0.5f);

  ledcWrite(SERVO_PIN, duty);
}

// --------------------------------------------------------------------
// Laatste servostand onthouden (over uitschakelen heen)
// --------------------------------------------------------------------
// De actuele servoPos wordt in NVS bewaard onder de sleutel "lastpos", zodat
// de servo na het opstarten weer op de laatste stand komt i.p.v. op 50%.
// Om slijtage van het flashgeheugen te beperken wordt NIET bij elke stap
// geschreven, maar pas als de servo SERVO_SAVE_DELAY_MS stil heeft gestaan
// en de stand afwijkt van de laatst opgeslagen waarde.
#define SERVO_SAVE_DELAY_MS 2000UL
int lastSavedServoPos = -1;           // laatst in NVS geschreven stand
int lastSeenServoPos  = -1;           // stand bij de vorige controle
unsigned long servoLastChangeMs = 0;  // tijdstip van de laatste wijziging
int lastSavedSpeedX10 = -1;           // laatst in NVS geschreven snelheid (in 0,1%)
int lastSeenSpeedX10  = -1;           // snelheid bij de vorige controle
unsigned long speedLastChangeMs = 0;  // tijdstip van de laatste snelheidswijziging

void loadLastServoPos() {
  int stored = prefs.getInt("lastpos", 500);        // 500 = 50,0% als er nog niets is opgeslagen
  servoPos = constrain(stored, 0, SERVO_MAX_STEP);
  lastSavedServoPos = servoPos;
  lastSeenServoPos  = servoPos;
  manRefStep = servoPos;

  int storedSpeed = prefs.getInt("speed", 500);      // 500 = 50,0% als er nog niets is opgeslagen
  speedPercent = constrain(storedSpeed, 10, 1000) / 10.0f;
  lastSavedSpeedX10 = lastSeenSpeedX10 = (int)(speedPercent * 10.0f + 0.5f);
  Serial.printf("[opstart] laatste snelheid geladen: %.1f%%\n", speedPercent);
  Serial.printf("[opstart] laatste servostand geladen: stap %d (%.1f%%)\n", servoPos, servoPos / 10.0f);
}

void saveServoPosIfSettled() {
  if (servoPos != lastSeenServoPos) {               // servo is (nog) in beweging
    lastSeenServoPos = servoPos;
    servoLastChangeMs = millis();
  } else if (servoPos != lastSavedServoPos && millis() - servoLastChangeMs >= SERVO_SAVE_DELAY_MS) {
    prefs.putInt("lastpos", servoPos);
    lastSavedServoPos = servoPos;
    Serial.printf("[opslaan] laatste servostand: stap %d (%.1f%%)\n", servoPos, servoPos / 10.0f);
  }

  // Snelheid (slider) op dezelfde manier bewaren
  int speedX10 = (int)(speedPercent * 10.0f + 0.5f);
  if (speedX10 != lastSeenSpeedX10) {
    lastSeenSpeedX10 = speedX10;
    speedLastChangeMs = millis();
  } else if (speedX10 != lastSavedSpeedX10 && millis() - speedLastChangeMs >= SERVO_SAVE_DELAY_MS) {
    prefs.putInt("speed", speedX10);
    lastSavedSpeedX10 = speedX10;
    Serial.printf("[opslaan] laatste snelheid: %.1f%%\n", speedPercent);
  }
}

// --------------------------------------------------------------------
// Snelheidsmodel: vaste tick, variabele brokgrootte
// --------------------------------------------------------------------
// Elke servoWriteCurrent()/ledcWrite()-aanroep kost door de lage PWM-
// frequentie (50 Hz -> 20 ms periode) hoe dan ook al zo'n 20-30 ms,
// ONGEACHT hoeveel software-vertraging we zelf toevoegen. Een model dat
// "snelheid" vertaalt naar de VERTRAGING TUSSEN vaste stapjes van 0,1%
// loopt daarom altijd tegen die hardware-bodem aan: bij 1000 stappen
// kost een volledige slag minimaal 1000 * ~25 ms, wat de instelling ook is.
//
// In plaats daarvan wordt hier de GROOTTE van elke stap gevarieerd, op een
// vaste tick (TICK_MS, rond de hardware-bodem): bij lage snelheid een
// piepklein brokje (0,1%, voor precieze fijnafstelling), bij hoge
// snelheid een groot brokje (4,4%, voor een snelle slag) - lineair
// ertussenin. Zo blijft de tijd tussen updates altijd gelijk (en dus altijd
// haalbaar voor de hardware), en bepaalt alleen de brokgrootte hoe snel de
// servo per tijdseenheid vordert. Wordt gebruikt door de preset-beweging
// (moveServoTo), het ingedrukt houden van MIN/MAX (handleMinMaxHold) en de
// potentiometer in MAN-mode (handleManualPotentiometer), zodat alle drie
// dezelfde, voorspelbare snelheidscurve delen.
// MAX_CHUNK_STEPS is zo gekozen (44 stappen = 4,4% van het 0-100% bereik)
// dat de FYSIEKE (vlinder-)snelheid exact gelijk blijft aan de oude
// 0,1 graad-resolutie (waar 40 stappen = 4,0 graden was op het oude
// 0-90 graden bereik: ook daar 40/900 = 4,4% van het bereik per tick).
const unsigned long TICK_MS = 25;      // vaste tijd tussen updates, rond de LEDC-hardwarebodem
const int MIN_CHUNK_STEPS   = 1;       // 0,1% per tick bij 1% snelheid (fijnste besturing)
const int MAX_CHUNK_STEPS   = 44;      // 4,4% per tick bij 100% snelheid (snelste besturing, fysiek gelijk aan voorheen)

int chunkStepsFromSpeed(float percent) {
  float frac = (percent - 1.0f) / 99.0f;  // 0.0 bij 1%, 1.0 bij 100%
  frac = constrain(frac, 0.0f, 1.0f);
  int chunk = (int)(MIN_CHUNK_STEPS + frac * (MAX_CHUNK_STEPS - MIN_CHUNK_STEPS) + 0.5f);
  return constrain(chunk, MIN_CHUNK_STEPS, MAX_CHUNK_STEPS);
}

// Beweegt de servo stapsgewijs (in brokken van chunkStepsFromSpeed() stappen,
// op een vaste tick TICK_MS) naar de doelpositie, met live update van de
// "Pos"-indicator. 'target' is uitgedrukt in stappen (0-1000, 0,1%/stap).
void moveServoTo(int target) {
  target = constrain(target, 0, SERVO_MAX_STEP);
  int chunk = chunkStepsFromSpeed(speedPercent);
  Serial.printf("[servo] beweeg van stap %d (%.1f%%) naar stap %d (%.1f%%), brokgrootte %d stappen (%.1f%%)\n",
                servoPos, servoPercent(), target, target / 10.0f, chunk, chunk / 10.0f);

  int dir = (target > servoPos) ? 1 : -1;

  // Het TFT-display via SPI herschrijven kost een paar ms per keer. Bij elke
  // tick verversen zou dat oplopen; daarom de display-update beperken tot
  // maximaal eens per DRAW_INTERVAL_MS, los van de tickfrequentie.
  const unsigned long DRAW_INTERVAL_MS = 40;
  unsigned long lastDrawMs = 0;

  while (servoPos != target) {
    int remaining = abs(target - servoPos);
    int step = min(chunk, remaining);   // laatste brok niet voorbij het doel schieten
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
// Hysteresis-compensatie bij het oproepen van een preset
// --------------------------------------------------------------------
// Door speling (backlash) in de tandwieloverbrenging komt de vlinder niet
// exact op dezelfde positie terecht als de vorige beweging van "hoog naar
// laag" kwam i.p.v. van "laag naar hoog" (of andersom) - de speling wordt
// pas aan het einde van de beweging "opgenomen", en dat gebeurt aan een
// andere kant van de tandwielen afhankelijk van de bewegingsrichting.
// Om dit te compenseren wordt bij het ophalen van een preset de
// EINDNADERING altijd vanuit DEZELFDE richting gedaan: de servo gaat eerst
// naar een tussenstop HYSTERESIS_COMP_PERCENT % ONDER de doelwaarde, en
// beweegt vandaar pas naar de uiteindelijke doelwaarde. Zo is de laatste
// deelbeweging altijd omhoog (laag -> hoog), ongeacht of de servo daarvoor
// hoger of lager stond dan het doel, en wordt de speling steeds op
// dezelfde manier "opgenomen" -> betere repeatability.
// HYSTERESIS_COMP_PERCENT (4,4%) is functioneel gelijk aan de vorige
// 4 graden op het oude 0-90 graden bereik (4/90 = 4,4% van het bereik).
#define HYSTERESIS_COMP_PERCENT 4.4f                                // % onder de doelwaarde voor de tussenstop
#define HYSTERESIS_COMP_STEPS  ((int)(HYSTERESIS_COMP_PERCENT * 10)) // omgerekend naar stappen (0,1% per stap)

// Beweegt de servo naar 'target' (in stappen) MET hysteresis-compensatie:
// eerst naar (target - HYSTERESIS_COMP_STEPS), daarna naar target. Wordt
// gebruikt bij het oproepen van een preset (Scherm 1, "set"); voor overige
// bewegingen (MIN/MAX, MAN-mode) blijft de gewone moveServoTo() in gebruik.
void moveServoToPreset(int target) {
  target = constrain(target, 0, SERVO_MAX_STEP);

  // Alleen compenseren als het nieuwe doel LAGER is dan de huidige stand:
  // dan komt de laatste deelbeweging van "onder het doel" omhoog naar het
  // doel, en wordt de speling steeds op dezelfde manier opgenomen. Staat de
  // servo al lager dan (of gelijk aan) het doel, dan is de laatste
  // deelbeweging toch al omhoog en is de tussenstop niet nodig.
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
// POTENTIOMETER (MAN-mode) - tweede manier om de servo te besturen,
// naast de MIN/MAX-knoppen.
// ======================================================================

// Leest de potentiometer en geeft een gecentreerde fractie terug:
// -1.0 = helemaal linksom, 0.0 = midden, +1.0 = helemaal rechtsom.
// Een paar keer bemonsteren dempt ruis van de ADC.
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

// Wordt elke loop-iteratie aangeroepen zolang MAN actief is (scherm 1).
// De pot werkt hier als een PROPORTIONELE POSITIE-besturing (geen snelheids-
// besturing meer): de stand van de pot bepaalt rechtstreeks een doelwaarde, en
// de servo beweegt daarheen en blijft daar staan - geen continue beweging
// meer zolang de pot in dezelfde stand blijft.
//   - Pot in het midden (binnen de dode zone)      -> doel = manRefStep (de
//     servostand op het moment dat MAN werd ingeschakeld - dus GEEN sprong
//     naar 50% als de servo ergens anders stond)
//   - Pot helemaal rechtsom (CW, zelfde kant MAX)   -> doel = manRefStep +
//     maximale uitwijking, waarbij de MAXIMALE uitwijking wordt bepaald
//     door de snelheids-slider: bij 100% is dat 50% (de helft van
//     het volledige 0-100% vlinderbereik), bij bv. 50% nog maar 25%
//   - Pot helemaal linksom (CCW, zelfde kant MIN)   -> spiegelbeeld hiervan
// Rond het midden zit een dode zone (POT_DEADBAND) zodat de servo niet
// "zoekt" als de pot niet perfect gecentreerd staat.
// De servo beweegt stapsgewijs (in brokken) naar de nieuwe doelwaarde, op
// hetzelfde tempo als de snelheids-slider aangeeft (chunkStepsFromSpeed()).
void handleManualPotentiometer() {
  float frac = readPotCentered(); // -1.0 (helemaal links) .. 0.0 (midden) .. +1.0 (helemaal rechts)

  // Maximale uitwijking (in stappen van 0,1%) vanaf manRefStep, bepaald
  // door de slider: 100% = volledige 50% naar elke kant toegestaan
  // (de helft van het 0-100% vlinderbereik).
  int maxOffsetSteps = (int)((speedPercent / 100.0f) * (SERVO_MAX_STEP / 2));

  int targetStep;
  if (fabs(frac) < POT_DEADBAND) {
    targetStep = manRefStep; // binnen de dode zone -> doel is de referentiepositie
  } else {
    // Herschaal het bruikbare deel van de slag (buiten de dode zone) naar 0.0-1.0
    float travel = (fabs(frac) - POT_DEADBAND) / (1.0f - POT_DEADBAND);
    travel = constrain(travel, 0.0f, 1.0f);
    int dir = (frac > 0) ? 1 : -1; // rechts = rechtsom (CW/MAX-kant), links = linksom (CCW/MIN-kant)
    targetStep = manRefStep + dir * (int)(travel * maxOffsetSteps);
  }
  targetStep = constrain(targetStep, 0, SERVO_MAX_STEP);

  if (servoPos == targetStep) {
    return; // al op de gewenste positie -> niets te doen
  }

  int chunk = chunkStepsFromSpeed(speedPercent); // hoe snel de servo naar de doelpositie toe beweegt
  int dir = (targetStep > servoPos) ? 1 : -1;
  int remaining = abs(targetStep - servoPos);
  int step = min(chunk, remaining);   // niet voorbij het doel schieten
  servoPos += dir * step;
  servoWriteCurrent();
  drawPosValue();
  delay(TICK_MS);
}

// ======================================================================
// SCHERM 1 - lay-out
// ======================================================================
Btn btnP2      = {15,   5,  290, 30, "SLA SERVO POSITIE OP"};
Btn btnMin     = {15,  45, 90, 50, "MIN"};
Btn btnMan     = {113, 45, 90, 50, "MAN"};   // nieuwe knop tussen MIN en MAX (potentiometer aan/uit)
Btn btnMax     = {211, 45, 90, 50, "MAX"};   // naar links geschoven om plaats te maken voor MAN
Btn posFieldP1   = {15,  144, 80, 36, ""};   // percentage-uitleesveld, direct boven het Freq-invoerveld
Btn memPosFieldP1= {145, 144, 50, 36, ""};   // mem pos-uitleesveld, naast het percentage-veld
Btn freqFieldP1  = {15, 185, 80, 36, ""};
Btn btnListP1    = {145,185, 60, 36, "Lijst"};
Btn btnSetP1   = {235,185, 80, 36, "SET"};

const int sliderX = 15, sliderY = 122, sliderW = 290, sliderH = 16;   // 3px omhoog, mooier midden tussen "snelheid"-tekst en het POS-veld

void drawSliderTrack() {
  tft.fillRect(sliderX, sliderY, sliderW, sliderH, TFT_DARKGREY);
  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);
}

// Niet-lineaire slider: de EERSTE 50% van de slider-slag beslaat maar het
// snelheidsbereik 1-25%, zodat het lage (trage) gebied veel fijner
// instelbaar is. De tweede helft van de slider gaat van 25% naar 100%.
#define SLIDER_KNEE_FRAC   0.5f    // slider-positie (0-1) waar het knikpunt ligt
#define SLIDER_KNEE_SPEED  25.0f   // snelheid (%) op het knikpunt

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
  // oude indicatie wissen (lokaal stuk track terugtekenen i.p.v. hele slider herladen)
  if (lastKnobX >= 0) {
    tft.fillRect(lastKnobX - 8, sliderY - 4, 16, sliderH + 8, TFT_BLACK);
    tft.fillRect(sliderX, sliderY, sliderW, sliderH, TFT_DARKGREY);
    tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);
  }
  tft.fillCircle(knobX, sliderY + sliderH / 2, 9, TFT_ORANGE);
  tft.drawCircle(knobX, sliderY + sliderH / 2, 9, TFT_WHITE);
  lastKnobX = knobX;

  // percentage tonen (1 decimaal, stappen van 0,5%)
  tft.fillRect(sliderX, sliderY - 20, 120, 16, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  char buf[20];
  snprintf(buf, sizeof(buf), "snelheid: %.1f%%", speedPercent);
  tft.drawString(buf, sliderX, sliderY - 20);
}

// Tekent het kader + het "%"-label van het percentage-uitleesveld. Wordt een
// keer getekend bij het opbouwen van scherm 1 (net als het Freq-veld eronder);
// de waarde zelf wordt ververst via drawPosValue().
void drawPosFieldP1() {
  tft.fillRoundRect(posFieldP1.x, posFieldP1.y, posFieldP1.w, posFieldP1.h, 6, TFT_NAVY);
  tft.drawRoundRect(posFieldP1.x, posFieldP1.y, posFieldP1.w, posFieldP1.h, 6, TFT_WHITE);

  // Label "%" achter de waarde, op dezelfde x-positie als "kHz" bij het
  // Freq-veld eronder, zodat beide velden mooi in lijn staan.
  int labelX = posFieldP1.x + posFieldP1.w + 6;
  tft.fillRect(labelX, posFieldP1.y, btnListP1.x - labelX - 4, posFieldP1.h, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("%", labelX, posFieldP1.y + posFieldP1.h / 2);
  tft.setTextDatum(TL_DATUM);

  drawPosValue();
}

// Ververst alleen de waarde IN het percentage-uitleesveld (kader blijft staan).
// Dit wordt vaak aangeroepen (elke stap tijdens een servobeweging), dus
// hier bewust geen fillRoundRect/drawRoundRect - dat zou onnodig knipperen.
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

// Toont (of wist) de "mem pos" match-waarde IN het mem-pos-uitleesveld
// (kader blijft staan, alleen de waarde wordt ververst). Vervangt de oude
// blauwe "Freq:"-tekst, die dubbelop was met het Freq-invoerveld.
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

// Tekent het kader + het "mem pos"-label van het mem-pos-uitleesveld. Wordt
// een keer getekend bij het opbouwen van scherm 1, naast het percentage-veld.
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

  // Label "kHz" iets groter, achter de waarde (kort gehouden i.v.m. ruimte voor de Lijst-knop)
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

// MIN/MAX ingedrukt houden: beweegt de servo in brokken van
// chunkStepsFromSpeed() stappen, op een vaste tick (TICK_MS), zolang de knop
// aangeraakt blijft. Bij lage snelheid is de brok klein (fijn afstellen), bij
// hoge snelheid groot (snel doorlopen) - één en dezelfde formule voor de hele
// slider, zonder speciale gevallen. Omdat elke tick de knop opnieuw wordt
// gecontroleerd, blijft loslaten altijd meteen (binnen 1 brok) werken.
void handleMinMaxHold(int dir, Btn btn) {
  int chunk = chunkStepsFromSpeed(speedPercent);
  Serial.printf("[MIN/MAX] start op stap %d, snelheid %.1f%%, brokgrootte %d stappen (%.1f%%)\n",
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
  drawPosValue(); // altijd de uiteindelijke stand tonen

  unsigned long elapsed = millis() - startMs;
  int stepsMoved = abs(servoPos - startPos);
  Serial.printf("[MIN/MAX] gestopt op stap %d, %d stappen in %lu ms\n", servoPos, stepsMoved, elapsed);
}

void handleScreenP1(bool touched, int tx, int ty, bool wasTouched) {
  bool edge = touched && !wasTouched;

  // MAN-mode: potentiometer bestuurt de servo. Dit loopt onafhankelijk van het
  // touchscreen mee, dus elke keer dat dit scherm wordt afgehandeld en MAN aan
  // staat wordt de pot uitgelezen (ook als er niets wordt aangeraakt).
  if (manMode) {
    handleManualPotentiometer();
  }

  // navigatie naar scherm 2 knop
  if (edge && hit(btnP2, tx, ty)) {
    currentScreen = SCREEN_P2;
    drawScreenP2();
    return;
  }

  // MAN - aan/uit schakelen (potentiometer als tweede besturingsmethode)
  if (edge && hit(btnMan, tx, ty)) {
    manMode = !manMode;
    if (manMode) {
      manRefStep = servoPos; // huidige positie wordt het "midden" voor de pot - geen sprong bij inschakelen
    }
    drawButton(btnMan, manMode ? TFT_YELLOW : TFT_DARKGREEN, manMode ? TFT_BLACK : TFT_WHITE, 3);
    return;
  }

  // MIN - zolang ingedrukt, stap voor stap (0,1%) linksom
  if (touched && hit(btnMin, tx, ty)) {
    handleMinMaxHold(-1, btnMin);
    return;
  }

  // MAX - zolang ingedrukt, stap voor stap (0,1%) rechtsom
  if (touched && hit(btnMax, tx, ty)) {
    handleMinMaxHold(1, btnMax);
    return;
  }

  // Slider snelheid (sleep-bediening, stappen van 0,5%)
  if (touched && tx >= sliderX && tx <= sliderX + sliderW &&
      ty >= sliderY - 15 && ty <= sliderY + sliderH + 15) {
    float raw = speedFromSliderFrac((float)(tx - sliderX) / (float)sliderW);
    float newSpeed = round(raw * 2.0f) / 2.0f;   // afronden op 0,5%
    newSpeed = constrain(newSpeed, 1.0f, 100.0f);
    if (fabs(newSpeed - speedPercent) >= 0.25f) {
      speedPercent = newSpeed;   // drawSliderKnob() wist de oude indicatie zelf
      drawSliderKnob();
      Serial.printf("[snelheid] gewijzigd naar %.1f%% (brokgrootte %d stappen / %.1f%% per tick)\n", speedPercent, chunkStepsFromSpeed(speedPercent), chunkStepsFromSpeed(speedPercent) / 10.0f);
    }
    return;
  }

  // Freq (Khz) invoerveld
  if (edge && hit(freqFieldP1, tx, ty)) {
    freqValueP1 = (int)numericKeypad("Freq (Khz) invoeren (10000-30000)", 10000, 30000, freqValueP1);
    lastFoundMemPos = -1; // nieuw doel ingevoerd, oude match niet meer geldig
    drawScreenP1();
    return;
  }

  // Lijst - kies een Freq uit de reeds opgeslagen waarden
  if (edge && hit(btnListP1, tx, ty)) {
    long chosen = chooseStoredFreq(freqValueP1);
    if (chosen != freqValueP1) {
      freqValueP1 = (int)chosen;
      lastFoundMemPos = -1; // nieuw doel gekozen, oude match niet meer geldig
    }
    drawScreenP1();
    return;
  }

  // SET - servo naar het opgeslagen percentage sturen dat bij deze Freq hoort
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
      tft.drawString("Geen kalibratie gevonden voor deze frequentie!", 15, freqFieldP1.y + freqFieldP1.h + 4);
      delay(1200);
      tft.fillRect(15, freqFieldP1.y + freqFieldP1.h + 4, 290, 16, TFT_BLACK);
    }
    return;
  }
}

// ======================================================================
// SCHERM 2 - lay-out
// ======================================================================
Btn memPosField  = {205,  42, 100, 32, ""};
Btn servoPosBox  = {205,  92, 100, 32, ""};
Btn freqFieldP2  = {205, 140, 100, 32, ""};
Btn btnSetP2     = {15,  182,  90, 36, "SET"};
Btn btnP1        = {15,   5, 290, 30, "HOOFD SCHERM"};

// Tekent een label vooraan een rij, met een invoer-/weergavevak uiterst
// rechts op dezelfde regel. valueText is de al opgemaakte tekst (bv. "123"
// of "90.0") die in het vak komt. labelSize/valueSize bepalen de
// tekstgrootte (2 = zelfde grootte als de tekst op de knop "MAIN SCREEN").
void drawLabelFieldRow(const char* label, Btn b, uint16_t boxBg, uint16_t textColor, const char* valueText, int labelSize = 2, int valueSize = 2) {
  tft.fillRect(15, b.y, b.x - 15 - 4, b.h, TFT_BLACK); // wis het label-gebied vóór het vak
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

// Controleert of een mem pos-adres al gegevens bevat, en geeft desgewenst de opgeslagen Freq + percentage terug
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

// "mem pos"-rij: het vak zelf toont de status via zijn kleur (rood = in
// gebruik, groen = vrij, marineblauw = nog leeg). Als het adres in gebruik
// is, komt de opgeslagen Freq/percentage in kleine letters onder het label te staan.
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
    boxBg = TFT_NAVY; // nog niets ingevuld
  } else if (memPosInUse(memPosValue, freq, percentStep)) {
    boxBg = TFT_RED;  // bezet
    inUse = true;
  } else {
    boxBg = TFT_DARKGREEN; // vrij
  }

  drawLabelFieldRow("GEH POSITIE", memPosField, boxBg, TFT_WHITE, buf);

  // Detailregel onder het label: alleen tonen als bezet.
  // Start ruim ONDER de rand van het mem pos-vak (i.p.v. er half overheen,
  // wat eerder de onderkant van het vak zwart overschilderde).
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
  drawLabelFieldRow("COND POSITIE", servoPosBox, TFT_DARKGREY, TFT_WHITE, buf);
}

void drawFreqRow() {
  char buf[16];
  if (freqValueP2 > 0) {
    snprintf(buf, sizeof(buf), "%d", freqValueP2);
  } else {
    buf[0] = '\0';
  }
  drawLabelFieldRow("FREQUENTIE", freqFieldP2, TFT_NAVY, TFT_YELLOW, buf, 2, 2);
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
    memPosValue = (int)numericKeypad("mem pos invoeren (1-180)", 1, 180, memPosValue);
    drawScreenP2();
    return;
  }

  if (edge && hit(freqFieldP2, tx, ty)) {
    freqValueP2 = (int)numericKeypad("Freq invoeren (10000-30000)", 10000, 30000, freqValueP2);
    drawScreenP2();
    return;
  }

  if (edge && hit(btnSetP2, tx, ty)) {
    if (memPosValue <= 0 || freqValueP2 <= 0) {
      tft.fillRect(15, 228, 290, 10, TFT_BLACK);
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setTextSize(1);
      tft.drawString("Vul eerst mem pos en Freq in!", 15, 228);
      delay(900);
      tft.fillRect(15, 228, 290, 10, TFT_BLACK);
      return;
    }

    saveFreqAtMemPos(memPosValue, freqValueP2, servoPos);

    // Velden leegmaken na het opslaan (0 = leeg weergegeven)
    memPosValue = 0;
    freqValueP2 = 0;
    drawMemPosRow();
    drawFreqRow();

    tft.fillRect(15, 228, 290, 10, TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(1);
    tft.drawString("Opgeslagen!", 15, 228);
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

  // Touch (aparte SPI-bus t.o.v. het display, standaard CYD-bedrading)
  touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  // Display
  tft.init();
  tft.setRotation(1);   // landschap, 320x240
  // LET OP: als kleuren fout blijven (witte achtergrond, rare tinten), zit het
  // probleem waarschijnlijk in de driver-/inversie-instelling van de TFT_eSPI
  // library (User_Setup.h), niet in deze sketch - zie toelichting in de chat.
  tft.fillScreen(TFT_BLACK);

  // Servo - rechtstreeks via ESP32 LEDC-hardware-PWM op SERVO_PIN
  // Niet-vluchtig geheugen (eerst openen: de laatste servostand moet geladen zijn
  // voordat de servo wordt aangestuurd)
  prefs.begin("freqtab", false);
  loadLastServoPos();

  ledcAttach(SERVO_PIN, SERVO_PWM_FREQ, LEDC_RESOLUTION_BITS);
  servoWriteCurrent();   // servo direct naar de laatst opgeslagen stand

  // Potentiometer voor MAN-mode
  analogReadResolution(12);                    // 0-4095
  analogSetPinAttenuation(POT_PIN, ADC_11db);   // volledig 0-3.3V bereik
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

  saveServoPosIfSettled();   // laatste servostand bewaren zodra de servo stilstaat

  // Alleen een vaste rustpauze als er niets wordt aangeraakt/bewogen -
  // tijdens actieve servobeweging bepaalt uitsluitend de snelheidsinstelling het tempo.
  if (!touched) {
    delay(15);
  }
}
