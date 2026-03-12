#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>

/*
  MAX2870 touch UI signal generator
  Target board: NUCLEO-F411RE (STM32 Arduino core)

  SPI bus (shared by TFT, touch, MAX2870):
    D13 -> SPI1_SCK  -> ILI9341 SCK, XPT2046 T_CLK, MAX2870 CLK
    D11 -> SPI1_MOSI -> ILI9341 MOSI, XPT2046 T_DIN, MAX2870 DATA
    D12 -> SPI1_MISO -> XPT2046 T_DO (TFT MISO if present)

  Display / touch pins:
    D8  -> TFT_DC
    D9  -> TFT_CS
    D10 -> TFT_RST
    D7  -> TOUCH_CS

  MAX2870 control pins:
    D5  -> MAX2870 LE
    D6  -> MAX2870 CE
    D4  -> MAX2870 LD  (optional lock detect input)

  Defaults:
    Target frequency    = 1200.0 MHz
    Reference frequency = 100.0 MHz

  Libraries needed:
    - Adafruit_GFX
    - Adafruit_ILI9341
    - XPT2046_Touchscreen
*/

// -------------------- Display / touch pins --------------------
static const uint8_t TFT_DC   = D8;
static const uint8_t TFT_CS   = D9;
static const uint8_t TFT_RST  = D10;
static const uint8_t TOUCH_CS = D7;

// -------------------- MAX2870 pins ----------------------------
static const uint8_t MAX_LE   = D5;
static const uint8_t MAX_CE   = D6;
static const uint8_t MAX_LD   = D4;   // optional

// -------------------- Display / touch objects -----------------
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TOUCH_CS);

// -------------------- Touch calibration -----------------------
static const int16_t TS_MINX = 416;
static const int16_t TS_MAXX = 3805;
static const int16_t TS_MINY = 411;
static const int16_t TS_MAXY = 3635;

// -------------------- Screen / UI constants -------------------
static const int16_t SCREEN_W = 320;
static const int16_t SCREEN_H = 240;

static const uint16_t COLOR_BG       = ILI9341_BLACK;
static const uint16_t COLOR_TEXT     = ILI9341_WHITE;
static const uint16_t COLOR_ACCENT   = ILI9341_CYAN;
static const uint16_t COLOR_KEY      = ILI9341_DARKGREY;
static const uint16_t COLOR_HILITE   = ILI9341_YELLOW;
static const uint16_t COLOR_ERR      = ILI9341_RED;
static const uint16_t COLOR_OK       = ILI9341_GREEN;
static const uint16_t COLOR_WARN     = ILI9341_RED;
static const uint16_t COLOR_PANEL    = ILI9341_NAVY;

static const uint32_t KEY_DEBOUNCE_MS = 180;
static const uint32_t KEY_HILITE_MS   = 120;
static const uint32_t LOCK_POLL_MS    = 150;

// -------------------- Frequency constants ---------------------
static const double FREQ_MIN_MHZ        = 23.5;
static const double FREQ_MAX_MHZ        = 6000.0;
static const double DEFAULT_TARGET_MHZ  = 1200.0;
static const double DEFAULT_REF_MHZ     = 100.0;

// Conservative PFD ceiling for broad loop-filter compatibility.
// MAX2870 itself allows 105 MHz integer-N and 50 MHz fractional-N.
static const double USER_MAX_PFD_HZ = 50.0e6;

// MAX2870 VCO range
static const double VCO_MIN_HZ = 3000.0e6;
static const double VCO_MAX_HZ = 6000.0e6;
static const uint16_t MOD_MAX  = 4095;

// -------------------- MAX2870 SPI -----------------------------
SPISettings max2870Spi(1000000, MSBFIRST, SPI_MODE0);

// -------------------- App state -------------------------------
enum class UiMode { CurrentFreq, Entry };
static UiMode uiMode = UiMode::CurrentFreq;

static String entryStr;
static const size_t MAX_ENTRY_LEN = 16;

static double currentTargetMHz   = DEFAULT_TARGET_MHZ;
static double currentActualMHz   = 0.0;
static double currentRefMHz      = DEFAULT_REF_MHZ;
static bool   lockState          = false;
static bool   lastDrawnLockState = false;
static uint32_t lastTouchMs      = 0;
static uint32_t lastLockPollMs   = 0;

// -------------------- Keypad layout ---------------------------
static const int16_t INPUT_H  = 40;
static const int16_t KEYPAD_Y = INPUT_H;
static const int16_t KEYPAD_H = SCREEN_H - INPUT_H;
static const int16_t ROWS = 4;
static const int16_t COLS = 4;
static const int16_t CELL_W = SCREEN_W / COLS;
static const int16_t CELL_H = KEYPAD_H / ROWS;

struct Key {
  const char* label;
  char        value;
  int16_t     x, y, w, h;
  bool        isEnter;
  bool        isDel;
  bool        isBlank;
};

static Key keys[] = {
  {"7",'7', 0*CELL_W, KEYPAD_Y+0*CELL_H, CELL_W, CELL_H, false,false,false},
  {"8",'8', 1*CELL_W, KEYPAD_Y+0*CELL_H, CELL_W, CELL_H, false,false,false},
  {"9",'9', 2*CELL_W, KEYPAD_Y+0*CELL_H, CELL_W, CELL_H, false,false,false},
  {"DEL",'\0',3*CELL_W,KEYPAD_Y+0*CELL_H, CELL_W, CELL_H, false,true,false},

  {"4",'4', 0*CELL_W, KEYPAD_Y+1*CELL_H, CELL_W, CELL_H, false,false,false},
  {"5",'5', 1*CELL_W, KEYPAD_Y+1*CELL_H, CELL_W, CELL_H, false,false,false},
  {"6",'6', 2*CELL_W, KEYPAD_Y+1*CELL_H, CELL_W, CELL_H, false,false,false},
  {"ENTER",'\0',3*CELL_W,KEYPAD_Y+1*CELL_H, CELL_W, 3*CELL_H, true,false,false},

  {"1",'1', 0*CELL_W, KEYPAD_Y+2*CELL_H, CELL_W, CELL_H, false,false,false},
  {"2",'2', 1*CELL_W, KEYPAD_Y+2*CELL_H, CELL_W, CELL_H, false,false,false},
  {"3",'3', 2*CELL_W, KEYPAD_Y+2*CELL_H, CELL_W, CELL_H, false,false,false},

  {"0",'0', 0*CELL_W, KEYPAD_Y+3*CELL_H, CELL_W, CELL_H, false,false,false},
  {".",'.', 1*CELL_W, KEYPAD_Y+3*CELL_H, CELL_W, CELL_H, false,false,false},
  {"",'\0',  2*CELL_W, KEYPAD_Y+3*CELL_H, CELL_W, CELL_H, false,false,true},
};

static const size_t KEY_COUNT = sizeof(keys) / sizeof(keys[0]);

// -------------------- Current screen controls ------------------
static const int16_t BTN_SET_X = 220;
static const int16_t BTN_SET_Y = 180;
static const int16_t BTN_SET_W = 90;
static const int16_t BTN_SET_H = 45;

// -------------------- MAX2870 plan structure -------------------
struct Max2870Plan {
  bool valid = false;
  bool integerMode = true;

  double targetHz = 0.0;
  double refHz = 0.0;
  double actualHz = 0.0;
  double errorHz = 0.0;
  double vcoHz = 0.0;
  double pfdHz = 0.0;

  bool dbr = false;
  bool rdiv2 = false;
  uint16_t r = 1;

  uint16_t n = 0;
  uint16_t frac = 0;
  uint16_t mod = MOD_MAX;

  uint8_t divider = 1;
  uint8_t divaCode = 0;
  uint16_t bs = 1;

  uint32_t regOff[6] = {0};
  uint32_t regOn[6]  = {0};
};

// -------------------- Small helpers ----------------------------
static uint16_t gcd16(uint16_t a, uint16_t b) {
  while (b != 0) {
    const uint16_t t = a % b;
    a = b;
    b = t;
  }
  return a;
}

static uint16_t clampU16(long value, uint16_t lo, uint16_t hi) {
  if (value < (long)lo) return lo;
  if (value > (long)hi) return hi;
  return (uint16_t)value;
}

static void deselectSpiDevicesForTouch() {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(MAX_LE, LOW);
}

static void deselectSpiDevicesForMax() {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);
}

static void printHex32(const char *label, uint32_t value) {
  char buf[11];
  snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)value);
  Serial.print(label);
  Serial.println(buf);
}

// -------------------- Touch handling ---------------------------
static bool getTouchXY(int16_t &sx, int16_t &sy) {
  deselectSpiDevicesForTouch();

  if (!ts.touched()) return false;
  TS_Point p = ts.getPoint();

  if (p.z < 300 || p.z > 4000) return false;
  if (p.x == 0 && p.y == 0) return false;

  sx = map(p.x, TS_MINX, TS_MAXX, 319, 0);
  sy = map(p.y, TS_MINY, TS_MAXY, 239, 0);
  sx = constrain(sx, 0, 319);
  sy = constrain(sy, 0, 239);
  return true;
}

// -------------------- UI drawing -------------------------------
static void drawKey(const Key& k, bool highlighted) {
  if (k.isBlank) {
    tft.fillRect(k.x, k.y, k.w, k.h, COLOR_BG);
    tft.drawRect(k.x, k.y, k.w, k.h, ILI9341_BLACK);
    return;
  }

  const uint16_t fill = highlighted ? COLOR_HILITE : COLOR_KEY;
  const uint16_t text = highlighted ? ILI9341_BLACK : COLOR_TEXT;

  tft.fillRect(k.x, k.y, k.w, k.h, fill);
  tft.drawRect(k.x, k.y, k.w, k.h, ILI9341_BLACK);

  tft.setTextColor(text);
  tft.setTextSize(2);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(k.label, 0, 0, &x1, &y1, &tw, &th);
  const int16_t cx = k.x + (k.w - (int16_t)tw) / 2;
  const int16_t cy = k.y + (k.h - (int16_t)th) / 2;
  tft.setCursor(cx, cy);
  tft.print(k.label);
}

static void updateEntryText() {
  tft.fillRect(70, 1, SCREEN_W - 71, INPUT_H - 2, COLOR_BG);
  tft.setCursor(70, 12);
  tft.setTextColor(COLOR_ACCENT);
  tft.setTextSize(2);
  tft.print(entryStr);
}

static void drawEntryScreen() {
  tft.fillScreen(COLOR_BG);
  tft.drawRect(0, 0, SCREEN_W, INPUT_H, COLOR_TEXT);
  tft.setTextColor(COLOR_ACCENT);
  tft.setTextSize(2);
  tft.setCursor(6, 12);
  tft.print("MHz:");

  for (size_t i = 0; i < KEY_COUNT; ++i) drawKey(keys[i], false);

  tft.fillRect(3 * CELL_W, KEYPAD_Y + 1 * CELL_H, CELL_W, 3 * CELL_H, COLOR_KEY);
  tft.drawRect(3 * CELL_W, KEYPAD_Y + 1 * CELL_H, CELL_W, 3 * CELL_H, ILI9341_BLACK);
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(3 * CELL_W + 8, KEYPAD_Y + 1 * CELL_H + (3 * CELL_H) / 2 - 8);
  tft.print("ENTER");

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(1);
  tft.setCursor(4, 28);
  tft.print("Range ");
  tft.print(FREQ_MIN_MHZ, 1);
  tft.print(" to ");
  tft.print(FREQ_MAX_MHZ, 1);

  updateEntryText();
}

static void drawLockBadge(bool locked) {
  const int16_t x = 10;
  const int16_t y = 150;
  const int16_t w = 170;
  const int16_t h = 40;

  tft.fillRoundRect(x, y, w, h, 8, COLOR_PANEL);
  tft.drawRoundRect(x, y, w, h, 8, locked ? COLOR_OK : COLOR_WARN);
  tft.fillCircle(x + 18, y + 20, 10, locked ? COLOR_OK : COLOR_WARN);

  tft.setTextSize(2);
  tft.setTextColor(locked ? COLOR_OK : COLOR_WARN, COLOR_PANEL);
  tft.setCursor(x + 38, y + 12);
  tft.print(locked ? "LOCKED" : "UNLOCKED");

  lastDrawnLockState = locked;
}

static void drawCurrentFreqScreen() {
  tft.fillScreen(COLOR_BG);

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(10, 8);
  tft.print("MAX2870 Generator");

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(10, 35);
  tft.print("Target");

  tft.setTextColor(COLOR_ACCENT);
  tft.setTextSize(4);
  tft.setCursor(10, 65);
  tft.print(currentTargetMHz, 1);
  tft.print(" MHz");

  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(10, 115);
  tft.print("Ref ");
  tft.print(currentRefMHz, 1);
  tft.print(" MHz");

  drawLockBadge(lockState);

  tft.fillRect(BTN_SET_X, BTN_SET_Y, BTN_SET_W, BTN_SET_H, COLOR_KEY);
  tft.drawRect(BTN_SET_X, BTN_SET_Y, BTN_SET_W, BTN_SET_H, ILI9341_WHITE);
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(BTN_SET_X + 18, BTN_SET_Y + 14);
  tft.print("SET");
}

static void flashError(const char* msg) {
  tft.fillScreen(COLOR_ERR);
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.setCursor(10, 90);
  tft.print("ERROR");
  tft.setCursor(10, 120);
  tft.print(msg);
  delay(350);

  if (uiMode == UiMode::Entry) drawEntryScreen();
  else drawCurrentFreqScreen();
}

// -------------------- Entry handling --------------------------
static bool isValidAppend(char c) {
  if (entryStr.length() >= MAX_ENTRY_LEN) return false;
  if (c >= '0' && c <= '9') return true;
  if (c == '.') {
    if (entryStr.length() == 0) return false;
    if (entryStr.indexOf('.') >= 0) return false;
    return true;
  }
  return false;
}

static void doDelete() {
  if (entryStr.length() == 0) return;
  entryStr.remove(entryStr.length() - 1);
  updateEntryText();
}

static bool parseEntryMHz(double &outMHz) {
  if (entryStr.length() == 0) return false;
  outMHz = entryStr.toFloat();
  return true;
}

// -------------------- MAX2870 planning ------------------------
static bool chooseOutputDivider(double targetHz, uint8_t &divider, uint8_t &divaCode, double &vcoHz) {
  const uint8_t dividers[8] = {1, 2, 4, 8, 16, 32, 64, 128};
  for (uint8_t code = 0; code < 8; ++code) {
    const double candidateVco = targetHz * (double)dividers[code];
    if (candidateVco >= VCO_MIN_HZ && candidateVco <= VCO_MAX_HZ) {
      divider = dividers[code];
      divaCode = code;
      vcoHz = candidateVco;
      return true;
    }
  }
  return false;
}

static bool planInteger(Max2870Plan &p) {
  bool found = false;
  double bestPfd = -1.0;

  for (uint8_t dbr = 0; dbr <= 1; ++dbr) {
    if (dbr && p.refHz > 100.0e6) continue;

    for (uint8_t rdiv2 = 0; rdiv2 <= 1; ++rdiv2) {
      const double denom2 = rdiv2 ? 2.0 : 1.0;

      for (uint16_t r = 1; r <= 1023; ++r) {
        const double pfd = p.refHz * (dbr ? 2.0 : 1.0) / ((double)r * denom2);
        if (pfd > USER_MAX_PFD_HZ) continue;
        if (pfd > 105.0e6) continue;

        const double nReal = p.vcoHz / pfd;
        const double nRounded = round(nReal);
        if (fabs(nReal - nRounded) > 1e-9) continue;
        if (nRounded < 16.0 || nRounded > 65535.0) continue;

        if (pfd > bestPfd) {
          found = true;
          bestPfd = pfd;
          p.integerMode = true;
          p.dbr = (dbr != 0);
          p.rdiv2 = (rdiv2 != 0);
          p.r = r;
          p.pfdHz = pfd;
          p.n = (uint16_t)nRounded;
          p.frac = 0;
          p.mod = MOD_MAX;
          p.actualHz = p.vcoHz / (double)p.divider;
          p.errorHz = p.actualHz - p.targetHz;
        }
      }
    }
  }

  return found;
}

static bool planFractional(Max2870Plan &p) {
  bool found = false;
  double bestError = 1.0e99;
  double bestPfd = -1.0;

  for (uint8_t dbr = 0; dbr <= 1; ++dbr) {
    if (dbr && p.refHz > 100.0e6) continue;

    for (uint8_t rdiv2 = 0; rdiv2 <= 1; ++rdiv2) {
      const double denom2 = rdiv2 ? 2.0 : 1.0;

      for (uint16_t r = 1; r <= 1023; ++r) {
        const double pfd = p.refHz * (dbr ? 2.0 : 1.0) / ((double)r * denom2);
        if (pfd > USER_MAX_PFD_HZ) continue;
        if (pfd > 50.0e6) continue;

        const double ratio = p.vcoHz / pfd;
        const double nFloor = floor(ratio);
        if (nFloor < 19.0 || nFloor > 4091.0) continue;

        uint16_t mod = MOD_MAX;
        uint16_t frac = (uint16_t)llround((ratio - nFloor) * (double)mod);
        uint16_t n = (uint16_t)nFloor;

        if (frac >= mod) {
          frac = 0;
          ++n;
        }
        if (n < 19 || n > 4091) continue;

        if (frac != 0) {
          const uint16_t g = gcd16(frac, mod);
          frac /= g;
          mod /= g;
          if (mod < 2) mod = 2;
        }

        const double actualVco = ((double)n + ((double)frac / (double)mod)) * pfd;
        const double actualRf = actualVco / (double)p.divider;
        const double err = fabs(actualRf - p.targetHz);

        if ((err < bestError - 0.5) || (fabs(err - bestError) <= 0.5 && pfd > bestPfd)) {
          found = true;
          bestError = err;
          bestPfd = pfd;
          p.integerMode = (frac == 0);
          p.dbr = (dbr != 0);
          p.rdiv2 = (rdiv2 != 0);
          p.r = r;
          p.pfdHz = pfd;
          p.n = n;
          p.frac = frac;
          p.mod = mod;
          p.actualHz = actualRf;
          p.errorHz = actualRf - p.targetHz;
        }
      }
    }
  }

  return found;
}

// -------------------- MAX2870 register builders ----------------
static uint32_t buildR0(const Max2870Plan &p) {
  uint32_t r = 0;
  if (p.integerMode) r |= (1UL << 31);
  r |= ((uint32_t)p.n    & 0xFFFFUL) << 15;
  r |= ((uint32_t)p.frac & 0x0FFFUL) << 3;
  return r;
}

static uint32_t buildR1(const Max2870Plan &p) {
  uint32_t r = 0;
  if (p.integerMode) {
    r |= (1UL << 31);      // CPOC = 1 in integer-N
  } else {
    r |= (1UL << 29);      // CPL = 01 in fractional-N
  }
  r |= (1UL << 15);        // P = 1
  r |= ((uint32_t)p.mod & 0x0FFFUL) << 3;
  r |= 1U;
  return r;
}

static uint32_t buildR2(const Max2870Plan &p, bool counterReset) {
  uint32_t r = 0;

  if (p.pfdHz > 32.0e6) r |= (1UL << 31);   // LDS
  if (p.dbr)   r |= (1UL << 25);
  if (p.rdiv2) r |= (1UL << 24);

  r |= ((uint32_t)p.r & 0x03FFUL) << 14;
  r |= (1UL << 13);                         // REG4DB = 1
  r |= (15UL << 9);                         // CP current = maximum
  if (p.integerMode) r |= (1UL << 8);      // LDF
  r |= (1UL << 6);                          // PDP = positive
  if (counterReset) r |= (1UL << 3);       // RST
  r |= 2U;
  return r;
}

static uint32_t buildR3() {
  // RETUNE = 1, VAS enabled, clock divider off, CDIV = 1
  return 0x0100000BUL;
}

static uint32_t buildR4(const Max2870Plan &p, bool enableA, bool enableB) {
  uint32_t r = 0;
  r |= (0b011000UL << 26);                      // reserved bits
  r |= ((uint32_t)((p.bs >> 8) & 0x03U)) << 24;
  r |= (1UL << 23);                             // FB = 1
  r |= ((uint32_t)p.divaCode & 0x07U) << 20;
  r |= ((uint32_t)(p.bs & 0xFFU)) << 12;

  if (enableB) r |= (1UL << 8);
  r |= (0UL << 6);                              // RFB power low

  if (enableA) r |= (1UL << 5);
  r |= (3UL << 3);                              // RFA power max

  r |= 4U;
  return r;
}

static uint32_t buildR5(const Max2870Plan &p) {
  uint32_t r = 0x00400005UL;                    // LD = digital lock detect
  if (p.integerMode || p.frac == 0) r |= (1UL << 24); // F01
  return r;
}

static void buildRegisters(Max2870Plan &p) {
  p.bs = clampU16(lround(p.pfdHz / 50000.0), 1, 1023);

  p.regOff[0] = buildR0(p);
  p.regOff[1] = buildR1(p);
  p.regOff[2] = buildR2(p, false);
  p.regOff[3] = buildR3();
  p.regOff[4] = buildR4(p, false, false);
  p.regOff[5] = buildR5(p);

  p.regOn[0] = p.regOff[0];
  p.regOn[1] = p.regOff[1];
  p.regOn[2] = p.regOff[2];
  p.regOn[3] = p.regOff[3];
  p.regOn[4] = buildR4(p, true, false);
  p.regOn[5] = p.regOff[5];
}

static bool makePlan(double targetMHz, double refMHz, Max2870Plan &p) {
  p = Max2870Plan();
  p.targetHz = targetMHz * 1.0e6;
  p.refHz = refMHz * 1.0e6;

  if (p.targetHz < (FREQ_MIN_MHZ * 1.0e6) || p.targetHz > (FREQ_MAX_MHZ * 1.0e6)) return false;
  if (!chooseOutputDivider(p.targetHz, p.divider, p.divaCode, p.vcoHz)) return false;

  if (!planInteger(p)) {
    if (!planFractional(p)) return false;
  }

  buildRegisters(p);
  p.valid = true;
  return true;
}

// -------------------- MAX2870 SPI writes ----------------------
static void max2870Write(uint32_t value) {
  deselectSpiDevicesForMax();

  SPI.beginTransaction(max2870Spi);
  digitalWrite(MAX_LE, LOW);

  SPI.transfer((uint8_t)((value >> 24) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 16) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 8)  & 0xFFU));
  SPI.transfer((uint8_t)( value        & 0xFFU));

  digitalWrite(MAX_LE, HIGH);   // latch the 32-bit word
  delayMicroseconds(2);
  SPI.endTransaction();
}

static void writeBlock(const uint32_t regs[6]) {
  max2870Write(regs[5]);
  max2870Write(regs[4]);
  max2870Write(regs[3]);
  max2870Write(regs[2]);
  max2870Write(regs[1]);
  max2870Write(regs[0]);
}

static void printPlan(const Max2870Plan &p) {
  Serial.println();
  Serial.println(F("MAX2870 plan"));
  Serial.println(F("------------------------"));
  Serial.print(F("Target RF  : ")); Serial.print(p.targetHz / 1.0e6, 6); Serial.println(F(" MHz"));
  Serial.print(F("Ref input  : ")); Serial.print(p.refHz / 1.0e6, 6); Serial.println(F(" MHz"));
  Serial.print(F("Mode       : ")); Serial.println(p.integerMode ? F("Integer-N") : F("Fractional-N"));
  Serial.print(F("Actual RF  : ")); Serial.print(p.actualHz / 1.0e6, 9); Serial.println(F(" MHz"));
  Serial.print(F("Error      : ")); Serial.print(p.errorHz, 3); Serial.println(F(" Hz"));
  Serial.print(F("VCO        : ")); Serial.print(p.vcoHz / 1.0e6, 6); Serial.println(F(" MHz"));
  Serial.print(F("Divider    : /")); Serial.println(p.divider);
  Serial.print(F("PFD        : ")); Serial.print(p.pfdHz / 1.0e6, 6); Serial.println(F(" MHz"));
  Serial.print(F("R / DBR / RDIV2 : ")); Serial.print(p.r); Serial.print(F(" / ")); Serial.print(p.dbr ? 1 : 0); Serial.print(F(" / ")); Serial.println(p.rdiv2 ? 1 : 0);
  Serial.print(F("N / FRAC / MOD : ")); Serial.print(p.n); Serial.print(F(" / ")); Serial.print(p.frac); Serial.print(F(" / ")); Serial.println(p.mod);
  Serial.print(F("BS         : ")); Serial.println(p.bs);

  printHex32("R5 = ", p.regOn[5]);
  printHex32("R4 = ", p.regOn[4]);
  printHex32("R3 = ", p.regOn[3]);
  printHex32("R2 = ", p.regOn[2]);
  printHex32("R1 = ", p.regOn[1]);
  printHex32("R0 = ", p.regOn[0]);
}

static void programMax2870(const Max2870Plan &p) {
  // Force a clean re-start of the MAX2870 on every retune.
  // This is the closest thing to the power-cycle behavior that has already
  // proven reliable in your earlier testing.
  digitalWrite(MAX_CE, LOW);
  digitalWrite(MAX_LE, HIGH);   // idle high so shared SPI traffic is ignored
  delay(5);

  digitalWrite(MAX_CE, HIGH);
  delay(25);                    // datasheet says allow at least 20 ms

  uint32_t resetRegs[6];
  for (uint8_t i = 0; i < 6; ++i) resetRegs[i] = p.regOff[i];
  resetRegs[2] = buildR2(p, true);

  // Latch any buffered divider change with outputs disabled
  max2870Write(p.regOff[4]);
  max2870Write(p.regOff[0]);

  // First pass with counters reset
  max2870Write(resetRegs[5]);
  delay(20);
  max2870Write(resetRegs[4]);
  max2870Write(resetRegs[3]);
  max2870Write(resetRegs[2]);
  max2870Write(resetRegs[1]);
  max2870Write(resetRegs[0]);

  // Second pass, normal operation
  max2870Write(p.regOff[5]);
  max2870Write(p.regOff[4]);
  max2870Write(p.regOff[3]);
  max2870Write(p.regOff[2]);
  max2870Write(p.regOff[1]);
  max2870Write(p.regOff[0]);

  // One more full pass has been helping warm-retune reliability
  writeBlock(p.regOff);

  // Give VAS / band select time to settle before enabling output
  delay(2);

  // Enable RF output last
  max2870Write(p.regOn[4]);
  delay(2);
}

static bool waitForLock(uint32_t timeoutMs = 250) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    if (digitalRead(MAX_LD) == HIGH) return true;
  }
  return false;
}

static bool tuneMax2870(double targetMHz) {
  Max2870Plan p;
  if (!makePlan(targetMHz, currentRefMHz, p)) return false;

  printPlan(p);
  programMax2870(p);

  currentTargetMHz = targetMHz;
  currentActualMHz = p.actualHz / 1.0e6;
  lockState = waitForLock();
  return true;
}

// -------------------- UI hit tests ----------------------------
static const Key* hitKeypad(int16_t x, int16_t y) {
  if (x >= 3 * CELL_W && x < 4 * CELL_W && y >= (KEYPAD_Y + 1 * CELL_H) && y < (KEYPAD_Y + 4 * CELL_H)) {
    static Key enterKey = {"ENTER", '\0', 3 * CELL_W, KEYPAD_Y + 1 * CELL_H, CELL_W, 3 * CELL_H, true, false, false};
    return &enterKey;
  }

  for (size_t i = 0; i < KEY_COUNT; ++i) {
    const Key& k = keys[i];
    if (k.isBlank) continue;
    if (x >= k.x && x < (k.x + k.w) && y >= k.y && y < (k.y + k.h)) return &k;
  }
  return nullptr;
}

static bool hitSetButton(int16_t x, int16_t y) {
  return (x >= BTN_SET_X && x < (BTN_SET_X + BTN_SET_W) && y >= BTN_SET_Y && y < (BTN_SET_Y + BTN_SET_H));
}

// -------------------- Setup / loop helpers --------------------
static void initPins() {
  pinMode(TFT_CS, OUTPUT);    digitalWrite(TFT_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT);  digitalWrite(TOUCH_CS, HIGH);
  pinMode(TFT_DC, OUTPUT);    digitalWrite(TFT_DC, HIGH);
  pinMode(TFT_RST, OUTPUT);   digitalWrite(TFT_RST, HIGH);

  // Keep the MAX2870 asleep and ignore shared SPI traffic during display init
  pinMode(MAX_LE, OUTPUT);    digitalWrite(MAX_LE, HIGH);   // idle HIGH
  pinMode(MAX_CE, OUTPUT);    digitalWrite(MAX_CE, LOW);    // keep in low-power mode
  pinMode(MAX_LD, INPUT);
}

static void updateLockOnScreenIfNeeded() {
  if (uiMode != UiMode::CurrentFreq) return;
  const uint32_t now = millis();
  if (now - lastLockPollMs < LOCK_POLL_MS) return;
  lastLockPollMs = now;

  const bool newLock = (digitalRead(MAX_LD) == HIGH);
  if (newLock != lastDrawnLockState) {
    lockState = newLock;
    drawLockBadge(lockState);
  }
}

// -------------------- Arduino setup ---------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  initPins();
  SPI.begin();

  tft.begin();
  tft.setRotation(1);

  if (!ts.begin()) {
    tft.fillScreen(COLOR_ERR);
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setCursor(10, 110);
    tft.print("Touch init FAILED");
    while (1) delay(1000);
  }

  // Now wake the MAX2870 only after the other SPI devices are initialized
  digitalWrite(MAX_CE, HIGH);
  digitalWrite(MAX_LE, HIGH);
  delay(25);   // datasheet says allow >=20 ms after exiting low-power mode

  if (!tuneMax2870(DEFAULT_TARGET_MHZ)) {
    tft.fillScreen(COLOR_ERR);
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setCursor(10, 100);
    tft.print("MAX2870 init FAILED");
    while (1) delay(1000);
  }

  uiMode = UiMode::CurrentFreq;
  drawCurrentFreqScreen();
}

// -------------------- Arduino loop ----------------------------
void loop() {
  updateLockOnScreenIfNeeded();

  int16_t x, y;
  if (!getTouchXY(x, y)) {
    delay(5);
    return;
  }

  const uint32_t now = millis();
  if (now - lastTouchMs < KEY_DEBOUNCE_MS) return;
  lastTouchMs = now;

  if (uiMode == UiMode::CurrentFreq) {
    if (hitSetButton(x, y)) {
      uiMode = UiMode::Entry;
      entryStr = "";
      drawEntryScreen();
    }
    return;
  }

  const Key* k = hitKeypad(x, y);
  if (!k) return;

  if (k->isEnter) {
    tft.fillRect(3 * CELL_W, KEYPAD_Y + 1 * CELL_H, CELL_W, 3 * CELL_H, COLOR_HILITE);
    tft.drawRect(3 * CELL_W, KEYPAD_Y + 1 * CELL_H, CELL_W, 3 * CELL_H, ILI9341_BLACK);
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(3 * CELL_W + 8, KEYPAD_Y + 1 * CELL_H + (3 * CELL_H) / 2 - 8);
    tft.print("ENTER");
    delay(KEY_HILITE_MS);

    tft.fillRect(3 * CELL_W, KEYPAD_Y + 1 * CELL_H, CELL_W, 3 * CELL_H, COLOR_KEY);
    tft.drawRect(3 * CELL_W, KEYPAD_Y + 1 * CELL_H, CELL_W, 3 * CELL_H, ILI9341_BLACK);
    tft.setTextColor(COLOR_TEXT);
    tft.setTextSize(2);
    tft.setCursor(3 * CELL_W + 8, KEYPAD_Y + 1 * CELL_H + (3 * CELL_H) / 2 - 8);
    tft.print("ENTER");
  } else {
    drawKey(*k, true);
    delay(KEY_HILITE_MS);
    drawKey(*k, false);
  }

  if (k->isDel) {
    doDelete();
    return;
  }

  if (k->isEnter) {
    double mhz;
    if (!parseEntryMHz(mhz)) {
      flashError("Bad entry");
      entryStr = "";
      updateEntryText();
      return;
    }

    if (mhz < FREQ_MIN_MHZ || mhz > FREQ_MAX_MHZ) {
      flashError("Out of range");
      entryStr = "";
      updateEntryText();
      return;
    }

    if (!tuneMax2870(mhz)) {
      flashError("Tune failed");
      entryStr = "";
      updateEntryText();
      return;
    }

    Serial.print("MAX2870 set to ");
    Serial.print(currentActualMHz, 6);
    Serial.println(" MHz");

    uiMode = UiMode::CurrentFreq;
    drawCurrentFreqScreen();
    return;
  }

  if (k->value != '\0' && isValidAppend(k->value)) {
    entryStr += k->value;
    updateEntryText();
  }
}