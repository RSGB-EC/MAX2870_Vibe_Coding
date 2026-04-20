***Acting as an arduino coding expert, produce a sketch to calculate the programming register values for a MAX 2870 from a given target and reference frequency. Use 1200MHz as the default target frequency. Use 100MHz as the default reference frequency. Include routines to programme the device using SPI. Target processor is a Nucleo-32 F303 with the Arduino IDE***



Thought for 12m 26s

Below is a complete Arduino sketch for a **NUCLEO-F303K8** in the **STM32 Arduino core**. On this board, the default hardware SPI pins are **D11 = MOSI (PB5), D12 = MISO (PB4), D13 = SCK (PB3)**, and D10 is just a GPIO-backed “SPI_CS” pin, so it is a convenient choice for the MAX2870 **LE** line. The MAX2870 wants 32-bit words, **MSB first**, and its startup write order is **R5 → R4 → R3 → R2 → R1 → R0**, programmed twice after power-up. Its key equations are `fPFD = fREF * (1 + DBR) / (R * (1 + RDIV2))`, `fVCO = fRFOUT * DIVA`, and with **FB = 1** the divider relation is `N + F/M = fVCO / fPFD`. The VCO must stay in the **3 GHz to 6 GHz** range, and the band-select value is `BS = round(fPFD / 50 kHz)` clamped to 1…1023. 

This sketch uses **1200 MHz** target and **100 MHz** reference by default. With those defaults, it lands on **4.8 GHz VCO with divide-by-4**, and because `4800 / 100 = 48`, it selects **integer-N mode** with **N = 48**. The loop-filter-dependent bits are left near conservative datasheet-style settings, except that the sketch turns on **REG4DB** so the R4 frequency-related bits update cleanly on the R0 write; charge-pump current and polarity may still need adjustment for your actual loop filter. 

```
/*
  MAX2870 register calculator + SPI programmer
  Target board: ST NUCLEO-F303K8
  Arduino IDE with STM32 Arduino core

  Wiring used by this sketch:
    NUCLEO D13 (PB3 / SPI1_SCK)  -> MAX2870 CLK
    NUCLEO D11 (PB5 / SPI1_MOSI) -> MAX2870 DATA
    NUCLEO D10                   -> MAX2870 LE
    NUCLEO D9                    -> MAX2870 CE
    NUCLEO D2  (optional)        -> MAX2870 LD   (digital lock detect)
    NUCLEO D12 (optional)        -> MAX2870 MUX_OUT (not used here)

  Defaults:
    RF output target = 1200 MHz
    Reference input  = 100 MHz

  Notes:
    - This sketch uses FB=1 (fundamental VCO feedback).
    - It tries integer-N first; if exact integer division is not possible,
      it falls back to fractional-N using MOD = 4095 for best resolution.
    - Frequency-critical bits are computed automatically.
    - Charge pump current, lock-detect precision, and loop-filter choices
      may need tuning for your hardware.
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// --------------------------- User settings ---------------------------

constexpr double DEFAULT_TARGET_MHZ = 1200.0;
constexpr double DEFAULT_REF_MHZ    = 100.0;

// Chosen control pins on NUCLEO-F303K8
constexpr uint8_t PIN_MAX_LE = D10;   // Load Enable
constexpr uint8_t PIN_MAX_CE = D9;    // Chip Enable / power control
constexpr uint8_t PIN_MAX_LD = D2;    // Optional lock-detect input

// RF output selection
constexpr bool ENABLE_RFOUTA = true;
constexpr bool ENABLE_RFOUTB = false;

// Output power: 0=-4 dBm, 1=-1 dBm, 2=+2 dBm, 3=+5 dBm
constexpr uint8_t RFOUTA_POWER = 3;
constexpr uint8_t RFOUTB_POWER = 3;

// Safe SPI speed. MAX2870 timing supports much faster, but 1 MHz is conservative.
SPISettings max2870Spi(1000000, MSBFIRST, SPI_MODE0);

// --------------------------- MAX2870 constants ---------------------------

constexpr double MIN_RF_HZ  = 23.5e6;
constexpr double MAX_RF_HZ  = 6000.0e6;
constexpr double MIN_VCO_HZ = 3000.0e6;
constexpr double MAX_VCO_HZ = 6000.0e6;
constexpr double MIN_REF_HZ = 10.0e6;
constexpr double MAX_REF_HZ = 200.0e6;

constexpr double MAX_PFD_INT_HZ  = 105.0e6;
constexpr double MAX_PFD_FRAC_HZ = 50.0e6;

constexpr uint16_t MOD_FIXED = 4095;

// --------------------------- Data structures ---------------------------

struct Max2870Config {
  bool valid = false;
  bool integerMode = true;

  double targetHz = 0.0;
  double refHz = 0.0;
  double actualHz = 0.0;
  double errorHz = 0.0;
  double pfdHz = 0.0;
  double vcoHz = 0.0;

  uint16_t rCounter = 1;
  bool dbr = false;
  bool rdiv2 = false;

  uint16_t n = 0;
  uint16_t frac = 0;
  uint16_t mod = MOD_FIXED;

  uint8_t diva = 1;       // actual divider: 1,2,4,...128
  uint8_t divaCode = 0;   // register encoding 0..7
  uint16_t bs = 1;        // 1..1023

  uint32_t regsDisabled[6] = {0};
  uint32_t regsEnabled[6]  = {0};
};

// --------------------------- Helpers ---------------------------

static uint16_t clampU16(long value, uint16_t lo, uint16_t hi) {
  if (value < (long)lo) return lo;
  if (value > (long)hi) return hi;
  return (uint16_t)value;
}

static void printHex32(const char *label, uint32_t value) {
  char buf[11];
  snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)value);
  Serial.print(label);
  Serial.println(buf);
}

static bool chooseOutputDivider(double targetHz, uint8_t &diva, uint8_t &divaCode, double &vcoHz) {
  const uint8_t dividers[8] = {1, 2, 4, 8, 16, 32, 64, 128};

  for (uint8_t code = 0; code < 8; ++code) {
    double candidateVco = targetHz * (double)dividers[code];
    if (candidateVco >= MIN_VCO_HZ && candidateVco <= MAX_VCO_HZ) {
      diva = dividers[code];
      divaCode = code;
      vcoHz = candidateVco;
      return true;
    }
  }
  return false;
}

static uint32_t buildReg0(bool integerMode, uint16_t n, uint16_t frac) {
  uint32_t r = 0;
  r |= (uint32_t)(integerMode ? 1UL : 0UL) << 31;
  r |= (uint32_t)(n & 0xFFFFU) << 15;
  r |= (uint32_t)(frac & 0x0FFFU) << 3;
  r |= 0U; // address = 0
  return r;
}

static uint32_t buildReg1(bool integerMode, uint16_t mod) {
  // P = 1, CPT = normal, CPOC = 0
  // CPL = 0 for integer-N, 1 for fractional-N
  uint32_t r = 0;
  r |= (uint32_t)(integerMode ? 0UL : 1UL) << 29;  // CPL[1:0] = 01 in frac mode
  r |= (uint32_t)1U << 15;                          // P = 1
  r |= (uint32_t)(mod & 0x0FFFU) << 3;             // MOD
  r |= 1U;                                          // address = 1
  return r;
}

static uint32_t buildReg2(bool integerMode, bool dbr, bool rdiv2, uint16_t rCounter) {
  // MUX = 0000 (tri-state), REG4DB = 1, CP current = default low setting,
  // LDP = 0, PDP = 1, SHDN = 0, TRI = 0, RST = 0
  uint32_t r = 0;
  r |= (uint32_t)(dbr ? 1UL : 0UL)   << 25;
  r |= (uint32_t)(rdiv2 ? 1UL : 0UL) << 24;
  r |= (uint32_t)(rCounter & 0x03FFU) << 14;
  r |= 1UL << 13;                                     // REG4DB = 1
  r |= (uint32_t)(integerMode ? 1UL : 0UL) << 8;     // LDF
  r |= 1UL << 6;                                      // PDP = positive
  r |= 2U;                                            // address = 2
  return r;
}

static uint32_t buildReg3() {
  // Keep VAS enabled, default clock divider value = 1, RETUNE disabled.
  return 0x0000000BUL;
}

static uint32_t buildReg4(uint16_t bs, uint8_t divaCode, bool enableA, bool enableB) {
  uint32_t r = 0;

  // Reserved bits 31:26 must be 011000
  r |= (uint32_t)0b011000UL << 26;

  // BS split across [25:24] and [19:12]
  r |= (uint32_t)((bs >> 8) & 0x03U) << 24;
  r |= 1UL << 23;                                     // FB = 1 (fundamental feedback)
  r |= (uint32_t)(divaCode & 0x07U) << 20;
  r |= (uint32_t)(bs & 0x00FFU) << 12;

  // RFOUTB
  r |= (uint32_t)(enableB ? 1UL : 0UL) << 8;
  r |= (uint32_t)(RFOUTB_POWER & 0x03U) << 6;

  // RFOUTA
  r |= (uint32_t)(enableA ? 1UL : 0UL) << 5;
  r |= (uint32_t)(RFOUTA_POWER & 0x03U) << 3;

  r |= 4U; // address = 4
  return r;
}

static uint32_t buildReg5(bool integerMode) {
  // F01 = 1 in integer mode when FRAC=0, LD = digital lock detect, MUX MSB = 0
  uint32_t r = 0;
  r |= (uint32_t)(integerMode ? 1UL : 0UL) << 24;   // F01
  r |= 1UL << 22;                                    // LD = 01 (digital lock detect)
  r |= 5U;                                           // address = 5
  return r;
}

static void writeMax2870(uint32_t word) {
  SPI.beginTransaction(max2870Spi);
  digitalWrite(PIN_MAX_LE, LOW);
  delayMicroseconds(1);

  SPI.transfer((uint8_t)((word >> 24) & 0xFFU));
  SPI.transfer((uint8_t)((word >> 16) & 0xFFU));
  SPI.transfer((uint8_t)((word >> 8)  & 0xFFU));
  SPI.transfer((uint8_t)(word & 0xFFU));

  delayMicroseconds(1);
  digitalWrite(PIN_MAX_LE, HIGH);   // latch
  delayMicroseconds(1);
  digitalWrite(PIN_MAX_LE, LOW);

  SPI.endTransaction();
}

static void writeRegisterBlock(const uint32_t regs[6]) {
  // Required order: R5, R4, R3, R2, R1, R0
  writeMax2870(regs[5]);
  writeMax2870(regs[4]);
  writeMax2870(regs[3]);
  writeMax2870(regs[2]);
  writeMax2870(regs[1]);
  writeMax2870(regs[0]);
}

static bool computeIntegerPlan(Max2870Config &cfg) {
  double bestPfd = -1.0;
  bool found = false;

  for (uint8_t dbr = 0; dbr <= 1; ++dbr) {
    if (dbr && cfg.refHz > 100.0e6) continue;

    for (uint8_t rdiv2 = 0; rdiv2 <= 1; ++rdiv2) {
      const double denomFactor = (double)(rdiv2 ? 2 : 1);

      for (uint16_t r = 1; r <= 1023; ++r) {
        double pfd = cfg.refHz * (double)(dbr ? 2 : 1) / ((double)r * denomFactor);
        if (pfd > MAX_PFD_INT_HZ) continue;

        double nReal = cfg.vcoHz / pfd;
        double nRounded = round(nReal);
        double vcoErrorHz = fabs(nReal - nRounded) * pfd;

        if (vcoErrorHz < 0.5 && nRounded >= 16.0 && nRounded <= 65535.0) {
          if (pfd > bestPfd) {
            bestPfd = pfd;
            cfg.integerMode = true;
            cfg.dbr = (dbr != 0);
            cfg.rdiv2 = (rdiv2 != 0);
            cfg.rCounter = r;
            cfg.pfdHz = pfd;
            cfg.n = (uint16_t)nRounded;
            cfg.frac = 0;
            cfg.mod = MOD_FIXED;
            cfg.actualHz = cfg.vcoHz / (double)cfg.diva;
            cfg.errorHz = cfg.actualHz - cfg.targetHz;
            found = true;
          }
        }
      }
    }
  }

  return found;
}

static bool computeFractionalPlan(Max2870Config &cfg) {
  double bestErrorHz = 1.0e99;
  double bestPfd = -1.0;
  bool found = false;

  for (uint8_t dbr = 0; dbr <= 1; ++dbr) {
    if (dbr && cfg.refHz > 100.0e6) continue;

    for (uint8_t rdiv2 = 0; rdiv2 <= 1; ++rdiv2) {
      const double denomFactor = (double)(rdiv2 ? 2 : 1);

      for (uint16_t r = 1; r <= 1023; ++r) {
        double pfd = cfg.refHz * (double)(dbr ? 2 : 1) / ((double)r * denomFactor);
        if (pfd > MAX_PFD_FRAC_HZ) continue;

        double ratio = cfg.vcoHz / pfd;
        double nReal = floor(ratio);
        if (nReal < 19.0 || nReal > 4091.0) continue;

        double fracPart = ratio - nReal;
        uint16_t frac = (uint16_t)llround(fracPart * (double)MOD_FIXED);
        uint16_t n = (uint16_t)nReal;

        if (frac >= MOD_FIXED) {
          frac = 0;
          ++n;
        }
        if (n < 19 || n > 4091) continue;

        double actualVco = ((double)n + ((double)frac / (double)MOD_FIXED)) * pfd;
        double actualRf = actualVco / (double)cfg.diva;
        double errHz = fabs(actualRf - cfg.targetHz);

        if ((errHz < bestErrorHz - 0.5) ||
            (fabs(errHz - bestErrorHz) <= 0.5 && pfd > bestPfd)) {
          bestErrorHz = errHz;
          bestPfd = pfd;

          cfg.integerMode = false;
          cfg.dbr = (dbr != 0);
          cfg.rdiv2 = (rdiv2 != 0);
          cfg.rCounter = r;
          cfg.pfdHz = pfd;
          cfg.n = n;
          cfg.frac = frac;
          cfg.mod = MOD_FIXED;
          cfg.actualHz = actualRf;
          cfg.errorHz = actualRf - cfg.targetHz;
          found = true;
        }
      }
    }
  }

  return found;
}

static void buildRegisters(Max2870Config &cfg) {
  cfg.bs = clampU16(lround(cfg.pfdHz / 50000.0), 1, 1023);

  cfg.regsDisabled[0] = buildReg0(cfg.integerMode, cfg.n, cfg.frac);
  cfg.regsDisabled[1] = buildReg1(cfg.integerMode, cfg.mod);
  cfg.regsDisabled[2] = buildReg2(cfg.integerMode, cfg.dbr, cfg.rdiv2, cfg.rCounter);
  cfg.regsDisabled[3] = buildReg3();
  cfg.regsDisabled[4] = buildReg4(cfg.bs, cfg.divaCode, false, false);
  cfg.regsDisabled[5] = buildReg5(cfg.integerMode);

  cfg.regsEnabled[0] = cfg.regsDisabled[0];
  cfg.regsEnabled[1] = cfg.regsDisabled[1];
  cfg.regsEnabled[2] = cfg.regsDisabled[2];
  cfg.regsEnabled[3] = cfg.regsDisabled[3];
  cfg.regsEnabled[4] = buildReg4(cfg.bs, cfg.divaCode, ENABLE_RFOUTA, ENABLE_RFOUTB);
  cfg.regsEnabled[5] = cfg.regsDisabled[5];
}

static bool calculateMax2870(double targetMHz, double refMHz, Max2870Config &cfg) {
  cfg = Max2870Config();

  cfg.targetHz = targetMHz * 1.0e6;
  cfg.refHz = refMHz * 1.0e6;

  if (cfg.targetHz < MIN_RF_HZ || cfg.targetHz > MAX_RF_HZ) return false;
  if (cfg.refHz < MIN_REF_HZ || cfg.refHz > MAX_REF_HZ) return false;

  if (!chooseOutputDivider(cfg.targetHz, cfg.diva, cfg.divaCode, cfg.vcoHz)) {
    return false;
  }

  if (!computeIntegerPlan(cfg)) {
    if (!computeFractionalPlan(cfg)) {
      return false;
    }
  }

  buildRegisters(cfg);
  cfg.valid = true;
  return true;
}

static void printConfig(const Max2870Config &cfg) {
  Serial.println();
  Serial.println(F("MAX2870 plan"));
  Serial.println(F("-----------------------------"));
  Serial.print(F("Target RF   : "));
  Serial.print(cfg.targetHz / 1.0e6, 6);
  Serial.println(F(" MHz"));

  Serial.print(F("Reference   : "));
  Serial.print(cfg.refHz / 1.0e6, 6);
  Serial.println(F(" MHz"));

  Serial.print(F("Mode        : "));
  Serial.println(cfg.integerMode ? F("Integer-N") : F("Fractional-N"));

  Serial.print(F("Actual RF   : "));
  Serial.print(cfg.actualHz / 1.0e6, 9);
  Serial.println(F(" MHz"));

  Serial.print(F("Error       : "));
  Serial.print(cfg.errorHz, 3);
  Serial.println(F(" Hz"));

  Serial.print(F("VCO         : "));
  Serial.print(cfg.vcoHz / 1.0e6, 6);
  Serial.println(F(" MHz"));

  Serial.print(F("PFD         : "));
  Serial.print(cfg.pfdHz / 1.0e6, 6);
  Serial.println(F(" MHz"));

  Serial.print(F("DIVA        : "));
  Serial.println(cfg.diva);

  Serial.print(F("R / DBR / RDIV2 : "));
  Serial.print(cfg.rCounter);
  Serial.print(F(" / "));
  Serial.print(cfg.dbr ? 1 : 0);
  Serial.print(F(" / "));
  Serial.println(cfg.rdiv2 ? 1 : 0);

  Serial.print(F("N / FRAC / MOD : "));
  Serial.print(cfg.n);
  Serial.print(F(" / "));
  Serial.print(cfg.frac);
  Serial.print(F(" / "));
  Serial.println(cfg.mod);

  Serial.print(F("BS          : "));
  Serial.println(cfg.bs);

  Serial.println();
  Serial.println(F("Registers (startup, outputs OFF)"));
  printHex32("R5 = ", cfg.regsDisabled[5]);
  printHex32("R4 = ", cfg.regsDisabled[4]);
  printHex32("R3 = ", cfg.regsDisabled[3]);
  printHex32("R2 = ", cfg.regsDisabled[2]);
  printHex32("R1 = ", cfg.regsDisabled[1]);
  printHex32("R0 = ", cfg.regsDisabled[0]);

  Serial.println();
  Serial.println(F("Registers (final, outputs ON)"));
  printHex32("R5 = ", cfg.regsEnabled[5]);
  printHex32("R4 = ", cfg.regsEnabled[4]);
  printHex32("R3 = ", cfg.regsEnabled[3]);
  printHex32("R2 = ", cfg.regsEnabled[2]);
  printHex32("R1 = ", cfg.regsEnabled[1]);
  printHex32("R0 = ", cfg.regsEnabled[0]);
}

static void programMax2870(const Max2870Config &cfg) {
  // Bring the part up
  digitalWrite(PIN_MAX_CE, LOW);
  delay(1);
  digitalWrite(PIN_MAX_CE, HIGH);
  delay(1);

  // Datasheet startup sequence:
  // R5, wait 20ms, then R4..R0
  // repeat full R5..R0
  writeMax2870(cfg.regsDisabled[5]);
  delay(20);
  writeMax2870(cfg.regsDisabled[4]);
  writeMax2870(cfg.regsDisabled[3]);
  writeMax2870(cfg.regsDisabled[2]);
  writeMax2870(cfg.regsDisabled[1]);
  writeMax2870(cfg.regsDisabled[0]);

  writeRegisterBlock(cfg.regsDisabled);

  // Finally enable the RF outputs requested in this sketch.
  writeMax2870(cfg.regsEnabled[4]);
}

static bool waitForLock(uint32_t timeoutMs = 200) {
  uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    if (digitalRead(PIN_MAX_LD) == HIGH) return true;
  }
  return false;
}

// Convenience function for retuning from code.
static bool setFrequencyMHz(double targetMHz, double refMHz) {
  Max2870Config cfg;
  if (!calculateMax2870(targetMHz, refMHz, cfg)) {
    Serial.println(F("Failed to calculate a valid MAX2870 setup."));
    return false;
  }

  printConfig(cfg);
  programMax2870(cfg);

  Serial.println();
  if (waitForLock()) {
    Serial.println(F("PLL lock detected."));
  } else {
    Serial.println(F("No lock detected on LD pin (or LD not wired)."));
  }

  return true;
}

void setup() {
  pinMode(PIN_MAX_LE, OUTPUT);
  pinMode(PIN_MAX_CE, OUTPUT);
  pinMode(PIN_MAX_LD, INPUT);

  digitalWrite(PIN_MAX_LE, LOW);
  digitalWrite(PIN_MAX_CE, LOW);

  Serial.begin(115200);
  delay(1000);

  SPI.begin();

  Serial.println(F("MAX2870 calculator / programmer"));
  Serial.println(F("NUCLEO-F303K8 + STM32 Arduino core"));
  Serial.println(F("Programming default 1200 MHz from 100 MHz reference..."));

  setFrequencyMHz(DEFAULT_TARGET_MHZ, DEFAULT_REF_MHZ);

  // Example retune calls:
  // setFrequencyMHz(915.0, 100.0);
  // setFrequencyMHz(2450.0, 100.0);
  // setFrequencyMHz(1575.42, 100.0);
}

void loop() {
  // Nothing required here.
}
```

For the default **1200 MHz / 100 MHz** case, this sketch will pick **DIVA = 4**, **fVCO = 4800 MHz**, **fPFD = 100 MHz**, and **N = 48**, so it programs the part in **integer-N** mode. The startup register set printed by the sketch should show values consistent with that plan, including an R0 with `INT=1`, `N=48`, `FRAC=0`, and an R4 with divide-by-4 selected. Those values follow directly from the datasheet register map and equations. 

If you want, I can also give you a second version that adds **serial command parsing** so you can type new target/reference frequencies into the Arduino Serial Monitor and retune live.

***I have tested with various frequencies as output, when set to target of 100MHz and Reference of 100MHz I see RF output at 192MHz.***

I’m checking the register map against the official MAX2870 datasheet and comparing it to the sketch logic. A symptom like 100 MHz requested but 192 MHz observed usually points to one wrong field in R4/R2/R0 rather than a math problem.

The clue is that 100 MHz is the first band that needs the divide-by-32 output path. I’m checking whether the low-band divider / feedback mode needs a different treatment there, because that would neatly explain why higher frequencies look more plausible but 100 MHz does not.

Thought for 7m 57s

That symptom looks more like a **lock / divider-latch / measurement** issue than a bad `N/FRAC/MOD` calculation.

For **target = 100 MHz** and **reference = 100 MHz**, the MAX2870 math should be:

- `DIVA = 32` because the VCO must stay in the **3 GHz to 6 GHz** range, so `fVCO = 100 MHz × 32 = 3200 MHz`
- with `FB = 1`, `N + F/M = fVCO / fPFD`
- with `DBR = 0`, `RDIV2 = 0`, `R = 1`, the PFD is `100 MHz`
- so the correct solution is **integer-N**, with **N = 32**, **FRAC = 0** 

So the key frequency-setting fields for that case are:

- **R0 = `0x80100000`**  → `INT=1`, `N=32`, `FRAC=0`
- **R4 = `0x63DFF0FC`**  → `FB=1`, `DIVA=101` (divide by 32), `BS=1023`, RFOUTA enabled

That divider code is correct for the **93.75 MHz to 187.5 MHz** output range. 

The three things I would check first are:

1. **Is the PLL actually locked?**
    The datasheet says the **LD pin goes high when locked** and low when unlocked. It also says that for **integer-N mode** you must set **LDF = 1**, and for **fPFD > 32 MHz** you should set **LDS = 1**. In my earlier sketch, I set `LDF`, but I did **not** set `LDS`, so I would fix that. Also, if your loop filter was designed around a lower comparison frequency, a **100 MHz PFD** can make the loop behave badly even though it is still inside the MAX2870’s integer-N limit of **105 MHz**. 
2. **Are you measuring the true fundamental, or a harmonic / poorly terminated output?**
    The MAX2870 outputs are **differential open-collector** outputs and require the proper **RF choke or 50 Ω pull-up to supply**, and if you use one side single-ended, the unused side should be terminated. If the output network is not right, the spectrum can look odd. 
3. **Did the divide-by-32 change actually latch?**
    `DIVA` lives in **R4**, and the datasheet says it is **double-buffered by R0 when REG4DB = 1**. So whenever you cross into a new divider band, you must write **R4** and then write **R0** to transfer the divider change. If a test version of your code only rewrites R0, the divider can stay stale. 

Here is the patch I would make first:

```
static uint32_t buildReg1(bool integerMode, uint16_t mod) {
  uint32_t r = 0;

  if (integerMode) {
    r |= 1UL << 31;          // CPOC = 1 in int-N
    // CPL = 00 in int-N
  } else {
    r |= 1UL << 29;          // CPL = 01 in frac-N
    // CPOC = 0 in frac-N
  }

  r |= 1UL << 15;            // P = 1 (recommended)
  r |= (uint32_t)(mod & 0x0FFFU) << 3;
  r |= 1U;                   // addr
  return r;
}

static uint32_t buildReg2(bool integerMode, bool dbr, bool rdiv2,
                          uint16_t rCounter, double pfdHz) {
  uint32_t r = 0;

  if (pfdHz > 32e6) r |= 1UL << 31;   // LDS = 1 when fPFD > 32 MHz

  r |= (uint32_t)(dbr ? 1UL : 0UL)   << 25;
  r |= (uint32_t)(rdiv2 ? 1UL : 0UL) << 24;
  r |= (uint32_t)(rCounter & 0x03FFU) << 14;
  r |= 1UL << 13;                    // REG4DB = 1
  r |= 7UL << 9;                     // CP current = midscale debug value
  r |= (uint32_t)(integerMode ? 1UL : 0UL) << 8;  // LDF
  r |= 1UL << 6;                     // PDP = positive
  r |= 2U;                           // addr
  return r;
}
```

and call it like this:

```
cfg.regsDisabled[2] = buildReg2(cfg.integerMode, cfg.dbr, cfg.rdiv2,
                                cfg.rCounter, cfg.pfdHz);
```

The next debug step I would do is **force a lower PFD**, just to separate “bad math” from “loop/filter problem.” For your 100 MHz reference, force **R = 4**, so `fPFD = 25 MHz`. Then the correct 100 MHz solution becomes:

- `DIVA = 32`
- `fVCO = 3200 MHz`
- `N = 128`
- `FRAC = 0`

If that version locks and gives the right output, then your problem is almost certainly **not the divider math**, but the **100 MHz comparison frequency / loop behavior**. The datasheet allows 100 MHz PFD in integer-N mode, but the loop still has to be stable with your actual filter and board. 

One more thing: the MAX2870 shifts data in with **LE low**, on the **rising edge of CLK**, and latches the word on the **rising edge of LE**, with the **29 data bits MSB-first plus 3 address bits**. That matches **SPI mode 0, MSB first**, so the SPI mode in the sketch is not the first thing I would blame. 

If you want, I’ll give you a revised full sketch with:

- the `LDS` / `CPOC` fixes,
- an option to cap max PFD for debugging,
- and a serial dump that prints the exact expected register set for 100 MHz so you can compare it against what your board is receiving.

***A target of 120MHz works ok, but 110MHz outputs at 200MHz, a target of 100Mhz outputs at around 192MHz***

That pattern is very revealing: 120 MHz is in one divider band, while 110 MHz and 100 MHz both cross into the next lower band. I’m checking the exact low-band divider rules and whether one register field changes at the 117.1875 MHz boundary, because that would fit your results very closely.

Thought for 6m 2s

***It works with the original code at 100Mhz if I cycle the power on the MAX2870 before re-programming***

That’s a strong clue. I’m checking the datasheet’s startup/write sequencing and which registers are double-buffered, because “works after a power cycle” usually means the part is keeping old state and one of the retune writes isn’t fully resetting it.

That is the key clue: this is now much more consistent with a **re-initialization problem** than a frequency-calculation problem.

A real power cycle clears the MAX2870’s internal state. By contrast, the datasheet says that in **low-power mode the register values are retained**, and after power-up the device should be programmed **twice**, with at least **20 ms** between the two register sweeps, because the first write enables the device and the second starts the VCO selection process. It also says **DIVA in R4 is double-buffered by R0 when REG4DB = 1**. So “works after power-cycle, but not on a warm reprogram” points to the part not being cleanly re-started before the new tuning words are applied. 

It also means the 100 MHz math itself is probably fine. For 100 MHz output, the correct band is still **divide-by-32**, because the MAX2870 uses divide-by-32 for **93.75 MHz to 187.5 MHz**, and with `FB = 1` the relation is `N + F/M = fVCO / fPFD`. 

So I would change the code in two ways:

1. **Stop treating CE as a true reset.**
    Your observation suggests CE-low is not giving you the same clean state as removing power. That matches the datasheet note that register contents can be retained in low-power mode. 
2. **Do an explicit PLL reset and full re-init before every retune** that changes divider/reference settings.
    The datasheet provides an RST bit in **R2 bit 3** that resets the **R and N counters**. Use that, then write the full `R5 → R4 → R3 → R2 → R1 → R0` sequence twice, with outputs off, then enable RF. 

Use this patch:

```
static uint32_t buildReg2(bool integerMode, bool dbr, bool rdiv2,
                          uint16_t rCounter, double pfdHz, bool rst) {
  uint32_t r = 0;

  if (pfdHz > 32e6) r |= 1UL << 31;                 // LDS = 1 when fPFD > 32 MHz
  r |= (uint32_t)(dbr ? 1UL : 0UL)   << 25;
  r |= (uint32_t)(rdiv2 ? 1UL : 0UL) << 24;
  r |= (uint32_t)(rCounter & 0x03FFU) << 14;
  r |= 1UL << 13;                                   // REG4DB = 1
  r |= 7UL << 9;                                    // CP current: midscale debug value
  r |= (uint32_t)(integerMode ? 1UL : 0UL) << 8;   // LDF
  r |= 1UL << 6;                                    // PDP = positive
  if (rst) r |= 1UL << 3;                           // RST = 1 resets R and N counters
  r |= 2U;                                          // addr
  return r;
}

static void writeRegisterBlock(const uint32_t regs[6]) {
  writeMax2870(regs[5]);
  writeMax2870(regs[4]);
  writeMax2870(regs[3]);
  writeMax2870(regs[2]);
  writeMax2870(regs[1]);
  writeMax2870(regs[0]);
}

static void programMax2870Robust(Max2870Config &cfg) {
  // Rebuild R2 in both reset and normal forms
  uint32_t r2_normal = buildReg2(cfg.integerMode, cfg.dbr, cfg.rdiv2,
                                 cfg.rCounter, cfg.pfdHz, false);
  uint32_t r2_reset  = buildReg2(cfg.integerMode, cfg.dbr, cfg.rdiv2,
                                 cfg.rCounter, cfg.pfdHz, true);

  cfg.regsDisabled[2] = r2_normal;
  cfg.regsEnabled[2]  = r2_normal;

  // 1) Force counters reset with outputs disabled
  uint32_t regsReset[6];
  memcpy(regsReset, cfg.regsDisabled, sizeof(regsReset));
  regsReset[2] = r2_reset;

  // Optional: keep CE high; do not rely on CE low/high as a full reset
  digitalWrite(PIN_MAX_CE, HIGH);
  delay(1);

  // 2) First full pass with counters held reset
  writeRegisterBlock(regsReset);
  delay(20);

  // 3) Second full pass with counters released
  writeRegisterBlock(cfg.regsDisabled);
  delay(20);

  // 4) Third pass, same normal registers again, to be extra explicit
  writeRegisterBlock(cfg.regsDisabled);

  // 5) Enable RF outputs only after the PLL has been re-initialized
  writeMax2870(cfg.regsEnabled[4]);

  // Give VCO autoselect / settling a moment
  delay(2);
}
```

Then replace:

```
programMax2870(cfg);
```

with:

```
programMax2870Robust(cfg);
```

I would also make one debug simplification: for **100 / 110 / 120 MHz**, force a **fixed 20 MHz PFD** so all three frequencies use the same divider strategy and only `N` changes. With a 100 MHz reference that means:

- `DBR = 0`
- `R = 5`
- `RDIV2 = 0`
- `fPFD = 20 MHz`
- `DIVA = 32`

So:

- 100 MHz → `N = 160`
- 110 MHz → `N = 176`
- 120 MHz → `N = 192`

That removes the special 100 MHz `R=1, fPFD=100 MHz` case entirely and is a cleaner retune test. The datasheet allows this because `fVCO = fRFOUT × DIVA` and divide-by-32 is the correct output-divider band for all three frequencies. 

The shortest diagnosis is: **your retune path is not fully resetting the synthesizer state**, and a real power cycle is masking that. The fix is to do a more explicit reset/re-init sequence before each retune, rather than relying on CE toggling alone.





***MAX2870_example.ino***

***File***

***I wrote this some time ago and I think the double writing of the registers was critical. Is this referenced in the datasheet?***

***I’m checking the official MAX2870 datasheet wording directly. This is one of those cases where the exact phrasing matters, because there’s a difference between “write all registers twice at startup” and “rewrite R0 to latch buffered fields during retune.”***

Thought for 51s

Yes. The MAX2870 datasheet explicitly says, **“Upon power-up, the registers should be programmed twice with at least a 20ms pause between writes.”** It then explains why: **the first write enables the device, and the second starts the VCO selection process.** 

It goes further and gives a startup sequence for a clean clock: write **R5**, wait **20 ms**, then **R4, R3, R2, R1, R0**, and then repeat that same sequence a second time, keeping the RF outputs disabled until the end. 

There is also a second, separate reason “double writing” shows up in real code: the datasheet says **several bits are double-buffered**, and updates are applied together when the right trigger register is written. In particular, **R4.DIVA** is **double-buffered by R0 when REG4DB = 1**, so if you change output divider bands during a retune, you generally need the **R4 write followed by an R0 write** for the new divider to take effect. 

So the clean interpretation is:

- **At power-up:** yes, the datasheet explicitly calls for writing the full register set **twice**. 
- **During retunes:** the datasheet does **not** say “always write everything twice,” but it **does** say some fields are double-buffered, which is why many working drivers still do a full ordered rewrite, or at least ensure an **R0 strobe** after changing buffered fields. 

That matches your observation very well: your old code probably wasn’t just being cautious — it was following a real startup requirement from the datasheet, and it may also have been masking divider-latch issues on retune. 

I can also mark up your old `.ino` and point out exactly which parts are startup-mandatory versus just “robust retune practice.”

produce a new full sketch with the corrections

I’m rebuilding the sketch end-to-end rather than patching fragments, so the startup sequence, buffered-register latching, and retune behavior are all consistent.

Thought for 9m 6s

I rebuilt the sketch around the two fixes that mattered most in your testing: a full **R5→R0 double-write sequence with the required 20 ms pause**, and a **warm-retune path that reprograms with outputs off before enabling RFOUTA again**. I also kept `REG4DB=1` and force an **R4 then R0 latch** before the full rewrite, because `DIVA` is double-buffered by R0 when `REG4DB` is enabled. The sketch uses a conservative **25 MHz PFD ceiling** by default to make low-band retunes less fragile; that is a design choice in the sketch, not a MAX2870 limit. The device itself supports up to **105 MHz PFD in integer-N** and **50 MHz in fractional-N**, retains register contents in low-power mode, and the datasheet explicitly requires programming the registers twice after power-up. 

I also updated the SPI setup for the current STM32 Arduino core style, using `SPI.setMOSI(...)` and `SPI.setSCLK(...)` before `SPI.begin()`. The file keeps the same pin choices as your older sketch by default: **D3 = DATA, D2 = CLK, D5 = LE, D6 = CE, D4 = LD**. 

Here is the corrected full sketch:

Download the corrected sketch

The two lines you’re most likely to tweak first are near the top:

```
constexpr double USER_MAX_PFD_HZ = 25.0e6;
constexpr uint8_t PIN_SPI_MOSI = D3;
constexpr uint8_t PIN_SPI_SCK  = D2;
```

If you want, I can also produce a second version that keeps your exact old register style but patches only the retune/programming sequence.

***I can't get the download of the corrected sketch to work***

I’ll paste the full sketch directly here so you don’t have to rely on the download link. I’m also checking the saved file so I can reattach it with a simpler name.

Thought for 42s

Here it is directly in the chat, and I also saved it again with a shorter filename:

Download max2870.ino

```
#include <SPI.h>

/*
  MAX2870 frequency planner + programmer
  Target: NUCLEO-F303K8 using STM32 Arduino core

  Default wiring (matches the old example you uploaded):
    D3 -> MAX2870 DATA
    D2 -> MAX2870 CLK
    D5 -> MAX2870 LE
    D6 -> MAX2870 CE
    D4 -> MAX2870 LD  (optional, lock detect input)

  Notes:
    - Uses a conservative default max PFD of 25 MHz for robust retunes.
    - Programs the MAX2870 with the datasheet's double-write startup sequence.
    - Reuses the same full sequence on every retune to avoid stale buffered state.
    - Keeps RF outputs off during programming, then enables RFOUTA at the end.
*/

// ------------------------- User configuration -------------------------

constexpr double DEFAULT_TARGET_MHZ = 1200.0;
constexpr double DEFAULT_REF_MHZ    = 100.0;

// Conservative PFD ceiling for generic loop filters.
// The MAX2870 itself allows up to 105 MHz in integer-N mode and 50 MHz in fractional-N mode.
constexpr double USER_MAX_PFD_HZ = 25.0e6;

// STM32 Arduino pin selections. These are the same logical pins as the older sketch.
constexpr uint8_t PIN_SPI_MOSI = D3;
constexpr uint8_t PIN_SPI_SCK  = D2;
constexpr uint8_t PIN_LE       = D5;
constexpr uint8_t PIN_CE       = D6;
constexpr uint8_t PIN_LD       = D4;   // optional

constexpr bool    ENABLE_RFOUT_A = true;
constexpr bool    ENABLE_RFOUT_B = false;
constexpr uint8_t RFOUTA_POWER   = 3;  // 0=-4dBm, 1=-1dBm, 2=+2dBm, 3=+5dBm
constexpr uint8_t RFOUTB_POWER   = 0;

SPISettings max2870Spi(1000000, MSBFIRST, SPI_MODE0);

// ---------------------------- MAX2870 limits --------------------------

constexpr double MIN_RF_HZ          = 23.5e6;
constexpr double MAX_RF_HZ          = 6000.0e6;
constexpr double MIN_VCO_HZ         = 3000.0e6;
constexpr double MAX_VCO_HZ         = 6000.0e6;
constexpr double MAX_PFD_INT_HZ     = 105.0e6;
constexpr double MAX_PFD_FRAC_HZ    = 50.0e6;
constexpr uint16_t MAX_R_COUNTER    = 1023;
constexpr uint16_t MOD_MAX          = 4095;

// ---------------------------- Helper types ----------------------------

struct Max2870Plan {
  bool valid        = false;
  bool integerMode  = true;

  double targetHz   = 0.0;
  double refHz      = 0.0;
  double actualHz   = 0.0;
  double errorHz    = 0.0;
  double vcoHz      = 0.0;
  double pfdHz      = 0.0;

  bool dbr          = false;
  bool rdiv2        = false;
  uint16_t r        = 1;

  uint16_t n        = 0;
  uint16_t frac     = 0;
  uint16_t mod      = MOD_MAX;

  uint8_t divider   = 1;    // actual RF divider 1/2/4/.../128
  uint8_t divaCode  = 0;    // register encoding 0..7
  uint16_t bs       = 1;    // band select clock divider

  uint32_t regOff[6] = {0}; // R0..R5 with outputs disabled
  uint32_t regOn[6]  = {0}; // R0..R5 with outputs enabled
};

// ---------------------------- Small helpers ---------------------------

static uint16_t gcd16(uint16_t a, uint16_t b) {
  while (b != 0) {
    uint16_t t = a % b;
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

static void printHex32(const char *label, uint32_t value) {
  char buf[11];
  snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)value);
  Serial.print(label);
  Serial.println(buf);
}

// -------------------------- Frequency planning ------------------------

static bool chooseOutputDivider(double targetHz, uint8_t &divider, uint8_t &divaCode, double &vcoHz) {
  const uint8_t dividers[8] = {1, 2, 4, 8, 16, 32, 64, 128};

  for (uint8_t code = 0; code < 8; ++code) {
    const double candidateVco = targetHz * (double)dividers[code];
    if (candidateVco >= MIN_VCO_HZ && candidateVco <= MAX_VCO_HZ) {
      divider  = dividers[code];
      divaCode = code;
      vcoHz    = candidateVco;
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

      for (uint16_t r = 1; r <= MAX_R_COUNTER; ++r) {
        const double pfd = p.refHz * (dbr ? 2.0 : 1.0) / ((double)r * denom2);
        if (pfd > USER_MAX_PFD_HZ) continue;
        if (pfd > MAX_PFD_INT_HZ) continue;

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
  double bestError = 1e99;
  double bestPfd = -1.0;

  for (uint8_t dbr = 0; dbr <= 1; ++dbr) {
    if (dbr && p.refHz > 100.0e6) continue;

    for (uint8_t rdiv2 = 0; rdiv2 <= 1; ++rdiv2) {
      const double denom2 = rdiv2 ? 2.0 : 1.0;

      for (uint16_t r = 1; r <= MAX_R_COUNTER; ++r) {
        const double pfd = p.refHz * (dbr ? 2.0 : 1.0) / ((double)r * denom2);
        if (pfd > USER_MAX_PFD_HZ) continue;
        if (pfd > MAX_PFD_FRAC_HZ) continue;

        const double ratio = p.vcoHz / pfd;
        const double nFloor = floor(ratio);
        if (nFloor < 19.0 || nFloor > 4091.0) continue;

        uint16_t mod  = MOD_MAX;
        uint16_t frac = (uint16_t)llround((ratio - nFloor) * (double)mod);
        uint16_t n    = (uint16_t)nFloor;

        if (frac >= mod) {
          frac = 0;
          ++n;
        }
        if (n < 19 || n > 4091) continue;

        if (frac != 0) {
          const uint16_t g = gcd16(frac, mod);
          frac /= g;
          mod  /= g;
          if (mod < 2) mod = 2;
        }

        const double actualVco = ((double)n + ((double)frac / (double)mod)) * pfd;
        const double actualRf  = actualVco / (double)p.divider;
        const double err       = fabs(actualRf - p.targetHz);

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

// -------------------------- Register builders -------------------------

static uint32_t buildR0(const Max2870Plan &p) {
  uint32_t r = 0;
  if (p.integerMode) r |= (1UL << 31);           // INT
  r |= ((uint32_t)p.n    & 0xFFFFUL) << 15;      // N
  r |= ((uint32_t)p.frac & 0x0FFFUL) << 3;       // FRAC
  r |= 0U;                                       // ADDR=0
  return r;
}

static uint32_t buildR1(const Max2870Plan &p) {
  uint32_t r = 0;

  if (p.integerMode) {
    r |= (1UL << 31);    // CPOC=1 in integer-N
    // CPL=00 in integer-N
  } else {
    r |= (1UL << 29);    // CPL=01 in fractional-N
  }

  r |= (1UL << 15);                                // P = 1
  r |= ((uint32_t)p.mod & 0x0FFFUL) << 3;         // MOD
  r |= 1U;                                        // ADDR=1
  return r;
}

static uint32_t buildR2(const Max2870Plan &p, bool counterReset) {
  uint32_t r = 0;

  if (p.pfdHz > 32.0e6) {
    r |= (1UL << 31);                             // LDS
  }

  if (p.dbr)   r |= (1UL << 25);
  if (p.rdiv2) r |= (1UL << 24);

  r |= ((uint32_t)p.r & 0x03FFUL) << 14;         // R
  r |= (1UL << 13);                              // REG4DB = 1
  r |= (7UL << 9);                               // CP current example default
  if (p.integerMode) r |= (1UL << 8);            // LDF
  r |= (1UL << 6);                               // PDP = positive
  if (counterReset) r |= (1UL << 3);             // RST
  r |= 2U;                                       // ADDR=2
  return r;
}

static uint32_t buildR3() {
  return 0x0000000BUL;
}

static uint32_t buildR4(const Max2870Plan &p, bool enableA, bool enableB) {
  uint32_t r = 0;

  r |= (0b011000UL << 26);                       // reserved bits
  r |= ((uint32_t)((p.bs >> 8) & 0x03U)) << 24; // BS[9:8]
  r |= (1UL << 23);                              // FB = 1
  r |= ((uint32_t)p.divaCode & 0x07U) << 20;    // DIVA
  r |= ((uint32_t)(p.bs & 0xFFU)) << 12;        // BS[7:0]

  if (enableB) r |= (1UL << 8);                 // RFB_EN
  r |= ((uint32_t)RFOUTB_POWER & 0x03U) << 6;

  if (enableA) r |= (1UL << 5);                 // RFA_EN
  r |= ((uint32_t)RFOUTA_POWER & 0x03U) << 3;

  r |= 4U;                                      // ADDR=4
  return r;
}

static uint32_t buildR5(const Max2870Plan &p) {
  uint32_t r = 0x00400005UL;                    // LD=digital lock detect

  if (p.integerMode || p.frac == 0) {
    r |= (1UL << 24);                           // F01
  }

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
  p.regOn[4] = buildR4(p, ENABLE_RFOUT_A, ENABLE_RFOUT_B);
  p.regOn[5] = p.regOff[5];
}

static bool makePlan(double targetMHz, double refMHz, Max2870Plan &p) {
  p = Max2870Plan();
  p.targetHz = targetMHz * 1.0e6;
  p.refHz    = refMHz * 1.0e6;

  if (p.targetHz < MIN_RF_HZ || p.targetHz > MAX_RF_HZ) return false;
  if (!chooseOutputDivider(p.targetHz, p.divider, p.divaCode, p.vcoHz)) return false;

  if (!planInteger(p)) {
    if (!planFractional(p)) return false;
  }

  buildRegisters(p);
  p.valid = true;
  return true;
}

// ----------------------------- SPI writes -----------------------------

static void max2870Write(uint32_t value) {
  SPI.beginTransaction(max2870Spi);
  digitalWrite(PIN_LE, LOW);
  SPI.transfer((uint8_t)((value >> 24) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 16) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 8)  & 0xFFU));
  SPI.transfer((uint8_t)( value        & 0xFFU));
  digitalWrite(PIN_LE, HIGH);
  delayMicroseconds(2);
  digitalWrite(PIN_LE, LOW);
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

static void programMax2870(const Max2870Plan &p) {
  digitalWrite(PIN_CE, HIGH);

  uint32_t resetRegs[6];
  for (uint8_t i = 0; i < 6; ++i) resetRegs[i] = p.regOff[i];
  resetRegs[2] = buildR2(p, true);

  // Disable outputs and latch buffered divider changes
  max2870Write(p.regOff[4]);
  max2870Write(p.regOff[0]);

  // First pass with reset asserted
  max2870Write(resetRegs[5]);
  delay(20);
  max2870Write(resetRegs[4]);
  max2870Write(resetRegs[3]);
  max2870Write(resetRegs[2]);
  max2870Write(resetRegs[1]);
  max2870Write(resetRegs[0]);

  // Second pass normal
  max2870Write(p.regOff[5]);
  max2870Write(p.regOff[4]);
  max2870Write(p.regOff[3]);
  max2870Write(p.regOff[2]);
  max2870Write(p.regOff[1]);
  max2870Write(p.regOff[0]);

  // Third pass for robust warm retune behavior
  writeBlock(p.regOff);

  // Enable outputs last
  max2870Write(p.regOn[4]);
}

// ------------------------------- Debug --------------------------------

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
  Serial.println();

  Serial.println(F("Registers with outputs OFF"));
  printHex32("R5 = ", p.regOff[5]);
  printHex32("R4 = ", p.regOff[4]);
  printHex32("R3 = ", p.regOff[3]);
  printHex32("R2 = ", p.regOff[2]);
  printHex32("R1 = ", p.regOff[1]);
  printHex32("R0 = ", p.regOff[0]);

  Serial.println();
  Serial.println(F("Registers with outputs ON"));
  printHex32("R5 = ", p.regOn[5]);
  printHex32("R4 = ", p.regOn[4]);
  printHex32("R3 = ", p.regOn[3]);
  printHex32("R2 = ", p.regOn[2]);
  printHex32("R1 = ", p.regOn[1]);
  printHex32("R0 = ", p.regOn[0]);
}

static bool waitForLock(uint32_t timeoutMs = 250) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    if (digitalRead(PIN_LD) == HIGH) return true;
  }
  return false;
}

static bool tuneMax2870(double targetMHz, double refMHz) {
  Max2870Plan p;
  if (!makePlan(targetMHz, refMHz, p)) {
    Serial.println(F("Unable to find a valid MAX2870 plan for that frequency pair."));
    return false;
  }

  printPlan(p);
  programMax2870(p);

  if (waitForLock()) {
    Serial.println(F("Lock detect: LOCKED"));
  } else {
    Serial.println(F("Lock detect: not asserted (or LD not wired)."));
  }

  return true;
}

// -------------------------- Optional serial UI ------------------------

static void serviceSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;

  const int sep = line.indexOf(' ');
  if (sep < 0) {
    Serial.println(F("Enter: <target_MHz> <ref_MHz>   e.g. 110 100"));
    return;
  }

  const double targetMHz = line.substring(0, sep).toFloat();
  const double refMHz    = line.substring(sep + 1).toFloat();

  if (targetMHz <= 0.0 || refMHz <= 0.0) {
    Serial.println(F("Bad input. Example: 1200 100"));
    return;
  }

  tuneMax2870(targetMHz, refMHz);
}

// -------------------------------- Setup --------------------------------

void setup() {
  pinMode(PIN_LE, OUTPUT);
  pinMode(PIN_CE, OUTPUT);
  pinMode(PIN_LD, INPUT);

  digitalWrite(PIN_LE, LOW);
  digitalWrite(PIN_CE, HIGH);

  Serial.begin(115200);
  delay(1000);

  SPI.setMOSI(PIN_SPI_MOSI);
  SPI.setSCLK(PIN_SPI_SCK);
  SPI.begin();

  Serial.println(F("MAX2870 corrected sketch"));
  Serial.println(F("Default tune: 1200 MHz from 100 MHz reference"));
  Serial.println(F("Serial command format: <target_MHz> <ref_MHz>"));

  tuneMax2870(DEFAULT_TARGET_MHZ, DEFAULT_REF_MHZ);
}

void loop() {
  serviceSerial();
}
```

One thing to watch: this version assumes your MAX2870 **DATA and CLK really are on D3 and D2** because it remaps SPI to those pins. If your hardware is wired to the board’s default SPI pins instead, tell me your exact wiring and I’ll rewrite the pin block accordingly.

***NUCLEO D13 (PB3 / SPI1_SCK)  -> MAX2870 CLK NUCLEO D11 (PB5 / SPI1_MOSI) -> MAX2870 DATA NUCLEO D10                   -> MAX2870 LE NUCLEO D9                    -> MAX2870 CE NUCLEO D2  (optional)        -> MAX2870 LD   (digital lock detect)***

Thought for 22s

Perfect — with that wiring, use the board’s **hardware SPI pins** directly:

- **D13** → SCK → MAX2870 **CLK**
- **D11** → MOSI → MAX2870 **DATA**
- **D10** → MAX2870 **LE**
- **D9** → MAX2870 **CE**
- **D2** → MAX2870 **LD**

That means the previous sketch should be changed as follows.

### Replace the pin definitions with this

```
constexpr uint8_t PIN_SPI_MOSI = D11;
constexpr uint8_t PIN_SPI_SCK  = D13;
constexpr uint8_t PIN_LE       = D10;
constexpr uint8_t PIN_CE       = D9;
constexpr uint8_t PIN_LD       = D2;
```

### In `setup()`, use this SPI init

```
void setup() {
  pinMode(PIN_LE, OUTPUT);
  pinMode(PIN_CE, OUTPUT);
  pinMode(PIN_LD, INPUT);

  digitalWrite(PIN_LE, LOW);
  digitalWrite(PIN_CE, HIGH);

  Serial.begin(115200);
  delay(1000);

  // Hardware SPI on NUCLEO-F303K8:
  // D11 = MOSI, D13 = SCK
  SPI.setMOSI(PIN_SPI_MOSI);
  SPI.setSCLK(PIN_SPI_SCK);
  SPI.begin();

  Serial.println(F("MAX2870 corrected sketch"));
  Serial.println(F("Default tune: 1200 MHz from 100 MHz reference"));
  Serial.println(F("Serial command format: <target_MHz> <ref_MHz>"));

  tuneMax2870(DEFAULT_TARGET_MHZ, DEFAULT_REF_MHZ);
}
```

### The SPI write routine stays like this

```
static void max2870Write(uint32_t value) {
  SPI.beginTransaction(max2870Spi);
  digitalWrite(PIN_LE, LOW);

  SPI.transfer((uint8_t)((value >> 24) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 16) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 8)  & 0xFFU));
  SPI.transfer((uint8_t)( value        & 0xFFU));

  digitalWrite(PIN_LE, HIGH);
  delayMicroseconds(2);
  digitalWrite(PIN_LE, LOW);
  SPI.endTransaction();
}
```

The main correction is that the earlier sketch version used **D3/D2 for SPI**, which does **not** match your wiring. With your board connected to **D11/D13**, the MAX2870 would have been seeing the wrong clock/data pins.

Here is the full sketch with your exact wiring applied:

```
#include <SPI.h>

/*
  MAX2870 frequency planner + programmer
  Target: NUCLEO-F303K8 using STM32 Arduino core

  Wiring:
    D13 -> MAX2870 CLK
    D11 -> MAX2870 DATA
    D10 -> MAX2870 LE
    D9  -> MAX2870 CE
    D2  -> MAX2870 LD  (optional)
*/

// ------------------------- User configuration -------------------------

constexpr double DEFAULT_TARGET_MHZ = 1200.0;
constexpr double DEFAULT_REF_MHZ    = 100.0;

// Conservative PFD ceiling for generic loop filters
constexpr double USER_MAX_PFD_HZ = 25.0e6;

// Correct hardware SPI pins for your wiring
constexpr uint8_t PIN_SPI_MOSI = D11;
constexpr uint8_t PIN_SPI_SCK  = D13;
constexpr uint8_t PIN_LE       = D10;
constexpr uint8_t PIN_CE       = D9;
constexpr uint8_t PIN_LD       = D2;

constexpr bool    ENABLE_RFOUT_A = true;
constexpr bool    ENABLE_RFOUT_B = false;
constexpr uint8_t RFOUTA_POWER   = 3;
constexpr uint8_t RFOUTB_POWER   = 0;

SPISettings max2870Spi(1000000, MSBFIRST, SPI_MODE0);

// ---------------------------- MAX2870 limits --------------------------

constexpr double MIN_RF_HZ          = 23.5e6;
constexpr double MAX_RF_HZ          = 6000.0e6;
constexpr double MIN_VCO_HZ         = 3000.0e6;
constexpr double MAX_VCO_HZ         = 6000.0e6;
constexpr double MAX_PFD_INT_HZ     = 105.0e6;
constexpr double MAX_PFD_FRAC_HZ    = 50.0e6;
constexpr uint16_t MAX_R_COUNTER    = 1023;
constexpr uint16_t MOD_MAX          = 4095;

// ---------------------------- Helper types ----------------------------

struct Max2870Plan {
  bool valid        = false;
  bool integerMode  = true;

  double targetHz   = 0.0;
  double refHz      = 0.0;
  double actualHz   = 0.0;
  double errorHz    = 0.0;
  double vcoHz      = 0.0;
  double pfdHz      = 0.0;

  bool dbr          = false;
  bool rdiv2        = false;
  uint16_t r        = 1;

  uint16_t n        = 0;
  uint16_t frac     = 0;
  uint16_t mod      = MOD_MAX;

  uint8_t divider   = 1;
  uint8_t divaCode  = 0;
  uint16_t bs       = 1;

  uint32_t regOff[6] = {0};
  uint32_t regOn[6]  = {0};
};

static uint16_t gcd16(uint16_t a, uint16_t b) {
  while (b != 0) {
    uint16_t t = a % b;
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

static void printHex32(const char *label, uint32_t value) {
  char buf[11];
  snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)value);
  Serial.print(label);
  Serial.println(buf);
}

static bool chooseOutputDivider(double targetHz, uint8_t &divider, uint8_t &divaCode, double &vcoHz) {
  const uint8_t dividers[8] = {1, 2, 4, 8, 16, 32, 64, 128};

  for (uint8_t code = 0; code < 8; ++code) {
    const double candidateVco = targetHz * (double)dividers[code];
    if (candidateVco >= MIN_VCO_HZ && candidateVco <= MAX_VCO_HZ) {
      divider  = dividers[code];
      divaCode = code;
      vcoHz    = candidateVco;
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

      for (uint16_t r = 1; r <= MAX_R_COUNTER; ++r) {
        const double pfd = p.refHz * (dbr ? 2.0 : 1.0) / ((double)r * denom2);
        if (pfd > USER_MAX_PFD_HZ) continue;
        if (pfd > MAX_PFD_INT_HZ) continue;

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
  double bestError = 1e99;
  double bestPfd = -1.0;

  for (uint8_t dbr = 0; dbr <= 1; ++dbr) {
    if (dbr && p.refHz > 100.0e6) continue;

    for (uint8_t rdiv2 = 0; rdiv2 <= 1; ++rdiv2) {
      const double denom2 = rdiv2 ? 2.0 : 1.0;

      for (uint16_t r = 1; r <= MAX_R_COUNTER; ++r) {
        const double pfd = p.refHz * (dbr ? 2.0 : 1.0) / ((double)r * denom2);
        if (pfd > USER_MAX_PFD_HZ) continue;
        if (pfd > MAX_PFD_FRAC_HZ) continue;

        const double ratio = p.vcoHz / pfd;
        const double nFloor = floor(ratio);
        if (nFloor < 19.0 || nFloor > 4091.0) continue;

        uint16_t mod  = MOD_MAX;
        uint16_t frac = (uint16_t)llround((ratio - nFloor) * (double)mod);
        uint16_t n    = (uint16_t)nFloor;

        if (frac >= mod) {
          frac = 0;
          ++n;
        }
        if (n < 19 || n > 4091) continue;

        if (frac != 0) {
          const uint16_t g = gcd16(frac, mod);
          frac /= g;
          mod  /= g;
          if (mod < 2) mod = 2;
        }

        const double actualVco = ((double)n + ((double)frac / (double)mod)) * pfd;
        const double actualRf  = actualVco / (double)p.divider;
        const double err       = fabs(actualRf - p.targetHz);

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

static uint32_t buildR0(const Max2870Plan &p) {
  uint32_t r = 0;
  if (p.integerMode) r |= (1UL << 31);
  r |= ((uint32_t)p.n    & 0xFFFFUL) << 15;
  r |= ((uint32_t)p.frac & 0x0FFFUL) << 3;
  r |= 0U;
  return r;
}

static uint32_t buildR1(const Max2870Plan &p) {
  uint32_t r = 0;

  if (p.integerMode) {
    r |= (1UL << 31);
  } else {
    r |= (1UL << 29);
  }

  r |= (1UL << 15);
  r |= ((uint32_t)p.mod & 0x0FFFUL) << 3;
  r |= 1U;
  return r;
}

static uint32_t buildR2(const Max2870Plan &p, bool counterReset) {
  uint32_t r = 0;

  if (p.pfdHz > 32.0e6) r |= (1UL << 31);
  if (p.dbr)   r |= (1UL << 25);
  if (p.rdiv2) r |= (1UL << 24);

  r |= ((uint32_t)p.r & 0x03FFUL) << 14;
  r |= (1UL << 13);
  r |= (7UL << 9);
  if (p.integerMode) r |= (1UL << 8);
  r |= (1UL << 6);
  if (counterReset) r |= (1UL << 3);
  r |= 2U;
  return r;
}

static uint32_t buildR3() {
  return 0x0000000BUL;
}

static uint32_t buildR4(const Max2870Plan &p, bool enableA, bool enableB) {
  uint32_t r = 0;

  r |= (0b011000UL << 26);
  r |= ((uint32_t)((p.bs >> 8) & 0x03U)) << 24;
  r |= (1UL << 23);
  r |= ((uint32_t)p.divaCode & 0x07U) << 20;
  r |= ((uint32_t)(p.bs & 0xFFU)) << 12;

  if (enableB) r |= (1UL << 8);
  r |= ((uint32_t)RFOUTB_POWER & 0x03U) << 6;

  if (enableA) r |= (1UL << 5);
  r |= ((uint32_t)RFOUTA_POWER & 0x03U) << 3;

  r |= 4U;
  return r;
}

static uint32_t buildR5(const Max2870Plan &p) {
  uint32_t r = 0x00400005UL;
  if (p.integerMode || p.frac == 0) r |= (1UL << 24);
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
  p.regOn[4] = buildR4(p, ENABLE_RFOUT_A, ENABLE_RFOUT_B);
  p.regOn[5] = p.regOff[5];
}

static bool makePlan(double targetMHz, double refMHz, Max2870Plan &p) {
  p = Max2870Plan();
  p.targetHz = targetMHz * 1.0e6;
  p.refHz    = refMHz * 1.0e6;

  if (p.targetHz < MIN_RF_HZ || p.targetHz > MAX_RF_HZ) return false;
  if (!chooseOutputDivider(p.targetHz, p.divider, p.divaCode, p.vcoHz)) return false;

  if (!planInteger(p)) {
    if (!planFractional(p)) return false;
  }

  buildRegisters(p);
  p.valid = true;
  return true;
}

static void max2870Write(uint32_t value) {
  SPI.beginTransaction(max2870Spi);
  digitalWrite(PIN_LE, LOW);
  SPI.transfer((uint8_t)((value >> 24) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 16) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 8)  & 0xFFU));
  SPI.transfer((uint8_t)( value        & 0xFFU));
  digitalWrite(PIN_LE, HIGH);
  delayMicroseconds(2);
  digitalWrite(PIN_LE, LOW);
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

static void programMax2870(const Max2870Plan &p) {
  digitalWrite(PIN_CE, HIGH);

  uint32_t resetRegs[6];
  for (uint8_t i = 0; i < 6; ++i) resetRegs[i] = p.regOff[i];
  resetRegs[2] = buildR2(p, true);

  max2870Write(p.regOff[4]);
  max2870Write(p.regOff[0]);

  max2870Write(resetRegs[5]);
  delay(20);
  max2870Write(resetRegs[4]);
  max2870Write(resetRegs[3]);
  max2870Write(resetRegs[2]);
  max2870Write(resetRegs[1]);
  max2870Write(resetRegs[0]);

  max2870Write(p.regOff[5]);
  max2870Write(p.regOff[4]);
  max2870Write(p.regOff[3]);
  max2870Write(p.regOff[2]);
  max2870Write(p.regOff[1]);
  max2870Write(p.regOff[0]);

  writeBlock(p.regOff);

  max2870Write(p.regOn[4]);
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
  Serial.println();

  Serial.println(F("Registers with outputs OFF"));
  printHex32("R5 = ", p.regOff[5]);
  printHex32("R4 = ", p.regOff[4]);
  printHex32("R3 = ", p.regOff[3]);
  printHex32("R2 = ", p.regOff[2]);
  printHex32("R1 = ", p.regOff[1]);
  printHex32("R0 = ", p.regOff[0]);

  Serial.println();
  Serial.println(F("Registers with outputs ON"));
  printHex32("R5 = ", p.regOn[5]);
  printHex32("R4 = ", p.regOn[4]);
  printHex32("R3 = ", p.regOn[3]);
  printHex32("R2 = ", p.regOn[2]);
  printHex32("R1 = ", p.regOn[1]);
  printHex32("R0 = ", p.regOn[0]);
}

static bool waitForLock(uint32_t timeoutMs = 250) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    if (digitalRead(PIN_LD) == HIGH) return true;
  }
  return false;
}

static bool tuneMax2870(double targetMHz, double refMHz) {
  Max2870Plan p;
  if (!makePlan(targetMHz, refMHz, p)) {
    Serial.println(F("Unable to find a valid MAX2870 plan for that frequency pair."));
    return false;
  }

  printPlan(p);
  programMax2870(p);

  if (waitForLock()) {
    Serial.println(F("Lock detect: LOCKED"));
  } else {
    Serial.println(F("Lock detect: not asserted (or LD not wired)."));
  }

  return true;
}

static void serviceSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (!line.length()) return;

  const int sep = line.indexOf(' ');
  if (sep < 0) {
    Serial.println(F("Enter: <target_MHz> <ref_MHz>   e.g. 110 100"));
    return;
  }

  const double targetMHz = line.substring(0, sep).toFloat();
  const double refMHz    = line.substring(sep + 1).toFloat();

  if (targetMHz <= 0.0 || refMHz <= 0.0) {
    Serial.println(F("Bad input. Example: 1200 100"));
    return;
  }

  tuneMax2870(targetMHz, refMHz);
}

void setup() {
  pinMode(PIN_LE, OUTPUT);
  pinMode(PIN_CE, OUTPUT);
  pinMode(PIN_LD, INPUT);

  digitalWrite(PIN_LE, LOW);
  digitalWrite(PIN_CE, HIGH);

  Serial.begin(115200);
  delay(1000);

  SPI.setMOSI(PIN_SPI_MOSI);
  SPI.setSCLK(PIN_SPI_SCK);
  SPI.begin();

  Serial.println(F("MAX2870 corrected sketch"));
  Serial.println(F("Default tune: 1200 MHz from 100 MHz reference"));
  Serial.println(F("Serial command format: <target_MHz> <ref_MHz>"));

  tuneMax2870(DEFAULT_TARGET_MHZ, DEFAULT_REF_MHZ);
}

void loop() {
  serviceSerial();
}
```

The most important change is simply this:

```
constexpr uint8_t PIN_SPI_MOSI = D11;
constexpr uint8_t PIN_SPI_SCK  = D13;
constexpr uint8_t PIN_LE       = D10;
constexpr uint8_t PIN_CE       = D9;
constexpr uint8_t PIN_LD       = D2;
```

If you want the sketch saved again as a downloadable `.ino` with these exact pins, say `save it again` and I’ll generate that version.

***In file included from /home/g0mgx/.arduino15/packages/STMicroelectronics/hardware/stm32/2.12.0/cores/arduino/Arduino.h:56,                 from /home/g0mgx/.cache/arduino/sketches/EA2A1831AAAD0C327B1859B144B1C8F0/sketch/Vibe_MAX2870_2.ino.cpp:1: /home/g0mgx/.arduino15/packages/STMicroelectronics/hardware/stm32/2.12.0/cores/arduino/pins_arduino.h:58:37: error: expected unqualified-id before numeric constant   58 |   #define PIN_SPI_MOSI              11      |                                     ^~ /home/g0mgx/Documents/Arduino/Vibe_MAX2870_2/Vibe_MAX2870_2.ino:24:19: note: in expansion of macro 'PIN_SPI_MOSI'   24 | constexpr uint8_t PIN_SPI_MOSI = D11;      |                   ^~~~~~~~~~~~ /home/g0mgx/.arduino15/packages/STMicroelectronics/hardware/stm32/2.12.0/cores/arduino/pins_arduino.h:64:37: error: expected unqualified-id before numeric constant   64 |   #define PIN_SPI_SCK               13      |                                     ^~ /home/g0mgx/Documents/Arduino/Vibe_MAX2870_2/Vibe_MAX2870_2.ino:25:19: note: in expansion of macro 'PIN_SPI_SCK'   25 | constexpr uint8_t PIN_SPI_SCK  = D13;      |                   ^~~~~~~~~~~ exit status 1 Compilation error: exit status 1***

Thought for 9s

That compile error is because the STM32 core already defines these as macros:

- `PIN_SPI_MOSI`
- `PIN_SPI_SCK`

So my variable names collided with the core’s names.

Use different names, for example:

```
constexpr uint8_t MAX_SPI_MOSI = D11;
constexpr uint8_t MAX_SPI_SCK  = D13;
constexpr uint8_t PIN_LE       = D10;
constexpr uint8_t PIN_CE       = D9;
constexpr uint8_t PIN_LD       = D2;
```

Then in `setup()` change:

```
SPI.setMOSI(PIN_SPI_MOSI);
SPI.setSCLK(PIN_SPI_SCK);
```

to:

```
SPI.setMOSI(MAX_SPI_MOSI);
SPI.setSCLK(MAX_SPI_SCK);
```

So the fixed block is:

```
constexpr double DEFAULT_TARGET_MHZ = 1200.0;
constexpr double DEFAULT_REF_MHZ    = 100.0;

constexpr double USER_MAX_PFD_HZ = 25.0e6;

// renamed to avoid collision with STM32 core macros
constexpr uint8_t MAX_SPI_MOSI = D11;
constexpr uint8_t MAX_SPI_SCK  = D13;
constexpr uint8_t PIN_LE       = D10;
constexpr uint8_t PIN_CE       = D9;
constexpr uint8_t PIN_LD       = D2;

constexpr bool    ENABLE_RFOUT_A = true;
constexpr bool    ENABLE_RFOUT_B = false;
constexpr uint8_t RFOUTA_POWER   = 3;
constexpr uint8_t RFOUTB_POWER   = 0;
```

and:

```
void setup() {
  pinMode(PIN_LE, OUTPUT);
  pinMode(PIN_CE, OUTPUT);
  pinMode(PIN_LD, INPUT);

  digitalWrite(PIN_LE, LOW);
  digitalWrite(PIN_CE, HIGH);

  Serial.begin(115200);
  delay(1000);

  SPI.setMOSI(MAX_SPI_MOSI);
  SPI.setSCLK(MAX_SPI_SCK);
  SPI.begin();

  Serial.println(F("MAX2870 corrected sketch"));
  Serial.println(F("Default tune: 1200 MHz from 100 MHz reference"));
  Serial.println(F("Serial command format: <target_MHz> <ref_MHz>"));

  tuneMax2870(DEFAULT_TARGET_MHZ, DEFAULT_REF_MHZ);
}
```

An even simpler option is to remove those two custom SPI pin constants entirely and just use the board defaults:

```
SPI.begin();
```

since your wiring already matches the default hardware SPI pins on the Nucleo-32.

If you want, I can paste the full sketch again with that naming fix applied.





***MCK_Vibe_AD4351_Sig_Gen_V1.ino***

***File***

***Can we now use the ILI9341 display and keypad from the attached to produce a new MAX2870 with user input the same with keypad and target frequency display and "locked" "unlocked" graphics - as per the attached sketch. User can input target frequency in range and default 100Mhz reference frequency. Use the Nucleo-64 board F411RE as the target***

Thought for 8m 45s

Yes — I rebuilt it for the **NUCLEO-F411RE** using the same **ILI9341 + XPT2046 touchscreen keypad UI style** as your attached sketch. On the NUCLEO-F411RE Arduino header, the standard SPI pins are **D13 = SCK, D12 = MISO, D11 = MOSI**, so the display, touch controller, and MAX2870 all share that SPI bus cleanly. 

Because your attached UI sketch already uses **D9** for `TFT_CS` and **D10** for `TFT_RST`, I remapped the MAX2870 control pins to avoid the clash:

- `D5` → MAX2870 `LE`
- `D6` → MAX2870 `CE`
- `D4` → MAX2870 `LD`
- `D13` → MAX2870 `CLK`
- `D11` → MAX2870 `DATA`

I also kept the robust MAX2870 programming sequence in the new sketch: outputs off during setup, the required **double programming after power-up with a 20 ms pause**, plus a warm-retune sequence that re-latches buffered divider changes before enabling RF again. That matches the MAX2870 datasheet guidance. 

Download the sketch here:

max2870_touch_f411re.ino

A couple of practical notes:

- It defaults to **1200.0 MHz target** and **100.0 MHz reference**.
- The on-screen keypad lets you enter any target from **23.5 MHz to 6000.0 MHz**.
- The main screen shows the target frequency plus a **LOCKED / UNLOCKED** badge driven from the `LD` pin.
- I carried over the touch calibration constants from your attached sketch, so if your panel alignment is off, adjust `TS_MINX`, `TS_MAXX`, `TS_MINY`, and `TS_MAXY`.
- Install these libraries in Arduino IDE: `Adafruit_GFX`, `Adafruit_ILI9341`, and `XPT2046_Touchscreen`. The XPT2046 library is the standard one used for these low-cost SPI touch controllers. 

Paste the first compile error or the first thing that misbehaves on hardware, and I’ll patch this version directly.

***I can't get the download to work, please supply sketch as text***

I’m reading your attached UI sketch now so I can mirror its screen layout and keypad behavior closely, then I’ll paste the full MAX2870 version here as plain text.

Thought for 41s

Absolutely — here is the full sketch as plain text for the **NUCLEO-F411RE** with:

- **ILI9341 display**
- **XPT2046 touch keypad**
- **MAX2870 control**
- on-screen **target frequency**
- **LOCKED / UNLOCKED** status from the `LD` pin
- default **100 MHz reference**
- user entry of target frequency by touchscreen keypad

This version uses the same UI style as your attached sketch.

```
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
static const double USER_MAX_PFD_HZ = 25.0e6;

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
  r |= (7UL << 9);                          // CP current midpoint
  if (p.integerMode) r |= (1UL << 8);      // LDF
  r |= (1UL << 6);                          // PDP = positive
  if (counterReset) r |= (1UL << 3);       // RST
  r |= 2U;
  return r;
}

static uint32_t buildR3() {
  return 0x0000000BUL;
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
  SPI.transfer((uint8_t)((value >> 8) & 0xFFU));
  SPI.transfer((uint8_t)(value & 0xFFU));
  digitalWrite(MAX_LE, HIGH);
  delayMicroseconds(2);
  digitalWrite(MAX_LE, LOW);
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
  digitalWrite(MAX_CE, HIGH);

  uint32_t resetRegs[6];
  for (uint8_t i = 0; i < 6; ++i) resetRegs[i] = p.regOff[i];
  resetRegs[2] = buildR2(p, true);

  // Latch any buffered DIVA change with outputs disabled
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

  // Second pass with normal operation
  max2870Write(p.regOff[5]);
  max2870Write(p.regOff[4]);
  max2870Write(p.regOff[3]);
  max2870Write(p.regOff[2]);
  max2870Write(p.regOff[1]);
  max2870Write(p.regOff[0]);

  // Third pass improves warm retune reliability
  writeBlock(p.regOff);

  // Enable RF output
  max2870Write(p.regOn[4]);
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

  pinMode(MAX_LE, OUTPUT);    digitalWrite(MAX_LE, LOW);
  pinMode(MAX_CE, OUTPUT);    digitalWrite(MAX_CE, HIGH);
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
```

A couple of practical notes:

- This expects your display and touch wiring to match the earlier attached sketch.
- `MAX_LD` is used for the lock badge. If LD is not wired, it will likely stay `UNLOCKED`.
- The touch calibration values may need tweaking for your panel.
- The sketch uses a shared SPI bus for TFT, touch, and MAX2870.

The first thing I’d test is whether it compiles cleanly for **NUCLEO-F411RE**. If you get a compile error, paste it exactly and I’ll patch the sketch.

***Code compiles. Initialisation to 1200MHz does not work, however if I then set 1200MHz using touch entry it works and locks at target.***

I’ve got a likely culprit: during display/touch startup, the MAX2870 is already enabled and its **LE line is held low**, so shared SPI traffic can clock junk into the MAX2870 shift register before the first real programming pass. I’m checking that against the datasheet behavior and then I’ll give you the minimal startup fix.

Thought for 1m 55s

Yes — that behavior makes sense.

What’s happening is likely this: on startup, the **TFT and touch controller use the shared SPI bus before the MAX2870’s first real tune**, while your sketch currently has the MAX2870 already enabled and its **LE line idle-low**. The MAX2870 shifts serial data with **LE low** and latches on the **rising edge of LE**. The datasheet also says that after exiting low-power via **CE**, you should allow **at least 20 ms** before programming the final VCO frequency, and that after power-up the registers should be programmed **twice** with a **20 ms pause** because the first write enables the device and the second starts VCO selection. That matches your symptom: the **first automatic 1200 MHz tune fails**, but the later touch-entered 1200 MHz works once the display/touch init traffic is over and enough time has elapsed. 

The clean fix is:

- keep **CE low** while the TFT and touch initialize
- keep **LE high** while idle, so the shared SPI bus cannot accidentally clock data into the MAX2870
- after the display/touch are initialized, bring **CE high**
- wait **25 ms**
- then do the normal MAX2870 programming sequence. 

Use these replacements.

### 1) Replace `initPins()`

```
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
```

### 2) Replace `max2870Write()`

```
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
```

### 3) Replace the startup part of `setup()`

```
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
```

### 4) In `programMax2870()`, remove this line

```
digitalWrite(MAX_CE, HIGH);
```

because startup power-up is now handled in `setup()`.

------

That should fix the “startup 1200 fails, manual 1200 works” problem without changing your frequency-planning code.

The reason this is the right fix is that the MAX2870’s **SPI remains alive in low-power mode**, **register values are retained in low-power mode**, and the device needs the documented startup timing around power-up / exit from low-power. Holding **CE low** and **LE high** during TFT/touch init avoids bus contamination and gives the PLL a clean first programming pass. 

Paste back the result after this patch, especially whether **cold power-up to 1200 MHz** now locks first time.

***Init at 1200Mhz now working. If I enter 1000Mhz as target, output is 680MHz with NO LOCK. If I then enter 1200Mhz output is as expected at target frequency with LOCK. If I set 500Mhz target I get 340Mhz with NO LOCK. 1100MHz works as expected.***

I’ve narrowed it to the retune path rather than the frequency math. The pattern **1200 works, 1100 works, but 1000 and 500 both fail** points to the **4.0 GHz VCO point** not being cleanly reselected on a warm retune.

Thought for 4m 5s

That pattern is very informative.

For the MAX2870, `fVCO = fRFOUT × DIVA`, and the output-divider ranges are `/4` for **750 to <1500 MHz** and `/8` for **375 to <750 MHz**. So:

- **1000 MHz** uses **/4**, giving **4.0 GHz VCO**
- **500 MHz** uses **/8**, also giving **4.0 GHz VCO**
- **1100 MHz** uses **/4**, giving **4.4 GHz VCO**
- **1200 MHz** uses **/4**, giving **4.8 GHz VCO** 

Because **1000 MHz and 500 MHz both fail together**, while **1100 MHz and 1200 MHz work**, the math itself is very likely fine. The common factor is the **4.0 GHz VCO point**, which points to the **retune sequence not forcing a fresh VCO autoselection cleanly enough** on warm retunes. The datasheet explicitly says that after power-up the registers should be programmed **twice** with a **20 ms pause**, and that the **second write starts the VCO selection process**. It also says that in **low-power mode the register values are retained**, which is why a CE-driven restart is a useful way to force a clean retune. 

So the next correction I’d make is:

1. **Cycle CE low/high on every tune**, not just at startup.
2. After CE goes high, wait **25 ms**.
3. Then do the normal **double-write startup sequence** again.
4. Set **RETUNE = 1** in R3 so the VAS auto-retune logic is enabled. The datasheet says RETUNE enables automatic retune if the VCO control voltage drifts to the rail states. 

Use these code changes.

### 1) Change `buildR3()`

```
static uint32_t buildR3() {
  // RETUNE = 1, VAS enabled, clock divider off, CDIV = 1
  return 0x0100000BUL;
}
```

### 2) Replace `programMax2870()` with this

```
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
```

### 3) Keep `max2870Write()` with LE idling high

```
static void max2870Write(uint32_t value) {
  deselectSpiDevicesForMax();

  SPI.beginTransaction(max2870Spi);
  digitalWrite(MAX_LE, LOW);

  SPI.transfer((uint8_t)((value >> 24) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 16) & 0xFFU));
  SPI.transfer((uint8_t)((value >> 8)  & 0xFFU));
  SPI.transfer((uint8_t)( value        & 0xFFU));

  digitalWrite(MAX_LE, HIGH);
  delayMicroseconds(2);
  SPI.endTransaction();
}
```

This is the change I’d try before anything else, because it directly targets the behavior you’re seeing: some VCO targets retune cleanly, but the **4.0 GHz VCO point does not** unless the part gets a “fresh start.” That lines up with the datasheet’s wording that the second write after power-up starts VCO selection. 

One small but important detail: with your current planner, **500 MHz** and **1000 MHz** really are a good diagnostic pair, because they both demand the same **4.0 GHz VCO** and only differ in the output divider. So if both fail together again after this patch, the divider logic is probably not the culprit. 

Paste back what happens for **500, 1000, 1100, and 1200 MHz** after this CE-cycle retune change.

***At 500MHz I have output at 340MHz and NO LOCK, at 1000MHz I have output at 720MHz  and NO LOCK, at 1100MHz I have output at 1100MHz and LOCK, at 1200MHz I have output at 1200MHz and LOCK***

I’m comparing those four cases against the exact MAX2870 divider bands and register bit fields. The fact that 500 and 1000 both fail, while 1100 and 1200 both work, still points to one specific register-field transition rather than the keypad/UI code.

Thought for 6m 48s

That result isolates the problem nicely: **500 MHz and 1000 MHz both require the MAX2870 to lock its internal VCO at 4.0 GHz**, while **1100 MHz** and **1200 MHz** require **4.4 GHz** and **4.8 GHz** respectively. The divider bands in the datasheet are `/4` for **750 MHz to <1500 MHz** and `/8` for **375 MHz to <750 MHz**, so both 1000 MHz and 500 MHz land on the same 4.0 GHz VCO target. That makes this look much more like a **4.0 GHz lock-acquisition / loop-dynamics issue** than a keypad or divider-math bug. 

The MAX2870 datasheet also ties lock behavior directly to the **phase-detector frequency**, the **band-select clock** (`BS = fPFD / 50 kHz`), and the **charge-pump current** set by **R2[12:9]**. It explicitly notes that VCO selection starts when **R0** is programmed, that the lock indication is not valid until the VCO autoselect and tune settling complete, and that the loop behavior depends on the loop filter bandwidth. 

So the next change I’d make is to stop using the conservative **25 MHz PFD** and move to a **50 MHz PFD**, while also increasing the **charge-pump setting to 15**. That gives the VCO autoselect a faster band-select clock and matches the style of known working MAX2870 examples much more closely. The device allows up to **105 MHz PFD in integer-N** and **50 MHz in fractional-N**, so **50 MHz** is comfortably inside spec. 

Change these two parts.

```
constexpr double USER_MAX_PFD_HZ = 50.0e6;
```

and in `buildR2()` change the CP field from `7` to `15`:

```
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
```

With a **100 MHz** reference, the planner will then typically settle on a **50 MHz PFD**. That gives these clean integer-N points:

- **500 MHz** → VCO **4000 MHz**, divider **/8**, **N = 80**
- **1000 MHz** → VCO **4000 MHz**, divider **/4**, **N = 80**
- **1100 MHz** → VCO **4400 MHz**, divider **/4**, **N = 88**
- **1200 MHz** → VCO **4800 MHz**, divider **/4**, **N = 96** 

If **500** and **1000** still fail after that, the next step is to add **register-6 readback** of the active VCO and VTUNE ADC, because the datasheet exposes both specifically for diagnosing VCO selection and tuning-voltage compliance. 

Paste back the results after the **50 MHz PFD + CP=15** change for **500 / 1000 / 1100 / 1200 MHz**.

***All four tests now produce output and LOCK as expected :-)***

Excellent — that strongly suggests the issue was **loop / acquisition margin at the 4.0 GHz VCO point**, not the frequency-planning math.

What fixed it was:

- raising the planner ceiling to **50 MHz PFD**
- increasing **charge pump current** to **15**
- keeping the improved **startup / retune sequence**
- keeping **LE idle high** and delaying MAX2870 enable until after the TFT/touch SPI traffic

That’s consistent with the MAX2870 datasheet behavior around **VCO autoselect**, **band-select timing**, and loop dynamics. The 500 MHz and 1000 MHz cases both map to a **4.0 GHz VCO**, so it makes sense they failed together and recovered together.

For your “known good” settings, the important line is now:

```
constexpr double USER_MAX_PFD_HZ = 50.0e6;
```

and in `buildR2()`:

```
r |= (15UL << 9);   // CP current = maximum
```

I’d keep those unless you later redesign the loop filter and want to optimize phase noise or spur performance.

The next sensible cleanup would be to produce one final consolidated sketch with all successful fixes merged and a short header block listing the confirmed working wiring and settings for the **NUCLEO-F411RE + ILI9341 + XPT2046 + MAX2870** setup.

***produce one final consolidated sketch with all successful fixes merged and a short header block listing the confirmed working wiring and settings for the NUCLEO-F411RE + ILI9341 + XPT2046 + MAX2870 setup.***

Thought for 22s

Below is the final consolidated sketch for the **NUCLEO-F411RE + ILI9341 + XPT2046 + MAX2870** setup.

It keeps the fixes that proved to work in your testing:

- shared SPI bus for TFT, touch, and MAX2870
- **MAX2870 CE held low** during TFT/touch startup
- **MAX2870 LE idles high**
- full MAX2870 re-init on each retune
- **double programming sequence** with the required pause
- **50 MHz PFD ceiling**
- **charge pump current = 15**
- touchscreen keypad entry of target frequency
- on-screen **LOCKED / UNLOCKED** indication from the `LD` pin

Those startup and retune behaviors align with the MAX2870 datasheet guidance on power-up programming, VCO selection, double-buffered fields, and allowed PFD limits. The NUCLEO-F411RE Arduino SPI pins are the standard **D13 SCK / D12 MISO / D11 MOSI**.

```
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>

/*
  ================================================================
  MAX2870 Touchscreen Signal Generator
  Target board: NUCLEO-F411RE (STM32 Arduino core)

  Confirmed working wiring
  ------------------------
  Shared SPI bus:
    D13 -> SPI1_SCK  -> ILI9341 SCK, XPT2046 T_CLK, MAX2870 CLK
    D11 -> SPI1_MOSI -> ILI9341 MOSI, XPT2046 T_DIN, MAX2870 DATA
    D12 -> SPI1_MISO -> XPT2046 T_DO

  ILI9341 TFT:
    D8  -> TFT_DC
    D9  -> TFT_CS
    D10 -> TFT_RST

  XPT2046 Touch:
    D7  -> TOUCH_CS

  MAX2870:
    D5  -> LE
    D6  -> CE
    D4  -> LD   (digital lock detect)

  Confirmed working settings
  --------------------------
    Default target frequency  : 1200.0 MHz
    Reference frequency       : 100.0 MHz
    MAX PFD used by planner   : 50 MHz
    Charge pump setting       : 15
    LE idle state             : HIGH
    CE during TFT/touch init  : LOW
    Retune method             : full CE cycle + full reprogram
    Startup programming       : double-write sequence with 20 ms pause

  Libraries needed
  ----------------
    Adafruit_GFX
    Adafruit_ILI9341
    XPT2046_Touchscreen
  ================================================================
*/

// -------------------- Display / touch pins --------------------
static const uint8_t TFT_DC   = D8;
static const uint8_t TFT_CS   = D9;
static const uint8_t TFT_RST  = D10;
static const uint8_t TOUCH_CS = D7;

// -------------------- MAX2870 pins ----------------------------
static const uint8_t MAX_LE   = D5;
static const uint8_t MAX_CE   = D6;
static const uint8_t MAX_LD   = D4;

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

// Confirmed working planner ceiling
static const double USER_MAX_PFD_HZ = 50.0e6;

// MAX2870 limits
static const double VCO_MIN_HZ = 3000.0e6;
static const double VCO_MAX_HZ = 6000.0e6;
static const double MAX_PFD_INT_HZ  = 105.0e6;
static const double MAX_PFD_FRAC_HZ = 50.0e6;
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

// -------------------- Helpers ---------------------------------
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
  digitalWrite(MAX_LE, HIGH);    // MAX2870 ignores bus traffic while LE is high
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
      divider  = dividers[code];
      divaCode = code;
      vcoHz    = candidateVco;
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
        if (pfd > MAX_PFD_INT_HZ) continue;

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
        if (pfd > MAX_PFD_FRAC_HZ) continue;

        const double ratio = p.vcoHz / pfd;
        const double nFloor = floor(ratio);
        if (nFloor < 19.0 || nFloor > 4091.0) continue;

        uint16_t mod  = MOD_MAX;
        uint16_t frac = (uint16_t)llround((ratio - nFloor) * (double)mod);
        uint16_t n    = (uint16_t)nFloor;

        if (frac >= mod) {
          frac = 0;
          ++n;
        }
        if (n < 19 || n > 4091) continue;

        if (frac != 0) {
          const uint16_t g = gcd16(frac, mod);
          frac /= g;
          mod  /= g;
          if (mod < 2) mod = 2;
        }

        const double actualVco = ((double)n + ((double)frac / (double)mod)) * pfd;
        const double actualRf  = actualVco / (double)p.divider;
        const double err       = fabs(actualRf - p.targetHz);

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

// -------------------- Register builders -----------------------
static uint32_t buildR0(const Max2870Plan &p) {
  uint32_t r = 0;
  if (p.integerMode) r |= (1UL << 31);              // INT
  r |= ((uint32_t)p.n    & 0xFFFFUL) << 15;         // N
  r |= ((uint32_t)p.frac & 0x0FFFUL) << 3;          // FRAC
  r |= 0U;
  return r;
}

static uint32_t buildR1(const Max2870Plan &p) {
  uint32_t r = 0;
  if (p.integerMode) {
    r |= (1UL << 31);                               // CPOC = 1 in integer-N
  } else {
    r |= (1UL << 29);                               // CPL = 01 in fractional-N
  }
  r |= (1UL << 15);                                 // P = 1
  r |= ((uint32_t)p.mod & 0x0FFFUL) << 3;          // MOD
  r |= 1U;
  return r;
}

static uint32_t buildR2(const Max2870Plan &p, bool counterReset) {
  uint32_t r = 0;

  if (p.pfdHz > 32.0e6) r |= (1UL << 31);          // LDS
  if (p.dbr)   r |= (1UL << 25);
  if (p.rdiv2) r |= (1UL << 24);

  r |= ((uint32_t)p.r & 0x03FFUL) << 14;           // R
  r |= (1UL << 13);                                // REG4DB = 1
  r |= (15UL << 9);                                // CP current = maximum
  if (p.integerMode) r |= (1UL << 8);              // LDF
  r |= (1UL << 6);                                 // PDP = positive
  if (counterReset) r |= (1UL << 3);               // RST
  r |= 2U;
  return r;
}

static uint32_t buildR3() {
  // RETUNE = 1, VAS enabled, CDIV default
  return 0x0100000BUL;
}

static uint32_t buildR4(const Max2870Plan &p, bool enableA, bool enableB) {
  uint32_t r = 0;

  r |= (0b011000UL << 26);                         // reserved bits
  r |= ((uint32_t)((p.bs >> 8) & 0x03U)) << 24;   // BS[9:8]
  r |= (1UL << 23);                                // FB = 1
  r |= ((uint32_t)p.divaCode & 0x07U) << 20;      // DIVA
  r |= ((uint32_t)(p.bs & 0xFFU)) << 12;          // BS[7:0]

  if (enableB) r |= (1UL << 8);
  r |= (0UL << 6);                                 // RFOUTB power

  if (enableA) r |= (1UL << 5);
  r |= (3UL << 3);                                 // RFOUTA power max

  r |= 4U;
  return r;
}

static uint32_t buildR5(const Max2870Plan &p) {
  uint32_t r = 0x00400005UL;                       // LD = digital lock detect
  if (p.integerMode || p.frac == 0) r |= (1UL << 24);  // F01
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
  p.refHz    = refMHz * 1.0e6;

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

  digitalWrite(MAX_LE, HIGH);   // latch word
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
  // Force clean restart on each retune
  digitalWrite(MAX_CE, LOW);
  digitalWrite(MAX_LE, HIGH);
  delay(5);

  digitalWrite(MAX_CE, HIGH);
  delay(25);    // allow >=20 ms after exit from low-power mode

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

  // Second pass normal
  max2870Write(p.regOff[5]);
  max2870Write(p.regOff[4]);
  max2870Write(p.regOff[3]);
  max2870Write(p.regOff[2]);
  max2870Write(p.regOff[1]);
  max2870Write(p.regOff[0]);

  // Third pass improves warm-retune reliability
  writeBlock(p.regOff);

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
  if (x >= 3 * CELL_W && x < 4 * CELL_W &&
      y >= (KEYPAD_Y + 1 * CELL_H) && y < (KEYPAD_Y + 4 * CELL_H)) {
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
  return (x >= BTN_SET_X && x < (BTN_SET_X + BTN_SET_W) &&
          y >= BTN_SET_Y && y < (BTN_SET_Y + BTN_SET_H));
}

// -------------------- Setup helpers ---------------------------
static void initPins() {
  pinMode(TFT_CS, OUTPUT);    digitalWrite(TFT_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT);  digitalWrite(TOUCH_CS, HIGH);
  pinMode(TFT_DC, OUTPUT);    digitalWrite(TFT_DC, HIGH);
  pinMode(TFT_RST, OUTPUT);   digitalWrite(TFT_RST, HIGH);

  // Keep MAX2870 asleep and immune to shared SPI traffic during display init
  pinMode(MAX_LE, OUTPUT);    digitalWrite(MAX_LE, HIGH);
  pinMode(MAX_CE, OUTPUT);    digitalWrite(MAX_CE, LOW);
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

  // Wake MAX2870 only after the other SPI devices are initialized
  digitalWrite(MAX_CE, HIGH);
  digitalWrite(MAX_LE, HIGH);
  delay(25);

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
```

The two “known good” lines you identified are already folded in:

```
static const double USER_MAX_PFD_HZ = 50.0e6;
r |= (15UL << 9);  // CP current = maximum
```

If you want, I can also produce a second version with a small status line showing **actual MHz, PFD, divider, and N/FRAC/MOD** on the TFT for debugging.

***Acting as a technical author, produce an article for publication that describes this full chat from start to finish and explains why this is a good example of hobbyist vibe coding. Note that the keypad and display functionality was also created by a previous chat in my history. Include details on the MAX2870, its basic capabilities and its manufacturer. Explain why this makes an excellent hobyist signal generator and include brief options for adding post MAX2870 broadband amplifier and SPI controlled attenuator module for output control.***  

I’m pulling the MAX2870 and NUCLEO-F411RE facts from primary sources, and I’m also checking current amplifier/attenuator options so the publication piece doesn’t rely on stale part suggestions.

Thought for 2m 34s

# From Prompt to PLL: How a Chat Session Became a Working MAX2870 Touchscreen Signal Generator

There is a particular kind of engineering session that feels unmistakably modern. It does not start with a polished specification or a finished block diagram. It starts with a hunch, a bench, a half-remembered earlier project, and a conversation. That is exactly what happened here.

The end result was a working touchscreen signal generator built around a **MAX2870** wideband PLL/VCO and a **NUCLEO-F411RE**, with an **ILI9341 display**, an **XPT2046 touch keypad**, on-screen frequency entry, and live **LOCKED / UNLOCKED** status from the synthesizer’s digital lock-detect pin. The path to that result was anything but linear, which is precisely why it makes such a good example of hobbyist “vibe coding.”

## The part at the center of the project

The **MAX2870**, manufactured by **Analog Devices**, is an ultra-wideband **fractional-N / integer-N PLL with integrated VCOs**. Its specified RF output range is **23.5 MHz to 6.0 GHz**, it is controlled over a **4-wire serial interface**, and it is intended for applications including test and measurement, wireless infrastructure, satellite communications, and clock generation. In practical terms, that means one chip can cover an enormous span of frequencies with relatively modest external support circuitry: a reference source, loop filter, power, and a controller. 

That combination is exactly why the MAX2870 is so attractive for hobbyists. One device can produce everything from low VHF through L-band and well into microwave territory, and it does so with a digitally programmable architecture that is friendly to microcontrollers. The official MAX2870 evaluation kit even leans into bench-style use with 50-ohm SMA connectivity, which tells you a lot about the part’s intended workflow. 

## How the session began

The original request was straightforward enough: act as an Arduino coding expert and produce a sketch to calculate MAX2870 register values from a target and reference frequency, with **1200 MHz** as the default target and **100 MHz** as the default reference. The first twist came immediately: the target microcontroller kept changing. The board moved from an F432 concept, to F412, to F303, before the project later settled on the **NUCLEO-F411RE**.

That shifting target is not a bug in the process. It is the process. Hobby projects often evolve in response to what is actually on the bench, what headers line up with existing modules, and which board is physically available that afternoon.

The early part of the conversation focused on the essentials: calculating the MAX2870’s **output divider**, **PFD frequency**, **integer or fractional settings**, and producing the six programming registers. A baseline SPI programming sketch was written and tested. It compiled. It even worked at some frequencies. That is usually when the real engineering begins.

## The first clues: frequencies that were “almost right”

The first hardware tests did not fail randomly. They failed in patterns.

A requested **100 MHz** output produced something near **192 MHz**. A requested **110 MHz** produced **200 MHz**. Later, **120 MHz** worked correctly, while **100 MHz** worked only if the MAX2870 was power-cycled before reprogramming. Those are the kinds of symptoms that matter because they are structured. They tell you the math is probably not universally wrong; instead, something about startup, state retention, or register latching is off.

That pushed the debugging away from “rewrite the formulas” and toward the MAX2870 datasheet. The crucial details were there. The datasheet explicitly states that **after power-up the registers should be programmed twice with at least a 20 ms pause**, because the first write enables the device and the second starts the VCO selection process. It also notes that register contents are retained in low-power mode, and that some fields are double-buffered, including the **output divider field in R4**, which is transferred when **R0** is written if buffering is enabled. 

That changed the direction of the code. The problem was no longer treated as a simple register calculator. It became a device-initialization and retune-sequencing problem.

## From a calculator to a robust driver

Once the conversation absorbed those datasheet details, the code changed character. The sketch stopped being a one-shot programmer and became a more careful MAX2870 driver.

The revised versions added the double-write startup sequence, retune handling that explicitly rewrote the full register set, and logic to respect the part’s buffered fields. That was followed by a second class of bugs: board-level realities.

On STM32, a naming collision appeared because the sketch used `PIN_SPI_MOSI` and `PIN_SPI_SCK`, which were already defined by the STM32 Arduino core. That was not a radio problem at all; it was a platform integration problem. Renaming those symbols fixed compilation. Then the physical wiring was corrected to the board’s actual hardware SPI pins.

By the time the session reached the **NUCLEO-F411RE**, the hardware foundation was much better defined. On that board’s Arduino-compatible headers, **D13 is SPI1_SCK**, **D12 is SPI1_MISO**, and **D11 is SPI1_MOSI**, making it a very natural host for a shared-SPI design. 

## The UI did not come out of nowhere

An important part of the finished instrument did not originate in this session at all. The **display and keypad behavior had already been created in an earlier chat in the user’s history**, and that earlier work arrived here as an attached sketch. That is another reason this is a strong example of hobbyist vibe coding: it was not “AI writes everything from zero.” It was compositional.

The touchscreen keypad UI became a reusable building block. In this chat, that earlier display work was adapted to a different RF engine: instead of merely showing values or supporting another synthesizer, it was used to drive the MAX2870. The conversation had to reconcile SPI bus sharing between three devices: the **ILI9341 TFT**, the **XPT2046 touch controller**, and the **MAX2870** itself.

That created one of the more interesting problems in the whole session. At startup, the display and touch controller were using the SPI bus before the MAX2870 had been cleanly initialized. The fix turned out to be elegantly physical: keep the MAX2870 **disabled with CE low** while the TFT and touch controller start up, and keep the MAX2870’s **LE line high** so it ignores shared SPI traffic. Only after the UI hardware is initialized does the code wake the synthesizer and run the official programming sequence. That is exactly the sort of detail that makes shared-bus embedded systems either feel solid or mysteriously flaky.

## The last bug was the best one

The most revealing late-stage symptom was this: **1100 MHz** and **1200 MHz** worked, but **1000 MHz** and **500 MHz** did not. Those failures were not random. Both 1000 MHz and 500 MHz map to the **same 4.0 GHz internal VCO operating point**, just with different output-divider settings. Meanwhile, 1100 MHz and 1200 MHz map to **4.4 GHz** and **4.8 GHz** respectively. According to the MAX2870’s divider structure, 1000 MHz uses the **/4** output path and 500 MHz uses **/8**, but both still require a 4.0 GHz VCO lock. 

That insight led to the successful fix: raising the planner’s working ceiling to a **50 MHz PFD** and increasing the MAX2870’s **charge-pump current setting to 15**. The MAX2870 supports up to **105 MHz PFD in integer-N mode** and **50 MHz in fractional-N mode**, so 50 MHz was comfortably inside the official limits. Once those settings were adopted, the previously bad cases locked and produced the expected outputs. 

That moment is a perfect snapshot of what good hobbyist vibe coding looks like. The AI did not “magically know” the answer. The user supplied measured behavior. The conversation noticed a pattern. The pattern was mapped back to the device architecture. Then a small, targeted parameter change fixed the system.

## Why this is a good example of hobbyist vibe coding

The phrase “vibe coding” can sound dismissive when it is used to mean careless prompting with no engineering discipline behind it. But this session demonstrates the better version of the idea.

First, it was **iterative**. The first sketch was not sacred. It was tested, measured, challenged, and revised.

Second, it was **bench-driven**. Observations like “100 MHz gives 192 MHz,” “120 MHz works,” or “1000 MHz gives 720 MHz and no lock” were not noise; they were the primary debugging data.

Third, it was **grounded in primary sources**. The turning points in the session came from the MAX2870 datasheet: the double-write power-up requirement, buffered register behavior, PFD limits, and retune-related details. 

Fourth, it was **compositional**. The touchscreen keypad and display flow came from a previous chat, while the synthesizer control logic was built here. The finished system is the result of joining earlier generated subsystems into a more capable instrument.

Finally, it was **human-directed**. The user kept steering the work: changing target boards, correcting wiring, reporting exact measurements, and validating each fix on real hardware. The AI accelerated the process, but the engineering remained collaborative.

That is hobbyist vibe coding at its best: not blind faith in generated code, but a tight loop between idea, prompt, hardware, measurement, and revision.

## Why the MAX2870 makes an excellent hobbyist signal generator

The MAX2870 sits in a sweet spot for experimental RF work. Because it integrates the VCO and covers **23.5 MHz to 6 GHz**, one design can serve as a general-purpose source for mixers, receiver testing, filter experiments, LO generation, and broad bench work. The 4-wire digital interface makes it very easy to pair with a small MCU, and the frequency-planning model is straightforward enough to expose directly to a touchscreen UI. 

For a hobbyist, that means fewer moving parts than older architectures that required an external VCO, more usable range than many entry-level synthesizer chips, and enough flexibility to support both simple fixed-frequency use and more ambitious projects such as sweep generators or scripted test sources.

The NUCLEO-F411RE is also a good match. It provides standard Arduino-style header access, hardware SPI on the familiar D11/D12/D13 pins, and enough I/O to manage the TFT, touch controller, and MAX2870 cleanly. 

## Practical output-stage upgrades

A raw synthesizer output is useful, but a more polished signal generator usually benefits from some output conditioning.

For a **broadband post-MAX2870 amplifier**, two sensible IC-level options are the **Analog Devices ADL5542**, a broadband 20 dB linear amplifier specified up to **6 GHz**, and the **Qorvo RF3374**, a 50-ohm gain-block-style amplifier intended for use in bands up to **6 GHz**. If the generator will mostly be used below 4 GHz, the **Qorvo SPF5189Z** is another easy-to-implement broadband option specified from **50 MHz to 4 GHz**. Any of these can sit after the MAX2870, provided the layout, supply decoupling, and output filtering are handled properly. 

For **digital output-level control**, a very attractive part is the **pSemi PE43711**, a **7-bit digital step attenuator** that supports **9 kHz to 6 GHz** and offers a serial programming interface. Another option, if your intended coverage is concentrated below 3.5 GHz, is the **Qorvo TQP4M9083**, a **7-bit SPI digital step attenuator** for **0.4 GHz to 3.5 GHz**. In other words, a hobbyist-friendly output chain can be as simple as **MAX2870 → broadband amplifier → digital step attenuator → output connector**, with the MCU controlling both the synthesizer and the attenuator over digital interfaces. 

## The real lesson

What makes this chat worth publishing is not that it ended with a working sketch. Plenty of sessions end with working code. What makes it interesting is that the finished instrument emerged from the kind of nonlinear, hardware-led conversation that hobby electronics actually thrives on.

A board changed. Then another board changed. A UI from an earlier chat was reused. The first RF code worked only partly. Startup sequencing mattered. Shared SPI bus behavior mattered. VCO acquisition at one internal frequency mattered. The solution was not one big flash of brilliance. It was a chain of small, testable corrections.

That is why this project is such a strong example of hobbyist vibe coding. It captures the real texture of building things now: part bench work, part datasheet reading, part software generation, part debugging by pattern recognition, and part creative reuse of previous work. The result is not just code. It is a usable instrument, and a record of how modern making actually happens.

***I've put the project in a case and had to change the screen rotation, I now have   tft.setRotation(3); Recommend changes needed in the code to accommodate this screen rotation for the touch functionality***

Thought for 38s

Yes. Going from:

```
tft.setRotation(1);
```

to:

```
tft.setRotation(3);
```

is a **180° change in landscape orientation**, so the display layout can stay the same, but the **touch coordinate mapping needs to be flipped**.

In your current code, the touch mapping is:

```
sx = map(p.x, TS_MINX, TS_MAXX, 319, 0);
sy = map(p.y, TS_MINY, TS_MAXY, 239, 0);
```

For `setRotation(3)`, the first thing to try is changing it to:

```
sx = map(p.x, TS_MINX, TS_MAXX, 0, 319);
sy = map(p.y, TS_MINY, TS_MAXY, 0, 239);
```

So replace your `getTouchXY()` with this:

```
static bool getTouchXY(int16_t &sx, int16_t &sy) {
  deselectSpiDevicesForTouch();

  if (!ts.touched()) return false;
  TS_Point p = ts.getPoint();

  if (p.z < 300 || p.z > 4000) return false;
  if (p.x == 0 && p.y == 0) return false;

  // For tft.setRotation(3)
  sx = map(p.x, TS_MINX, TS_MAXX, 0, 319);
  sy = map(p.y, TS_MINY, TS_MAXY, 0, 239);

  sx = constrain(sx, 0, 319);
  sy = constrain(sy, 0, 239);
  return true;
}
```

That is the most likely correct change because `rotation(1)` and `rotation(3)` are opposite landscape orientations.

A slightly better version is to make it rotation-aware:

```
static bool getTouchXY(int16_t &sx, int16_t &sy) {
  deselectSpiDevicesForTouch();

  if (!ts.touched()) return false;
  TS_Point p = ts.getPoint();

  if (p.z < 300 || p.z > 4000) return false;
  if (p.x == 0 && p.y == 0) return false;

  switch (tft.getRotation()) {
    case 1:
      sx = map(p.x, TS_MINX, TS_MAXX, 319, 0);
      sy = map(p.y, TS_MINY, TS_MAXY, 239, 0);
      break;

    case 3:
      sx = map(p.x, TS_MINX, TS_MAXX, 0, 319);
      sy = map(p.y, TS_MINY, TS_MAXY, 0, 239);
      break;

    default:
      sx = map(p.x, TS_MINX, TS_MAXX, 0, 319);
      sy = map(p.y, TS_MINY, TS_MAXY, 0, 239);
      break;
  }

  sx = constrain(sx, 0, 319);
  sy = constrain(sy, 0, 239);
  return true;
}
```

A couple of points:

- Your **button hit-testing and screen layout do not need changing** because `rotation(3)` is still 320×240 landscape.
- Your existing calibration values `TS_MINX`, `TS_MAXX`, `TS_MINY`, `TS_MAXY` will probably still be fine.
- If touches end up on the wrong button after this change, the next thing to try is a **swap of X and Y**, but for a straight change from rotation 1 to 3, that usually is **not** needed.

If you test this and tell me whether the touch is now mirrored left-right, up-down, or diagonal, I can give you the exact final mapping in one pass.