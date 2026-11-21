// =====================================================================
//  myFunctions.ino  --  THE IMPLEMENTATION
//  EVENT HORIZON PROPULSION LABS  ::  Vehicle Pre-Flight Checkout
//
//  Everything the checklist calls for is defined here, and nothing
//  here decides anything. The limits live in preflight.ino; the names
//  live in preflight.h; the work lives in this file.
//
//  You are not asked to write any of this. You are asked to be able to
//  find any of it. Every test below is code you wrote yourself in an
//  earlier lab, wrapped in a question with a yes or no answer.
//
//  Reading order, if you only read one: testBuzzer(). Forty seconds.
//  Every other test has that same four-step shape.
// =====================================================================

// =====================================================================
//  DATA  --  the definitions the header promised
// =====================================================================

// ------------------------------------------------------------ libraries
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <PCA95x5.h>
#include <Adafruit_NeoPixel.h>
#include <Anitracks_ADS7142.h>
#include <Arduino_LSM6DSOX.h>
#include <QMC5883LCompass.h>
#include <VL6180X.h>
#include <Servo.h>
#include <WiFiNINA.h>

#include "preflight.h"

// ---------------------------------------------------------------------
//  1.  VEHICLE UNDER TEST
// ---------------------------------------------------------------------
#define VEHICLE_ID       "EH-007"     // put your board number here

// ---------------------------------------------------------------------
//  2.  ACCEPTANCE LIMITS
//
//  These are the numbers that decide PASS and FAIL. They are policy,
//  not code: the Software Group wrote the tests, but the limits belong
//  to the manager who signs the certificate. Loosen one and marginal
//  hardware flies. Tighten one and good hardware gets scrapped. Every
//  number here is a decision you must be able to defend.
// ---------------------------------------------------------------------
#define POT_SPAN_PCT      80     // % of full scale the throttle must sweep
#define TEMP_MIN_C        15.0   // coldest plausible cabin reading, C
#define TEMP_MAX_C        35.0   // hottest plausible cabin reading, C
#define TEMP_RISE_C       1.0    // rise required when the sensor is pinched
#define LIGHT_DROP_PCT    70     // covered reading must fall below this % of
                                 // the open reading. Tuned for a normally lit
                                 // room, not direct sunlight: tighten it if
                                 // your bench is bright.
#define LIGHT_MIN_OPEN    3000   // below this the room is too dark to judge
#define IMU_TILT_DEG      15.0   // tilt that must register on each side
#define COMPASS_TURN_DEG  90     // azimuth change required on a half turn
#define RANGE_TARGET_MM   100    // where the operator holds their hand
#define RANGE_TOL_MM      20     // how far off that reading may be
#define WIFI_RSSI_MIN    -80     // weakest network the radio must hear, dBm

// ---- operator interaction timing ----------------------------------
//  Raise these if the checkout feels rushed at the bench.
#define START_TIMEOUT     20000  // ms to press RIGHT and begin a test
#define ANSWER_TIMEOUT     8000  // ms to answer YES before it counts as NO
#define BUTTON_TIMEOUT    10000  // ms to find and press one named button
#define WATCH_TIME         4000  // ms each thing you must look at stays on
#define SWEEP_TIME        10000  // ms to sweep the pot / pinch the sensor
#define AIM_TIME           6000  // ms of live readout to aim the rangefinder

// -------------------------------------------------------------- hardware
Adafruit_ST7789   tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);
PCA9535           muxU31;        // buttons and enable lines   (0x20)
PCA9535           ledmux;        // 7-segment display          (0x21)
ADS7142           adcTempPot(ADDR_TEMPPOT);
ADS7142           adcLight(ADDR_LIGHT);
QMC5883LCompass   compass;
VL6180X           rangefinder;
Servo             mast;

// Segment patterns and digit positions, exactly as built in Lab 2.
uint16_t SEG[] = {
  0x3F00, 0x0600, 0x5B00, 0x4F00, 0x6600,   // 0 1 2 3 4
  0x6D00, 0x7D00, 0x0700, 0x7F00, 0x6F00    // 5 6 7 8 9
};
uint16_t POS[] = { 0x0007, 0x000B, 0x000D, 0x000E };   // [0] = rightmost

// The devices this checkout expects to find. Adding a row is the only
// change needed to put a new device on the roll call: busCount counts
// itself. The required flag is a design decision, not a fact about the
// hardware -- see the Part 0 checkpoint.
Device bus[] = {
  { ADDR_MUX,     "GPIO expander", true  },   // buttons and enable lines
  { ADDR_LEDMUX,  "7-seg display", true  },
  { ADDR_TEMPPOT, "ADC1 pot/temp", true  },
  { ADDR_LIGHT,   "ADC2 light",    true  },
  { ADDR_COMPASS, "Compass",       false },
  { ADDR_RANGE,   "Rangefinder",   false },
  { ADDR_IMU,     "IMU",           false },
};
int  busCount = sizeof(bus) / sizeof(bus[0]);
bool busOK[16];

TestResult results[20];
int        testCount = 0;

// =====================================================================
//  STARTUP AND SHUTDOWN
// =====================================================================

// Bring every bus and every chip to a known state, then take the roll
// call. Nothing has been tested when this returns -- the vehicle has
// only been asked who is aboard.
void beginCheckout() {
  Serial.begin(115200);
  delay(3000);                      // give the serial monitor time to attach

  Wire.begin();

  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  pinMode(RIGHT_BUTTON, INPUT);
  pinMode(BUZZ_PIN, OUTPUT);

  // U31: buttons in, two control lines out (0xFCFF, from Lab 3)
  muxU31.attach(Wire, ADDR_MUX);
  muxU31.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
  muxU31.direction(0xFCFF);

  // P11 powers the rangefinder. Without this line the VL6180X never
  // answers the roll call, and the checkout would report a healthy
  // sensor as missing. A checkout that lies is worse than no checkout.
  muxU31.write(PCA95x5::Port::P11, PCA95x5::Level::H);
  delay(100);

  // U32: the 7-segment display (0x21, direction 0x0010, from Lab 2)
  ledmux.attach(Wire, ADDR_LEDMUX);
  ledmux.polarity(PCA95x5::Polarity::ORIGINAL_ALL);
  ledmux.direction(0x0010);
  ledmux.write(0x000F);             // blank the display

  testCount = 0;

  Serial.println();
  Serial.println("=====================================================");
  Serial.println(" EVENT HORIZON PROPULSION LABS");
  Serial.println(" VEHICLE PRE-FLIGHT CHECKOUT   rev 1.0");
  Serial.print  (" VEHICLE ID: ");  Serial.println(VEHICLE_ID);
  Serial.println("=====================================================");

  scanBus();
  showRollCall();
  addResult("I2C BUS", requiredBusOK());
}

// Draw and print the health report, then stop. The vehicle is either
// certified or it is not; there is no third answer and no partial credit.
void endCheckout() {
  showReport();
}

// =====================================================================
//  ROLL CALL  --  who is on the bus?
// =====================================================================

// beginTransmission() + endTransmission() is a knock on the door.
// endTransmission() returns 0 when somebody answers (ACK) and nonzero
// when nobody does (NACK). That is the entire I2C scan, in two lines.
void scanBus() {
  Serial.println();
  Serial.println("--- ROLL CALL (I2C) ---");
  for (int i = 0; i < busCount; i++) {
    Wire.beginTransmission(bus[i].addr);
    busOK[i] = (Wire.endTransmission() == 0);

    Serial.print("  0x");
    if (bus[i].addr < 0x10) Serial.print("0");
    Serial.print(bus[i].addr, HEX);
    Serial.print("  ");
    Serial.print(bus[i].name);
    for (int s = strlen(bus[i].name); s < 16; s++) Serial.print(" ");
    Serial.print(busOK[i] ? "FOUND" : "MISSING");
    if (!busOK[i]) Serial.print(bus[i].required ? "   *** REQUIRED ***" : "   (optional)");
    Serial.println();
  }
}

// A missing optional device is information. A missing required one is
// a fault: nothing downstream of it can be trusted.
bool requiredBusOK() {
  for (int i = 0; i < busCount; i++) {
    if (bus[i].required && !busOK[i]) return false;
  }
  return true;
}

// Did this address answer the roll call? Tests call this before they
// touch a device, so that one absent chip produces one honest failure
// instead of five confusing ones. That is the "I" in FDIR: isolation.
bool present(byte addr) {
  for (int i = 0; i < busCount; i++) {
    if (bus[i].addr == addr) return busOK[i];
  }
  return false;
}

void showRollCall() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(4, 4);
  tft.setTextColor(ST77XX_CYAN);
  tft.println("ROLL CALL");

  for (int i = 0; i < busCount; i++) {
    tft.setCursor(4, 18 + i * 12);
    tft.setTextColor(busOK[i] ? ST77XX_GREEN
                              : (bus[i].required ? ST77XX_RED : ST77XX_YELLOW));
    tft.print(busOK[i] ? "FOUND   " : "MISSING ");
    tft.println(bus[i].name);
  }
  delay(3000);
}

// OPTIONAL (+3). Knock on every address the I2C standard allows, not
// just the ones this checkout expects. Your board carries devices this
// course never used.
void sweepBus() {
  Serial.println();
  Serial.println("--- FULL BUS SWEEP 0x08 - 0x77 ---");
  int found = 0;
  for (byte a = 0x08; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  device at 0x");
      if (a < 0x10) Serial.print("0");
      Serial.println(a, HEX);
      found++;
    }
  }
  Serial.print("  ");
  Serial.print(found);
  Serial.println(" device(s) answered.");
}

// =====================================================================
//  REPORTING
// =====================================================================

// Record one line of the health report. Note the order of events: the
// test has already finished running by the time this is called, because
// C++ evaluates testBuzzer() before it can pass the answer in.
void addResult(const char* name, bool passed) {
  if (testCount < 20) {
    results[testCount].name   = name;
    results[testCount].passed = passed;
    testCount++;
  }
  Serial.print(passed ? "[ PASS ] " : "[ FAIL ] ");
  Serial.println(name);
}

void showReport() {
  bool allGood = true;
  for (int i = 0; i < testCount; i++) {
    if (!results[i].passed) allGood = false;
  }

  // ---- the TFT annunciator panel: two columns, seven rows ----
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(4, 3);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("HEALTH REPORT   ");
  tft.print(VEHICLE_ID);

  for (int i = 0; i < testCount; i++) {
    int col = i / 7;                      // 0 = left, 1 = right
    int row = i % 7;
    tft.setCursor(4 + col * 120, 17 + row * 11);
    tft.setTextColor(results[i].passed ? ST77XX_GREEN : ST77XX_RED);
    tft.print(results[i].passed ? "OK   " : "FAIL ");
    tft.print(results[i].name);
  }

  tft.setCursor(4, 104);
  tft.setTextSize(3);
  tft.setTextColor(allGood ? ST77XX_GREEN : ST77XX_RED);
  tft.print(allGood ? "CERTIFIED" : "GROUNDED");

  // ---- the same report, down the serial link ----
  int failures = 0;
  Serial.println();
  Serial.println("=========== PRE-FLIGHT HEALTH REPORT ===========");
  Serial.print  (" VEHICLE ID: ");  Serial.println(VEHICLE_ID);
  for (int i = 0; i < testCount; i++) {
    Serial.print(results[i].passed ? " [ PASS ]  " : " [ FAIL ]  ");
    Serial.println(results[i].name);
    if (!results[i].passed) failures++;
  }
  Serial.print(" ");
  Serial.print(testCount - failures);
  Serial.print(" of ");
  Serial.print(testCount);
  Serial.println(" checks passed.");
  Serial.println(allGood ? " STATUS: CERTIFIED" : " STATUS: GROUNDED");
  Serial.println("===============================================");
}

// =====================================================================
//  OPERATOR HELPERS
//
//  A checkout is a conversation with a human, and every conversation in
//  this tool follows the same four beats:
//
//      1. the screen says what is about to happen
//      2. you press RIGHT when you are ready       <- nothing moves first
//      3. the thing happens, with a countdown on screen
//      4. RIGHT means yes.  Waiting means no.
//
//  Beat 2 is the important one. Nothing on this vehicle starts until an
//  inspector says go, so you are never reading an instruction about an
//  event that already happened.
// =====================================================================

// Screen geometry, 240 x 135 in this rotation. Text size 2 is 12 px per
// character, so roughly 18 characters fit on a line.
#define SCR_W        240
#define SCR_H        135
#define ROW_TITLE      6
#define ROW_LINE1     40
#define ROW_LINE2     64
#define ROW_FOOTER   104

static int lastSec = -1;         // so the countdown only redraws on change

// ---------------------------------------------------------------------
//  Draw a standard three-part screen: title, two instruction lines.
//  Everything drawn on the TFT is also echoed to the serial log, so the
//  log alone tells you what the operator was looking at.
// ---------------------------------------------------------------------
static void screen(const char* title, const char* l1, const char* l2,
                   uint16_t titleColor) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);

  tft.setTextColor(titleColor);
  tft.setCursor(6, ROW_TITLE);   tft.print(title);

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(6, ROW_LINE1);   tft.print(l1);
  tft.setCursor(6, ROW_LINE2);   tft.print(l2);

  Serial.print("   SCREEN: ");
  Serial.print(title);
  if (strlen(l1)) { Serial.print(" | "); Serial.print(l1); }
  if (strlen(l2)) { Serial.print(" | "); Serial.print(l2); }
  Serial.println();
}

// Bottom line: what the operator is supposed to press. Keep the text to
// 15 characters or fewer -- at size 2 that is 180 px, and the countdown
// digit starts at 194.
static void footer(const char* text, uint16_t color) {
  tft.fillRect(0, ROW_FOOTER - 4, SCR_W, SCR_H - ROW_FOOTER + 4, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(color);
  tft.setCursor(6, ROW_FOOTER);
  tft.print(text);
}

// ---------------------------------------------------------------------
//  reviveDisplay -- bring the TFT back after the LED ring has run.
//
//  Look at the header: TFT_RST is 17 and NEO_PIN is 17. They are the
//  same physical trace. Every bit the NeoPixel driver clocks out is also
//  a pulse on the display's reset line, so the moment the ring lights
//  up, the ST7789 resets and every prompt after it is drawn into a dead
//  display. The operator sees a black screen and the checkout appears to
//  hang, when in fact it is running perfectly and talking to nobody.
//
//  This cannot be fixed in software. Telling the display library there
//  is no reset pin would stop IT from touching the line, but the ring
//  would still yank it. Two subsystems were given one wire, and the only
//  honest response is to put the display back together afterwards.
//
//  Write this one up. A shared resource that works fine until two
//  subsystems are used in the same session is exactly the class of fault
//  a checkout exists to find, and it is invisible to any test that
//  exercises one subsystem at a time.
// ---------------------------------------------------------------------
static void reviveDisplay() {
  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  Serial.println("         display re-initialised (NEO_PIN shares TFT_RST)");
}

// Big seconds-remaining digit at the bottom right. A screen that is
// waiting must look like it is waiting; a frozen-looking screen makes
// an operator press things at random.
static void countdown(unsigned long start, unsigned long total) {
  long left = ((long)total - (long)(millis() - start) + 999) / 1000;
  if (left < 0) left = 0;
  if (left == lastSec) return;
  lastSec = left;

  tft.fillRect(SCR_W - 46, ROW_FOOTER - 6, 46, 34, ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(left <= 3 ? ST77XX_RED : ST77XX_YELLOW);
  tft.setCursor(SCR_W - 40, ROW_FOOTER - 2);
  tft.print(left);
}

// Live numeric readout on line 2, for tests where the operator needs to
// see the sensor moving while they move it.
static void showValue(const char* label, long v) {
  tft.fillRect(0, ROW_LINE2 - 2, SCR_W, 26, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(6, ROW_LINE2);
  tft.print(label);
  tft.print(v);
}

// --------------------------------------------------------- button reads
static bool rightDown() { return !digitalRead(RIGHT_BUTTON); }   // active-LOW

static bool maskDown(uint16_t mask, bool activeHigh) {
  if (mask == MASK_SW1) return rightDown();
  uint16_t v = muxU31.read();
  return activeHigh ? (v & mask) : !(v & mask);
}

// Wait until the button is let go, then settle.
//
// This is the fix for the fault that makes a checkout feel broken: a
// held button satisfies the next wait as well as this one, and the tool
// runs two prompts ahead of the operator. Debouncing on the way down is
// not enough. You have to wait for the release.
static void waitRelease(uint16_t mask, bool activeHigh) {
  unsigned long start = millis();
  while (maskDown(mask, activeHigh) && millis() - start < 3000) delay(10);
  delay(60);
}

// ---------------------------------------------------------------------
//  waitPress -- wait for one specific button, with a countdown.
//    mask == MASK_SW1    -> SW1, direct GPIO,  active-LOW
//    mask == MASK_SW2    -> SW2, expander,     active-LOW
//    mask == MASK_FRONT  -> FRONT, expander,   active-HIGH
//    mask == MASK_BACK   -> BACK, expander,    active-HIGH
//  Returns false if nothing arrives in time.
// ---------------------------------------------------------------------
bool waitPress(uint16_t mask, bool activeHigh, unsigned long timeout) {
  waitRelease(mask, activeHigh);           // ignore a button already held
  lastSec = -1;
  unsigned long start = millis();
  while (millis() - start < timeout) {
    countdown(start, timeout);
    if (maskDown(mask, activeHigh)) {
      waitRelease(mask, activeHigh);
      return true;
    }
  }
  Serial.println("         (no press -- timed out)");
  return false;
}

// ---------------------------------------------------------------------
//  waitStart -- show the instruction, then hold until the operator is
//  ready. Every operator test begins here. Nothing moves before it.
// ---------------------------------------------------------------------
static bool waitStart(const char* title, const char* l1, const char* l2) {
  screen(title, l1, l2, ST77XX_CYAN);
  footer("RIGHT=START", ST77XX_GREEN);
  bool ok = waitPress(MASK_SW1, false, START_TIMEOUT);
  if (!ok) Serial.println("         operator never started the test");
  return ok;
}

// ---------------------------------------------------------------------
//  askYesNo -- RIGHT means yes. Waiting means no.
//
//  There is no NO button. Silence is the answer, and it is the safe one:
//  a question nobody answers must not be allowed to pass a vehicle.
// ---------------------------------------------------------------------
static bool askYesNo(const char* question, const char* detail) {
  screen(question, detail, "OR WAIT FOR NO", ST77XX_YELLOW);
  footer("RIGHT=YES", ST77XX_GREEN);

  waitRelease(MASK_SW1, false);
  lastSec = -1;
  unsigned long start = millis();
  while (millis() - start < ANSWER_TIMEOUT) {
    countdown(start, ANSWER_TIMEOUT);
    if (rightDown()) {
      waitRelease(MASK_SW1, false);
      Serial.println("         operator answered: YES");
      return true;
    }
  }
  Serial.println("         operator answered: NO (waited it out)");
  return false;
}

// Header-declared name, kept so preflight.h does not change.
bool askOperator(const char* question, const char* detail) {
  return askYesNo(question, detail);
}

// Header-declared name. Plain three-line message, no buttons.
void prompt(const char* l1, const char* l2, const char* l3) {
  screen(l1, l2, l3, ST77XX_YELLOW);
}

// ---------------------------------------------------------------------
//  watchWindow -- run an action for a fixed time with a countdown, so
//  the operator has a known number of seconds to look at it.
// ---------------------------------------------------------------------
static void watchWindow(const char* title, const char* l1, unsigned long ms) {
  screen(title, l1, "", ST77XX_WHITE);
  footer("WATCH", ST77XX_CYAN);
  lastSec = -1;
  unsigned long start = millis();
  while (millis() - start < ms) countdown(start, ms);
}

// Motors are already running when this is called, so the countdown on
// screen matches the wheels actually turning. Draw the screen first,
// start the motion, then hold: never the other way round.

// ---------------------------------------------------------------------
//  MOTION LAYERS
//  Private to this file. Neither of the next two is in preflight.h,
//  because nothing outside this file needs to call them. That is what a
//  header is for: publishing what others may use, hiding what they may not.
//
//  LAYER 1 (lowest): four PWM channels, two per motor. Nothing here
//  knows what a vehicle is. It knows about pins.
// ---------------------------------------------------------------------
static void driveMotors(int left, int right) {
  if (left >= 0) { analogWrite(AIN1, left);  analogWrite(AIN2, 0); }
  else           { analogWrite(AIN2, 0);     analogWrite(AIN2, -left); }
  if (right >= 0){ analogWrite(BIN2, right); analogWrite(BIN1, 0); }
  else           { analogWrite(BIN2, 0);     analogWrite(BIN1, -right); }
}

// LAYER 2: the vehicle. A rover is commanded the way a spacecraft is
// commanded: in what the whole machine should do, not in what each
// actuator should do. Lab 8 gave you the forward kinematics:
//
//      v = (pwm_left + pwm_right) / 2        forward velocity
//      w = (pwm_right - pwm_left) / K        turn rate
//
// A checkout needs them solved the other way round -- you say what the
// vehicle should do, and the wheels work out what that means for them:
//
//      pwm_left  = v - K*w/2
//      pwm_right = v + K*w/2
//
// Note what this function does NOT do: it never touches a pin, and it
// holds no opinion about whether anything passed.
static void driveVehicle(float v, float w) {
  float left  = v - (K_TRACK * w) / 2.0;
  float right = v + (K_TRACK * w) / 2.0;

  left  = constrain(left,  -PWM_MAX, PWM_MAX);
  right = constrain(right, -PWM_MAX, PWM_MAX);

  Serial.print("         v=");      Serial.print(v, 0);
  Serial.print("  w=");             Serial.print(w, 0);
  Serial.print("   ->  left=");     Serial.print((int)left);
  Serial.print("  right=");         Serial.println((int)right);

  driveMotors((int)left, (int)right);
}

// Announce which test is starting, on both channels.
static void announce(const char* name) {
  Serial.println();
  Serial.print("--> ");
  Serial.println(name);
}

// =====================================================================
//  THE TESTS
//
//  Every one has the same shape:
//      1. set the subsystem up      (copied from the lab it came from)
//      2. waitStart(...)            operator reads, then presses RIGHT
//      3. do it, with a countdown
//      4. decide, and return one bool
//
//  The pass condition is always written against a limit from
//  preflight.ino. No test invents its own threshold.
// =====================================================================

// --------------------------------------------------------- BUZZER (L1)
// Proves: the buzzer produces two distinguishable tones.
// PASS:   a human hears high then low, and says so.
// The microcontroller has no ears. The cleanest example in the package
// of a test that cannot be automatic.
bool testBuzzer() {
  announce("BUZZER");
  pinMode(BUZZ_PIN, OUTPUT);

  if (!waitStart("BUZZER", "LISTEN FOR", "2 BEEPS")) return false;

  screen("BUZZER", "HIGH...", "", ST77XX_WHITE);
  tone(BUZZ_PIN, 1000, 500);  delay(900);
  screen("BUZZER", "HIGH... LOW", "", ST77XX_WHITE);
  tone(BUZZ_PIN,  500, 500);  delay(900);
  noTone(BUZZ_PIN);

  return askYesNo("HEARD 2 TONES?", "HIGH THEN LOW");
}

// ------------------------------------------------------ LED RING (L5)
// Proves: all 18 NeoPixels light, in all three colour channels.
// PASS:   the operator confirms red, then green, then blue, all 18.
bool testNeoPixels() {
  announce("LED RING");
  strip.begin();

  if (!waitStart("LED RING", "WATCH FOR", "RED GRN BLUE")) return false;

  const uint32_t colours[3] = { strip.Color(60, 0, 0),
                                strip.Color(0, 60, 0),
                                strip.Color(0, 0, 60) };
  const char*    names[3]   = { "RED", "GREEN", "BLUE" };

  for (int c = 0; c < 3; c++) {
    screen("LED RING", names[c], "", ST77XX_WHITE);
    strip.fill(colours[c]);
    strip.show();
    delay(1500);
  }
  strip.clear();
  strip.show();

  reviveDisplay();          // the ring just reset the TFT. Put it back.

  return askYesNo("SAW R G B?", "ALL 18 LIT");
}

// ------------------------------------------------------- 7-SEG (Lab 2)
// Proves: every digit position and every segment driver works.
// PASS:   the operator reads 1 2 3 4, left to right, all four legible.
// A dead segment shows a digit that is almost right, which is the
// hardest kind of fault to see, and the reason a human reads it.
bool test7Seg() {
  announce("7-SEGMENT");
  if (!present(ADDR_LEDMUX)) {
    Serial.println("         display did not answer roll call - not tested");
    return false;
  }

  if (!waitStart("7-SEGMENT", "READ THE GREEN", "LED DIGITS")) return false;

  screen("7-SEGMENT", "SHOULD READ", "1 2 3 4", ST77XX_WHITE);
  footer("WATCH", ST77XX_CYAN);
  lastSec = -1;
  unsigned long start = millis();
  while (millis() - start < WATCH_TIME) {       // multiplex the display
    for (int d = 3; d >= 0; d--) {
      ledmux.write(0x000F);                     // anti-ghost, from Lab 2
      ledmux.write(SEG[4 - d] + POS[d]);        // 1 at left, 4 at right
      delay(2);
    }
    countdown(start, WATCH_TIME);
  }
  ledmux.write(0x000F);

  return askYesNo("SAW 1 2 3 4?", "ALL 4 CLEAR");
}

// ----------------------------------------------------- BUTTONS (Lab 3)
// Proves: all four switches close, on both interfaces and both polarities.
// PASS:   every button answers inside the timeout.
//
// The screen names one button at a time and confirms each press before
// asking for the next, so an operator is never guessing which switch the
// tool is waiting on.
bool testButtons() {
  announce("BUTTONS");

  if (!waitStart("BUTTONS", "PRESS EACH ONE", "AS NAMED")) return false;

  const char*    names[4] = { "RIGHT / SW1", "SW2", "FRONT", "BACK" };
  const uint16_t masks[4] = { MASK_SW1, MASK_SW2, MASK_FRONT, MASK_BACK };
  const bool     high[4]  = { false, false, true, true };
  bool           got[4];

  for (int i = 0; i < 4; i++) {
    screen("PRESS:", names[i], "", ST77XX_YELLOW);
    footer("PRESS+RELEASE", ST77XX_GREEN);
    got[i] = waitPress(masks[i], high[i], BUTTON_TIMEOUT);

    screen("PRESS:", names[i], got[i] ? "OK" : "NO PRESS",
           got[i] ? ST77XX_GREEN : ST77XX_RED);
    delay(700);

    Serial.print("         ");
    Serial.print(names[i]);
    Serial.println(got[i] ? "  ok" : "  NO PRESS");
  }

  return got[0] && got[1] && got[2] && got[3];
}

// --------------------------------------------------------- POT (Lab 5)
// Proves: the throttle potentiometer sweeps its full mechanical travel.
// PASS:   the span between lowest and highest count exceeds POT_SPAN_PCT
//         of full scale. A pot that only moves a little has a cracked
//         wiper, and it reads plausibly right up until it matters.
bool testPot() {
  announce("POTENTIOMETER");
  if (!present(ADDR_TEMPPOT)) {
    Serial.println("         ADC1 did not answer roll call - not tested");
    return false;
  }
  adcTempPot.begin();

  if (!waitStart("POT", "TURN IT FULLY", "BOTH WAYS")) return false;

  screen("POT", "TURN NOW", "", ST77XX_WHITE);
  footer("TURN IT", ST77XX_CYAN);

  uint16_t lo = 65535, hi = 0, pot, temp;
  lastSec = -1;
  unsigned long start = millis();
  while (millis() - start < SWEEP_TIME) {
    if (adcTempPot.read2Ch(&temp, &pot)) {      // ch0 = TEMP0, ch1 = POT
      if (pot < lo) lo = pot;
      if (pot > hi) hi = pot;
    }
    showValue("SPAN ", ((long)hi - (long)lo) * 100L / 65535L);
    countdown(start, SWEEP_TIME);
  }

  long spanPct = ((long)hi - (long)lo) * 100L / 65535L;
  Serial.print("         low: ");   Serial.print(lo);
  Serial.print("  high: ");         Serial.print(hi);
  Serial.print("  span: ");         Serial.print(spanPct);
  Serial.print("%  (limit ");       Serial.print(POT_SPAN_PCT);
  Serial.println("%)");

  return (spanPct >= POT_SPAN_PCT);
}

// -------------------------------------------------------- TEMP (Lab 5)
// Proves TWO things, and this is the interesting test in the package:
//   (a) plausible  -- the reading sits between TEMP_MIN_C and TEMP_MAX_C
//   (b) responsive -- pinching the sensor raises it by TEMP_RISE_C
// A sensor frozen at a plausible number passes (a) and fails (b). That
// is the difference between a reading and a working sensor, and it is
// why (a) alone is not a test.
bool testTemp() {
  announce("TEMPERATURE");
  if (!present(ADDR_TEMPPOT)) {
    Serial.println("         ADC1 did not answer roll call - not tested");
    return false;
  }
  adcTempPot.begin();

  uint16_t raw, pot;
  adcTempPot.read2Ch(&raw, &pot);
  float baseC = (raw * 3.3 / 65535.0 - 0.5) / 0.01;   // TC1047A, Lab 5
  bool  plausible = (baseC > TEMP_MIN_C && baseC < TEMP_MAX_C);

  if (!waitStart("TEMPERATURE", "WARM THE CHIP", "WITH FINGERS")) return false;

  screen("TEMPERATURE", "WARM IT NOW", "", ST77XX_WHITE);
  footer("WARM IT", ST77XX_CYAN);

  float peakC = baseC;
  lastSec = -1;
  unsigned long start = millis();
  while (millis() - start < SWEEP_TIME) {
    if (adcTempPot.read2Ch(&raw, &pot)) {
      float c = (raw * 3.3 / 65535.0 - 0.5) / 0.01;
      if (c > peakC) peakC = c;
    }
    showValue("NOW ", (long)peakC);
    countdown(start, SWEEP_TIME);
  }
  bool responsive = ((peakC - baseC) >= TEMP_RISE_C);

  Serial.print("         start: ");  Serial.print(baseC, 1);
  Serial.print(" C   peak: ");       Serial.print(peakC, 1);
  Serial.print(" C   rise: ");       Serial.print(peakC - baseC, 1);
  Serial.println(" C");
  Serial.print("         plausible: ");  Serial.print(plausible ? "yes" : "NO");
  Serial.print("   responsive: ");       Serial.println(responsive ? "yes" : "NO");

  return plausible && responsive;
}

// ------------------------------------------------------- LIGHT (Lab 5)
// Proves: the photocell responds to the thing it is supposed to measure.
// PASS:   covered reading falls below LIGHT_DROP_PCT of the open reading.
// Whether that limit is right depends on how bright this room is. Say so
// in your report.
bool testLight() {
  announce("LIGHT");
  if (!present(ADDR_LIGHT)) {
    Serial.println("         ADC2 did not answer roll call - not tested");
    return false;
  }
  adcLight.begin();

  uint16_t l1, l2;
  adcLight.read2Ch(&l1, &l2);                   // ch0 = LIGHT1, ch1 = LIGHT2
  uint16_t openRaw = l1;

  if (openRaw < LIGHT_MIN_OPEN) {
    screen("LIGHT", "ROOM TOO DARK", "CANNOT TEST", ST77XX_RED);
    Serial.print("         room reads ");   Serial.print(openRaw);
    Serial.print(", below the ");           Serial.print(LIGHT_MIN_OPEN);
    Serial.println(" needed for a meaningful test");
    delay(2500);
    return false;
  }

  if (!waitStart("LIGHT", "COVER SENSOR", "THEN PRESS")) return false;

  adcLight.read2Ch(&l1, &l2);
  uint16_t covered = l1;

  long pct = (openRaw > 0) ? ((long)covered * 100L / (long)openRaw) : 100L;
  screen("LIGHT", "READING", "", ST77XX_WHITE);
  showValue("PCT OF OPEN ", pct);
  delay(1500);

  Serial.print("         open: ");     Serial.print(openRaw);
  Serial.print("  covered: ");         Serial.print(covered);
  Serial.print("  (");                 Serial.print(pct);
  Serial.print("% of open, limit ");   Serial.print(LIGHT_DROP_PCT);
  Serial.println("%)");

  return (pct < LIGHT_DROP_PCT);
}

// --------------------------------------------------------- IMU (Lab 6)
// Proves: the accelerometer resolves attitude, and in the right direction.
// PASS:   pitch exceeds IMU_TILT_DEG on both sides AND changes sign.
// Sign is the whole point. A sensor mounted backwards passes any test
// that only checks magnitude, then flies the vehicle into the ground.
bool testIMU() {
  announce("IMU");
  if (!IMU.begin()) {
    Serial.println("         LSM6DSOX did not initialise - not tested");
    return false;
  }
  float ax, ay, az;

  if (!waitStart("IMU", "TILT BOARD", "LEFT, HOLD IT")) return false;
  IMU.readAcceleration(ax, ay, az);
  float pitch1 = atan2(ax, az) * 180.0 / PI;
  screen("IMU", "LEFT", "", ST77XX_WHITE);
  showValue("PITCH ", (long)pitch1);
  delay(1200);

  if (!waitStart("IMU", "TILT BOARD", "RIGHT, HOLD IT")) return false;
  IMU.readAcceleration(ax, ay, az);
  float pitch2 = atan2(ax, az) * 180.0 / PI;
  screen("IMU", "RIGHT", "", ST77XX_WHITE);
  showValue("PITCH ", (long)pitch2);
  delay(1200);

  Serial.print("         pitch left: ");  Serial.print(pitch1, 1);
  Serial.print(" deg   pitch right: ");   Serial.print(pitch2, 1);
  Serial.println(" deg");

  bool bigEnough   = (fabs(pitch1) > IMU_TILT_DEG) && (fabs(pitch2) > IMU_TILT_DEG);
  bool signFlipped = (pitch1 * pitch2 < 0);
  return bigEnough && signFlipped;
}

// ----------------------------------------------------- COMPASS (Lab 6)
// Proves: the magnetometer tracks heading.
// PASS:   azimuth moves at least COMPASS_TURN_DEG on a half turn.
// Keep the board away from the steel table frame: the field it measures
// includes everything in the room, which is a fault mode of its own.
bool testCompass() {
  compass.setCalibrationOffsets(-1323.00, -192.00, -1074.00);
  compass.setCalibrationScales(0.93, 0.95, 1.14);
  announce("COMPASS");
  if (!present(ADDR_COMPASS)) {
    Serial.println("         QMC5883L did not answer roll call - not tested");
    return false;
  }
  compass.init();

  compass.read();
  int a1 = compass.getAzimuth();
  screen("COMPASS", "START HEADING", "", ST77XX_WHITE);
  showValue("DEG ", a1);
  delay(1500);

  if (!waitStart("COMPASS", "TURN BOARD", "HALF A TURN")) return false;
  compass.read();
  int a2 = compass.getAzimuth();

  int change = abs(a2 - a1);
  if (change > 180) change = 360 - change;      // headings wrap at 360

  screen("COMPASS", "CHANGE", "", ST77XX_WHITE);
  showValue("DEG ", change);
  delay(1500);

  Serial.print("         before: ");  Serial.print(a1);
  Serial.print(" deg   after: ");     Serial.print(a2);
  Serial.print(" deg   change: ");    Serial.print(change);
  Serial.print(" deg  (limit ");      Serial.print(COMPASS_TURN_DEG);
  Serial.println(")");

  return (change >= COMPASS_TURN_DEG);
}

// ---------------------------------------------------- RANGEFINDER (L8)
// Proves: the time-of-flight sensor measures a known distance correctly.
// PASS:   a hand at RANGE_TARGET_MM reads within RANGE_TOL_MM.
//
// The live readout matters here. The operator is trying to hold a hand at
// 100 mm, and without a number on the screen they are guessing at a
// distance while the tool silently disagrees with them.
bool testRange() {
  announce("RANGEFINDER");
  if (!present(ADDR_RANGE)) {
    Serial.println("         VL6180X did not answer roll call - not tested");
    Serial.println("         (is the P11 enable line high? see beginCheckout)");
    return false;
  }
  rangefinder.init();
  rangefinder.configureDefault();
  rangefinder.setTimeout(500);

  if (!waitStart("RANGE", "HAND ABOUT", "100 MM AWAY")) return false;

  // Live aiming window: the operator adjusts until the number reads 100.
  screen("RANGE", "HOLD AT 100", "", ST77XX_WHITE);
  footer("AIM IT", ST77XX_CYAN);
  lastSec = -1;
  unsigned long start = millis();
  while (millis() - start < AIM_TIME) {
    int mm = rangefinder.readRangeSingleMillimeters();
    if (!rangefinder.timeoutOccurred()) showValue("MM ", mm);
    countdown(start, AIM_TIME);
  }

  // Now take the measurement that counts: five samples, not one.
  screen("RANGE", "MEASURING", "HOLD STILL", ST77XX_YELLOW);
  long sum = 0;
  int  good = 0;
  for (int i = 0; i < 5; i++) {
    int mm = rangefinder.readRangeSingleMillimeters();
    if (!rangefinder.timeoutOccurred()) { sum += mm; good++; }
    delay(60);
  }
  if (good == 0) {
    Serial.println("         sensor timed out on every sample");
    return false;
  }
  long avg = sum / good;

  screen("RANGE", "MEASURED", "", ST77XX_WHITE);
  showValue("MM ", avg);
  delay(1500);

  Serial.print("         measured: ");  Serial.print(avg);
  Serial.print(" mm   target: ");       Serial.print(RANGE_TARGET_MM);
  Serial.print(" +/- ");                Serial.print(RANGE_TOL_MM);
  Serial.println(" mm");

  return (labs(avg - RANGE_TARGET_MM) <= RANGE_TOL_MM);
}

// ------------------------------------------------------- SERVO (Lab 8)
// Proves: the mast actuator drives to commanded position, both ways.
// PASS:   the operator confirms a smooth sweep in both directions.
// The servo has no feedback path to the microcontroller. Commanding a
// position and assuming it arrived is exactly the assumption a checkout
// exists to break.
bool testServo() {
  announce("SERVO");
  mast.attach(SERVO_PIN);

  if (!waitStart("SERVO", "WATCH THE MAST", "SWEEP")) return false;

  int  sweep[5]  = { 0, 90, 180, 90, 0 };
  char label[16];
  for (int i = 0; i < 5; i++) {
    sprintf(label, "%d DEG", sweep[i]);
    screen("SERVO", label, "", ST77XX_WHITE);
    mast.write(sweep[i]);
    delay(900);
  }
  mast.detach();

  return askYesNo("MAST SWEPT?", "SMOOTH, BOTH WAYS");
}

// ------------------------------------------------------ MOTORS (Lab 8)
// Proves: both drive channels work, and they work independently.
// PASS:   the operator confirms all three phases.
//
// Each phase is confirmed on its own, right after it runs, instead of
// asking one vague question at the end. That gives the operator time to
// look, and it isolates the fault: "reverse failed" is a repair note,
// "the motors failed" is not.
//
// Straight running alone is a weak test, because a differential channel
// that is stuck or swapped hides inside it. Commanding a turn rate with
// v = 0 forces the wheels apart and leaves that fault nowhere to hide.
//
// PIT MODE. This test refuses to run until a human confirms the wheels
// are off the bench. Every test stand in the world has this interlock,
// and every one of them has it because somebody once did not.
bool testMotors() {
  announce("MOTORS");

  if (!askYesNo("WHEELS OFF BENCH?", "PIT MODE")) {
    Serial.println("         interlock not confirmed - test refused to run");
    return false;
  }
  if (!waitStart("MOTORS", "3 PHASES", "WATCH WHEELS")) return false;

  driveVehicle(150, 0);                              // motion starts first
  watchWindow("PHASE 1 of 3", "FORWARD", WATCH_TIME); // countdown while it runs
  driveVehicle(0, 0);
  bool fwd = askYesNo("BOTH WENT FWD?", "PHASE 1");

  driveVehicle(-150, 0);
  watchWindow("PHASE 2 of 3", "REVERSE", WATCH_TIME);
  driveVehicle(0, 0);
  bool rev = askYesNo("BOTH WENT BACK?", "PHASE 2");

  driveVehicle(0, 200);
  watchWindow("PHASE 3 of 3", "SPIN IN PLACE", WATCH_TIME);
  driveVehicle(0, 0);
  bool spin = askYesNo("SPUN IN PLACE?", "PHASE 3");

  Serial.print("         forward: ");   Serial.print(fwd  ? "yes" : "NO");
  Serial.print("   reverse: ");         Serial.print(rev  ? "yes" : "NO");
  Serial.print("   differential: ");    Serial.println(spin ? "yes" : "NO");

  return fwd && rev && spin;
}

// -------------------------------------------------------- WIFI (Lab 10)
// Proves: the radio module answers and can hear a network.
// PASS:   the module reports firmware, and at least one network is
//         audible at or above WIFI_RSSI_MIN.
// Fully automatic: no human is faster at this than the radio is.
bool testWiFi() {
  announce("WIFI");
  screen("WIFI", "SCANNING...", "NO ACTION NEEDED", ST77XX_WHITE);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("         radio module did not answer");
    return false;
  }
  Serial.print("         firmware: ");
  Serial.println(WiFi.firmwareVersion());

  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("         no networks heard");
    return false;
  }

  long best = -200;
  for (int i = 0; i < n; i++) {
    long r = WiFi.RSSI(i);
    if (r > best) best = r;
  }
  Serial.print("         networks heard: ");  Serial.print(n);
  Serial.print("   strongest: ");             Serial.print(best);
  Serial.print(" dBm  (limit ");              Serial.print(WIFI_RSSI_MIN);
  Serial.println(" dBm)");

  return (best >= WIFI_RSSI_MIN);
}
