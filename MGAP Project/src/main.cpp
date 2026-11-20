// =====================================================================
//  preflight.ino  --  THE CHECKLIST
//  EVENT HORIZON PROPULSION LABS  ::  Vehicle Pre-Flight Checkout
//  CESC 325L Lab 11
//
//  VEHICLE ID: #07        INSPECTORS: Aditya Shastri, Khalifa and Jay Sadafule
//
//  This is the whole procedure, on one screen, in the order it runs.
//  There is no hidden logic in this file: every line below is either a
//  limit you set or a test you called for. The engineering is in
//  myFunctions.ino; the decisions are here.
//
//  You own this file. You do not write functions in it.
// =====================================================================
#include <preflight.h>

// Ramndom comment for github testing shit 

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

// ---------------------------------------------------------------------
//  3.  THE CHECKLIST
//
//  AUTO     = the vehicle checks itself. No human needed.
//  OPERATOR = a human must do something and the vehicle reads the result.
//
//  Every operator test waits for you to press RIGHT before anything
//  moves. RIGHT means yes. Waiting means no.
//
//  Comment a line out and that subsystem is not tested. It will not
//  appear on the health report, and the certificate will not mention
//  it. "Not tested" is not the same as "passed": whatever you leave
//  out here, you are choosing not to know.
// ---------------------------------------------------------------------
void setup() {

  beginCheckout();                             // power up + I2C roll call
  addResult("WIFI",    testWiFi());            // AUTO      Lab 10
  addResult("BUZZER",  testBuzzer());          // OPERATOR  Lab 1
  addResult("RING",    testNeoPixels());       // OPERATOR  Lab 5
  addResult("7SEG",    test7Seg());            // OPERATOR  Lab 2
  addResult("BUTTONS", testButtons());         // OPERATOR  Lab 3
  addResult("POT",     testPot());             // OPERATOR  Lab 5
  addResult("TEMP",    testTemp());            // OPERATOR  Lab 5
  addResult("LIGHT",   testLight());           // OPERATOR  Lab 5
  addResult("IMU",     testIMU());             // OPERATOR  Lab 6
  addResult("RANGE",   testRange());           // OPERATOR  Lab 8
  addResult("SERVO",   testServo());           // OPERATOR  Lab 8
  addResult("MOTORS",  testMotors());          // OPERATOR  Lab 8  PIT MODE
  addResult("COMPASS", testCompass());         // OPERATOR  Lab 6

  // sweepBus();                               // +3 option, see Part 4

  // addResult("MIC", testMicrophone());       // Part 2 experiment 2

  endCheckout();                               // health report + verdict
}

// The checkout runs once, at power-up, exactly like the real thing.
// To run it again: press the white RESET button on the board.
void loop() { }