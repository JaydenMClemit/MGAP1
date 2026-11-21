// =====================================================================
//  preflight.ino  --  THE CHECKLIST
//  EVENT HORIZON PROPULSION LABS  ::  Vehicle Pre-Flight Checkout
//  CESC 325L Lab 11
//
//  VEHICLE ID: #07 
//
//  This is the whole procedure, on one screen, in the order it runs.
//  There is no hidden logic in this file: every line below is either a
//  limit you set or a test you called for. The engineering is in
//  myFunctions.ino; the decisions are here.
//
//  You own this file. You do not write functions in it.
// =====================================================================

#include <Arduino.h>

#include "preflight.h"

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