#ifndef LAUNCHER_H
#define LAUNCHER_H

// Definitions for both controllers and remotes
  
  /* 

  State machine defintions

  1) STATE_BOOT - power on, try to link with controller
  2) STATE_NOCONT - linked but no continuity
  3) STATE_READY - linked, continuity, and ready for commands
  4) STATE_ARMED - the controller has asked us to arm
  5) STATE_LAUNCH - we've been given the command to launch
  6) STATE_DONE - we no longer have continuity and cannot be used without a reset
  7) STATE_ERROR - something is horribly wrong
  */
#define STATE_BOOT      0
#define STATE_NOCONT    1
#define STATE_READY     2
#define STATE_ARMED     3
#define STATE_LAUNCH    4
#define STATE_DONE      5
#define STATE_ERROR     6
#define MIN_NODE        2
#define MAX_NODE        5 
#define HB_CHECK_TIME 5000 // Check for hearbeat with linked controller every 5 seconds
#define MAX_HB_FAIL 5 // after 5 failed heartbeats go back to booting state

// Frequency
#define RF95_FREQ 		918.0
#define RF95_TX_HIGH  	23
#define RF95_TX_NORMAL  13

// Each state name
const char* stateName[] = {
  "BOOTING",
  "NO CONTINUITY",
  "READY",
  "ARMED",
  "LAUNCHING",
  "DONE",
  "STATE_ERROR"
};

// Each state's colors
int ledState[7][2] = {
  { RGB_BLUE,    BLINK_SLOW },    // Blue for Booting
  { RGB_YELLOW,  BLINK_MED },     // Yellow for no continuity
  { RGB_GREEN,   BLINK_MED },     // Green for ready
  { RGB_RED,     BLINK_FAST },    // Red for armed
  { RGB_MAGENTA, BLINK_FAST },    // Magenta for launching
  { RGB_WHITE,   BLINK_SLOW },    // White for done
  { RGB_WHITE,   BLINK_FAST }     // White (fast blink) for error
};

#endif