//======================================================================
//
// Test program to test the infrared sensors (and motors) of the
// 4tronix initio robot car. One can run this program within an
// ssh session.
//
// author: Raimund Kirner, University of Hertfordshire
//         initial version: Oct.2016
//
// license: GNU LESSER GENERAL PUBLIC LICENSE
//          Version 2.1, February 1999
//          (for details see LICENSE file)
//
// Compilation:
// gcc -o line_follower -Wall -Werror -lcurses -lwiringPi -lpthread -linitio line_follower.c
//
//======================================================================

#include <stdlib.h>
#include <initio.h>
#include <curses.h>

// Line sensor calibration: Adjust WHITE macro based on your specific car
// Testing showed that for most cars, sensor returns 0 when detecting white surface
// and 1 when detecting black line. Verify this during initial testing.
#define WHITE(a) ((a) == 0) // Car calibration: Sensor returns 0 for white surface
// Alternative calibration (uncomment if your car has inverted sensor logic):
// #define WHITE(a) ((a)!=0)  // Use this if sensor returns 1 for white surface
#define BLACK(a) (!WHITE(a))

// Control parameters (tuned through testing)
#define FORWARD_SPEED 60      // PWM value for straight-line driving (0-100)
#define FORWARD_SPEED_FAST 80 // PWM value for fast mode (competition)
#define SPIN_SPEED 100        // PWM value for pivot corrections (0-100)
#define SEARCH_SPEED 80       // PWM value for line recovery search
#define SEARCH_DURATION 200   // Time (ms) to search when line is lost
#define LOOP_DELAY 100        // Control loop period in milliseconds

//======================================================================
// line_follower():
// Autonomous line following with obstacle avoidance.
//
// Control Strategy:
// 1. Obstacle avoidance has highest priority (safety first)
// 2. Line following uses binary sensor feedback for steering
// 3. Lost line triggers recovery search behavior
//
// Sensors:
// - IR obstacle sensors (left/right): Detect objects in front
// - Line sensors (left/right): Detect black line vs white floor
//
// Actuators:
// - DC motors (left/right): Controlled via PWM for speed and direction
//======================================================================
void line_follower(int argc, char *argv[])
{
  int ch = 0;
  int irL, irR;                     // Status of infrared obstacle sensors (0=clear, 1=obstacle)
  int lfL, lfR;                     // Status of line sensors (0=white, 1=black for this car)
  int lostLineCounter = 0;          // Counter for line loss recovery attempts
  int fastMode = 0;                 // Toggle for competition speed mode (press 'f')
  int currentSpeed = FORWARD_SPEED; // Active forward speed

  while (ch != 'q')
  {
    mvprintw(1, 1, "%s: Press 'q' to quit, 'f' to toggle fast mode", argv[0]);
    mvprintw(2, 1, "Mode: %s (Speed: %d)                    ",
             fastMode ? "FAST" : "NORMAL", currentSpeed);

    // Read all sensors
    irL = initio_IrLeft();
    irR = initio_IrRight();
    lfL = initio_ReadLineSensorLeft();
    lfR = initio_ReadLineSensorRight();

    // Display sensor status for debugging
    mvprintw(4, 1, "Sensors - IR: L=%d R=%d | Line: L=%d R=%d   ", irL, irR, lfL, lfR);

    // ===================================================================
    // PRIORITY 1: OBSTACLE AVOIDANCE (Safety Critical)
    // ===================================================================
    // If either IR sensor detects an obstacle, immediately stop.
    // This overrides all other behaviors to prevent collisions.
    // Uses OR logic (||) for redundancy - if either sensor fails,
    // the other can still trigger emergency stop.
    if (irL != 0 || irR != 0)
    {
      mvprintw(5, 1, "Action 1: STOP - Obstacle detected (IR: L=%d R=%d)     ", irL, irR);
      initio_SetMotors(0, 0); // Emergency stop
      lostLineCounter = 0;    // Reset recovery counter
    }

    // ===================================================================
    // PRIORITY 2: LINE FOLLOWING (Normal Operation)
    // ===================================================================
    // No obstacle detected, proceed with line following logic

    // STATE 1: Both sensors on black line -> Drive straight
    // This is the ideal state where car is centered on the line
    else if (BLACK(lfL) && BLACK(lfR))
    {
      mvprintw(5, 1, "Action 2: Straight forward (Line: L=%d R=%d)    ", lfL, lfR);
      initio_SetMotors(currentSpeed, currentSpeed); // Balanced forward motion
      lostLineCounter = 0;                          // Reset recovery counter
    }

    // STATE 2: Left sensor sees white, right sees black -> Car drifted right
    // Correction: Spin left to bring left sensor back onto the black line
    // Using differential steering: left motor backward, right motor forward
    else if (WHITE(lfL) && BLACK(lfR))
    {
      mvprintw(5, 1, "Action 3: Correcting LEFT (Line: L=%d R=%d)    ", lfL, lfR);
      initio_SetMotors(-SPIN_SPEED, SPIN_SPEED); // Pivot left
      lostLineCounter = 0;
    }

    // STATE 3: Left sensor sees black, right sees white -> Car drifted left
    // Correction: Spin right to bring right sensor back onto the black line
    else if (BLACK(lfL) && WHITE(lfR))
    {
      mvprintw(5, 1, "Action 4: Correcting RIGHT (Line: L=%d R=%d)    ", lfL, lfR);
      initio_SetMotors(SPIN_SPEED, -SPIN_SPEED); // Pivot right
      lostLineCounter = 0;
    }

    // STATE 4: Both sensors see white -> Line lost
    // This can happen at sharp turns or if car goes off-track
    // Recovery strategy: Pivot search to relocate the line
    else
    {
      mvprintw(5, 1, "Action 5: LINE LOST - Searching (Line: L=%d R=%d)    ", lfL, lfR);

      // Increment lost line counter
      lostLineCounter++;

      // Recovery behavior: Alternate search direction based on counter
      // This creates a sweeping search pattern
      if (lostLineCounter % 2 == 1)
      {
        // Odd attempts: Search by pivoting left
        initio_SetMotors(-SEARCH_SPEED, SEARCH_SPEED);
      }
      else
      {
        // Even attempts: Search by pivoting right
        initio_SetMotors(SEARCH_SPEED, -SEARCH_SPEED);
      }

      // If lost for too long (>10 cycles = 1 second), stop for safety
      if (lostLineCounter > 10)
      {
        mvprintw(5, 1, "Action 5: LINE LOST - Stopping (timeout)              ");
        initio_SetMotors(0, 0);
        lostLineCounter = 0; // Reset for next attempt
      }
    }

    // Check for user input (non-blocking)
    ch = getch();
    if (ch != ERR)
    {
      mvprintw(6, 1, "Key pressed: '%c' (%d)                    ", ch, ch);

      // Toggle fast mode with 'f' key
      if (ch == 'f' || ch == 'F')
      {
        fastMode = !fastMode;
        currentSpeed = fastMode ? FORWARD_SPEED_FAST : FORWARD_SPEED;
        mvprintw(7, 1, "Speed mode changed to: %s                    ",
                 fastMode ? "FAST" : "NORMAL");
      }
    }

    refresh(); // Update curses display

    // Control loop timing: 100ms period provides adequate response time
    // while avoiding excessive CPU usage. This gives ~10Hz control frequency.
    // WCET analysis: Sensor reads + logic + motor commands < 5ms
    // Therefore 100ms delay ensures deterministic timing with margin.
    delay(LOOP_DELAY);
  } // while

  return;
}

//======================================================================
// main(): initialisation of libraries, etc
//======================================================================
int main(int argc, char *argv[])
{
  WINDOW *mainwin = initscr(); // curses: init screen
  noecho();                    // curses: prevent the key being echoed
  cbreak();                    // curses: disable line buffering
  nodelay(mainwin, TRUE);      // curses: set getch() as non-blocking
  keypad(mainwin, TRUE);       // curses: enable detection of cursor and other keys

  initio_Init(); // initio: init the library

  line_follower(argc, argv);

  initio_Cleanup(); // initio: cleanup the library (reset robot car)
  endwin();         // curses: cleanup the library
  return EXIT_SUCCESS;
}
