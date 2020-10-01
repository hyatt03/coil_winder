/**
 * Coil winder main source file
 * Uses two steppers to wind a small coil when a button is pressed
 */

// Start by grabbing the relevant libraries
#include "Arduino.h"
#include "AccelStepper.h"
#include "math.h"

// Define stepper 1 pins
#define dir_pin_m1 6
#define step_pin_m1 3
#define enable_pin_m1 8

// Define stepper 2 pins
#define dir_pin_m2 5
#define step_pin_m2 2
#define enable_pin_m2 8

// Set motor interface type (to use DRV 8825)
#define motor_interfaceType 1

// Set the calibration numbers
// Operator calibration
#define operator_delay 1000

// Motor calibration
#define steps_per_revolution 3200 // Steps required for one full pass
#define max_speed 3200.0 // Max speed in steps per second
#define max_acceleration 3000 // Max acceleration in steps per second squared

// Screw calibration
#define mm_per_step 0.0025 // Linear distance of finger per step (in mm)

// Coil settings
#define wirewidth 0.2 // Width of the wire used to spool the magnet
#define n_layers 9 // Number of layers required
#define n_turns_per_layer 110 // Number of turns per layer
#define coil_height 25 // Height of the coil in mm (essentially how far we're moving during the spooling)

// Setup stepper class instances
AccelStepper coil_motor = AccelStepper(motor_interfaceType, step_pin_m1, dir_pin_m1);
AccelStepper finger_motor = AccelStepper(motor_interfaceType, step_pin_m2, dir_pin_m2);

// Setup a bool to check if we're spooling the coil (Starts with false)
bool is_spooling = false;
bool done_spooling = false;

// Setup a bool to hold the LED state
bool led_state = false;

// Setup counting variables
unsigned long current_millis = 0;
unsigned long previous_millis = 0;
unsigned long led_interval_before = 500;
unsigned long led_interval_after = 1000;

// Create a string to hold incoming data
String input_string = "";
bool string_complete = false;

void setup() {
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup the enable pins
  pinMode(enable_pin_m1, OUTPUT);
  pinMode(enable_pin_m2, OUTPUT);
  
  // Don't energize the motors, we have to wait for the button signal
  digitalWrite(enable_pin_m1, HIGH);
  digitalWrite(enable_pin_m2, HIGH);
  
  // Set the maximum speed in steps per second
  coil_motor.setMaxSpeed(max_speed);
  finger_motor.setMaxSpeed(max_speed);

  // Set the max acceleration
  coil_motor.setAcceleration(max_acceleration);
  finger_motor.setAcceleration(max_acceleration);

  // Set the previous and current millis
  previous_millis = millis();
  current_millis = millis();

  // Open serial
  Serial.begin(9600);
  Serial.println("Ready! Type help for more info");
  input_string.reserve(200);
}

// just calls run on both motors
void run_motors() {
  coil_motor.run();
  finger_motor.run();
}

// Called when the button is pressed
void start_spooling() {
  // turn the LED permenantly on to indicate we're running
  digitalWrite(LED_BUILTIN, HIGH);

  // Set the is_spooling variable so we call the correct function at subsequent iterations
  is_spooling = true;

  // Energize the coils
  digitalWrite(enable_pin_m1, LOW);
  digitalWrite(enable_pin_m2, LOW);
}

// Called when we're done spooling
void stop_spooling() {
  // Turn off the LED
  digitalWrite(LED_BUILTIN, HIGH);

  // Indicate we're done spooling
  done_spooling = true;
}

// Called when we're done spooling and the button has been pressed
void turn_off_coils() {
  // De-energize the coils
  digitalWrite(enable_pin_m1, HIGH);
  digitalWrite(enable_pin_m2, HIGH);

  // Reset the machine, ready for the next run
  is_spooling = false;
  done_spooling = false;
}

void do_calibration() {
  // Energize the coils
  digitalWrite(enable_pin_m1, LOW);
  digitalWrite(enable_pin_m2, LOW);

  float calib_distance = -50.0;

  int steps_to_go_finger = round(calib_distance / mm_per_step);
  float speed_ratio = ((float) steps_to_go_finger) / ((float) steps_per_revolution);
  float finger_speed = max_speed * speed_ratio;

  Serial.print("Moving ");
  Serial.print(steps_to_go_finger);
  Serial.println(" steps");

  finger_motor.setSpeed(finger_speed);
  finger_motor.move(steps_to_go_finger);
  
  while (coil_motor.distanceToGo() != 0 || finger_motor.distanceToGo() != 0) {
    run_motors();
  }

  // De-energize the coils
  digitalWrite(enable_pin_m1, HIGH);
  digitalWrite(enable_pin_m2, HIGH);
}

// Called at every loop until the button has been pressed
void monitor_button() {
  unsigned long led_interval = 0;
  if (is_spooling) {
    led_interval = led_interval_after;
  } else {
    led_interval = led_interval_before;
  }

  // We blink the led every 0.5s to indicate we're ready for input
  current_millis = millis();
  if (current_millis - previous_millis >= led_interval) {
    // save the last time you blinked the LED
    previous_millis = current_millis;

    // if the LED is off turn it on and vice-versa:
    if (led_state) {
      led_state = false;
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      led_state = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  // Now we actually read the button state
  // If it's high we want to start spooling
  if (string_complete) {
    input_string.trim();
    Serial.println(input_string);
    if (input_string == "go" && !is_spooling) {
      Serial.println("Going!");
      start_spooling();
    }

    if (input_string == "calib") {
      Serial.println("Doing calibration!");
      do_calibration();
    }

    if (input_string == "off") {
      Serial.println("Turning off the motors!");
      turn_off_coils();
    }

    if (input_string == "help") {
      Serial.println("Available commands are: go, calib, off, help");
    }

    input_string = "";
    string_complete = false;
  }
}

// Called at every loop after the button has been pressed
void spooling() {
  // If the motors aren't done move, just move them
  if (coil_motor.distanceToGo() > 0 || finger_motor.distanceToGo() > 0) {
    run_motors();
  } else if (!done_spooling) {
    // We start by waiting, to allow the operator prep time
    // To get the wire ready for example
    // delay(operator_delay);

    int m = 1;
    int turns = n_turns_per_layer / m;

    // For each layer in the spool
    for (int i = 0; i < n_layers; i++) {
      // For each turn per layer
      Serial.print("Starting layer: ");
      Serial.println(i + 1);

      for (int j = 0; j < turns; j++) {
        // Compute distance and speed
        int steps_to_go_finger = round(pow(-1., i) * (((float) coil_height) / ((float) turns)) / mm_per_step);

        // Set the speed
        finger_motor.setSpeed(max_speed * 0.1);
        coil_motor.setSpeed(max_speed);

        // Set the distance
        finger_motor.move(steps_to_go_finger);
        coil_motor.move(m * steps_per_revolution);

        // Move the motors
        while (coil_motor.distanceToGo() > 0 || finger_motor.distanceToGo() > 0) {
          run_motors();
        }
      }
    }

    // We're done making this spool
    stop_spooling();
  }
  else {
    // We're waiting for user action (indicated by blinking LED)
    monitor_button();
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char) Serial.read();

    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      string_complete = true;
    } else {
      // otherwise add it to the inputString:
      input_string += inChar;
    }
  }
}

// Main loop: Just calls the relevant function
void loop() {
  // Start by checking if we're spooling
  if (is_spooling) {
    // If we are, just keep going
    spooling();
  } else {
    // Otherwise we want to monitor the button
    monitor_button();
  }
}
