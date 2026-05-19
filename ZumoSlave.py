/*
 * Zumo Robot Closed-Loop Serial Controller
 * Receives Linear (V) and Angular (W) velocity commands over USB Serial
 * Converts to Left/Right PWM using Characterization Look-Up Tables (LUT)
 */

#include <Wire.h>
#include <Zumo32U4.h>

// --- HARDWARE OBJECTS ---
Zumo32U4Motors motors;
Zumo32U4OLED display;

// --- ROBOT CONSTANTS ---
const float TRACK_WIDTH = 0.175; // Distance between tracks in meters

// --- CHARACTERIZATION DATA (LUT) ---
const int NUM_DATA_POINTS = 9;
const float lut_velocities[] = {0.0, 0.035, 0.102, 0.168, 0.236, 0.305, 0.371, 0.427, 0.483};
const int lut_pwms_R[]       = {0, 50, 100, 150, 200, 250, 300, 350, 400};
const int lut_pwms_L[]       = {0, 45, 90, 135, 185, 240, 289, 340, 390};
const int MAX_PWM = 400;

// --- STATE VARIABLES ---
unsigned long last_command_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500; // Stop motors if no data for 500ms

// --- CORE FUNCTIONS ---

/**
 * Linearly interpolates a target track velocity into a PWM signal
 */
int velocityToPWM(float target_v, const int* motor_pwm_array) {
  float abs_v = abs(target_v);
  int dir_sign = (target_v < 0) ? -1 : 1;

  // Edge Case 1: Stopped
  if (abs_v == 0.0) return 0;
  
  // Edge Case 2: Target velocity is higher than max characterized speed
  if (abs_v >= lut_velocities[NUM_DATA_POINTS - 1]) {
    return dir_sign * motor_pwm_array[NUM_DATA_POINTS - 1];
  }

  // Linear Interpolation
  for (int i = 0; i < NUM_DATA_POINTS - 1; i++) {
    if (abs_v >= lut_velocities[i] && abs_v <= lut_velocities[i+1]) {
      
      float v_low = lut_velocities[i];
      float v_high = lut_velocities[i+1];
      int pwm_low = motor_pwm_array[i];
      int pwm_high = motor_pwm_array[i+1];
      
      int interpolated_pwm = pwm_low + ((abs_v - v_low) * (pwm_high - pwm_low) / (v_high - v_low));
      return dir_sign * interpolated_pwm;
    }
  }
  
  return 0; // Failsafe return
}

/**
 * Converts global V and W commands into individual motor PWMs
 */
void calculateMotorCommands(float V, float W, int &out_pwmL, int &out_pwmR) {
  // 1. Inverse Kinematics
  float v_left  = V - (W * TRACK_WIDTH / 2.0);
  float v_right = V + (W * TRACK_WIDTH / 2.0);

  // 2. Interpolate the PWM based on the LUTs
  out_pwmL = velocityToPWM(v_left, lut_pwms_L);
  out_pwmR = velocityToPWM(v_right, lut_pwms_R);

  // 3. Constrain to prevent overflow
  out_pwmL = constrain(out_pwmL, -MAX_PWM, MAX_PWM);
  out_pwmR = constrain(out_pwmR, -MAX_PWM, MAX_PWM);
}

// --- ARDUINO SETUP & LOOP ---

void setup() {
  // Crucial: Baud rate must match the Python serial setup!
  Serial.begin(115200); 
  
  display.clear();
  display.print(F("Listening"));
  
  // Initialize the watchdog timer
  last_command_time = millis(); 
}

void loop() {
  // 1. Check for incoming Serial data from the Raspberry Pi
  if (Serial.available() > 0) {
    // Read the incoming string until the newline character '\n'
    String incomingCmd = Serial.readStringUntil('\n');
    incomingCmd.trim(); // Clean up any trailing \r or spaces
    
    // 2. Parse the "V,W" format
    int commaIndex = incomingCmd.indexOf(',');
    if (commaIndex > 0) {
      float target_V = incomingCmd.substring(0, commaIndex).toFloat();
      float target_W = incomingCmd.substring(commaIndex + 1).toFloat();
      
      // 3. Calculate PWMs and drive motors
      int left_pwm = 0, right_pwm = 0;
      calculateMotorCommands(target_V, target_W, left_pwm, right_pwm);
      motors.setSpeeds(left_pwm, right_pwm);
      
      // 4. Reset watchdog timer because we just got a valid command
      last_command_time = millis();
    }
  }

  // 5. Failsafe Watchdog
  // If the Pi stops sending commands for more than half a second, kill the motors.
  if (millis() - last_command_time > WATCHDOG_TIMEOUT_MS) {
    motors.setSpeeds(0, 0);
    
    // We use a static variable here so we aren't clearing the screen 
    // a thousand times a second, which would actually slow down the loop.
    static unsigned long last_display_update = 0;
    if (millis() - last_display_update > 1000) {
        display.clear();
        display.print(F("TIMEOUT"));
        last_display_update = millis();
    }
  }
}
