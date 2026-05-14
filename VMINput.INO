/*
 * Zumo Robot Open-Loop Kinematics & Asymmetric PWM Lookup
 * Converts Linear (V) and Angular (W) velocity to Left/Right PWM
 */

#include <Wire.h>
#include <Zumo32U4.h>

// --- HARDWARE OBJECTS ---
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED display;

// --- USER CONFIGURATION ---
// Set your desired velocities and movement time here:
float TARGET_LINEAR_V = .3;       // Target linear velocity in m/s
float TARGET_RADIAL_W = PI/2 ;          // Target angular velocity in rad/s
unsigned long MOVE_DURATION_MS = 4000; // Time to move in milliseconds (e.g., 3000 = 3 seconds)


// --- ROBOT CONSTANTS ---
// Distance between the center line of the left track and right track in meters.
const float TRACK_WIDTH = 0.175; // Update with your physical measurement!

// --- CHARACTERIZATION DATA (LUT) ---
const int NUM_DATA_POINTS = 9;
const float lut_velocities[] = {0.0, 0.035, 0.102, 0.168, 0.236, 0.305, 0.371, 0.427, 0.483};

// Separate arrays for Left and Right motors based on your data
const int lut_pwms_R[]       = {0, 50, 100, 150, 200, 250, 300, 350, 400};
const int lut_pwms_L[]       = {0, 45, 90, 135, 185, 240, 289, 340, 390};

// Maximum PWM allowed by your motor drivers
const int MAX_PWM = 400;


// --- CORE FUNCTIONS ---

/**
 * Linearly interpolates a target track velocity into a PWM signal
 * using a specific motor's characterization array.
 */
int velocityToPWM(float target_v, const int* motor_pwm_array) {
  float abs_v = abs(target_v);
  int dir_sign = (target_v < 0) ? -1 : 1;

  // Edge Case 1: Stopped
  if (abs_v == 0.0) return 0;

  // Edge Case 2: Target velocity is higher than our max characterized speed
  if (abs_v >= lut_velocities[NUM_DATA_POINTS - 1]) {
    return dir_sign * motor_pwm_array[NUM_DATA_POINTS - 1];
  }

  // Linear Interpolation: Find which two data points the target speed falls between
  for (int i = 0; i < NUM_DATA_POINTS - 1; i++) {
    if (abs_v >= lut_velocities[i] && abs_v <= lut_velocities[i+1]) {
      
      float v_low = lut_velocities[i];
      float v_high = lut_velocities[i+1];
      int pwm_low = motor_pwm_array[i];
      int pwm_high = motor_pwm_array[i+1];

      // Interpolation formula
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
  // 1. Inverse Kinematics (Assuming positive W = CCW rotation)
  float v_left  = V - (W * TRACK_WIDTH / 2.0);
  float v_right = V + (W * TRACK_WIDTH / 2.0);

  // 2. Lookup/Interpolate the PWM, passing the specific array for each motor
  out_pwmL = velocityToPWM(v_left, lut_pwms_L);
  out_pwmR = velocityToPWM(v_right, lut_pwms_R);

  // 3. Constrain to prevent overflow
  out_pwmL = constrain(out_pwmL, -MAX_PWM, MAX_PWM);
  out_pwmR = constrain(out_pwmR, -MAX_PWM, MAX_PWM);
}


// --- ARDUINO SETUP & LOOP ---

void setup() {
  Serial.begin(115200);
  
  // Optional: clear display on boot
  display.clear();
}

void loop() {
  // 1. Prompt user to press A
  display.clear();
  display.print(F("Press A"));
  
  // This line pauses the loop until button A is pressed and released
  buttonA.waitForButton();

  // 2. Wait 2 seconds
  display.clear();
  display.print(F("Wait..."));
  delay(2000);

  // 3. Calculate necessary PWM commands
  int left_pwm_command = 0;
  int right_pwm_command = 0;
  calculateMotorCommands(TARGET_LINEAR_V, TARGET_RADIAL_W, left_pwm_command, right_pwm_command);

  // Print to Serial for debugging
  Serial.print("Target V: "); Serial.print(TARGET_LINEAR_V);
  Serial.print(" | Target W: "); Serial.print(TARGET_RADIAL_W);
  Serial.print(" ===> Left PWM: "); Serial.print(left_pwm_command);
  Serial.print(" | Right PWM: "); Serial.println(right_pwm_command);

  // 4. Move motors for the specified duration
  display.clear();
  display.print(F("Moving"));
  
  motors.setSpeeds(left_pwm_command, right_pwm_command);
  delay(MOVE_DURATION_MS);

  // 5. Stop motors
  motors.setSpeeds(0, 0);
  
  display.clear();
  display.print(F("Done"));
  
  // Brief pause before restarting the loop and asking to press A again
  delay(1000); 
}
