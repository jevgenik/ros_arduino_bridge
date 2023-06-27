/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#endif

// Initializes the motor controller and should be called once at the beginning of the program.
void initMotorController();

// Sets the speed of the motor
// i = 0 for left motor, 1 for right motor
// spd = -MAX_PWM to +MAX_PWM
void setMotorSpeed(int i, int spd); 

// Sets the speed of the left and right motors
// leftSpeed = -MAX_PWM to +MAX_PWM
// rightSpeed = -MAX_PWM to +MAX_PWM
void setMotorSpeeds(int leftSpeed, int rightSpeed);
