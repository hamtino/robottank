#include <ros.h> // Timer.h seems to bugger it up, so nope.
#include "Kinematics.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#define ROS_SERIAL_BAUD_RATE 115200 // Rosserial baud rate 115200 bps for example

//// caracteristicas de las ruedas ////

#define MOTOR_MAX_RPM 90         // velocidad maxima rpm
#define WHEEL_DIAMETER 0.2       // diametro de las ruedas expresadas en metros
#define FR_WHEEL_DISTANCE 0.0    // distancia entre la rueda delantera y la rueda trasera (para 4WD)
#define LR_WHEEL_DISTANCE 0.25   // distancia entre rueda izquierda y rueda derecha
#define PWM_BITS 8               // resolución de pin PWM del microcontrolador. Arduino Uno / Mega Teensy está usando 8 bits (0-255)

Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);

//// ----------------------------- ////

////       lectura de voltaje      ////

#define BATTERY_READ_PIN 1 // pin de voltaje
#define BATTERY_VOLTAGE_PUBLISH_RATE 1000 // ms (publicar voltaje cada segundo)

// Battery Voltage Publisher
std_msgs::Float32 voltage_msg;
ros::Publisher battery_pub("voltaje_bateria", &voltage_msg);

void publishBatteryVoltage() {
  voltage_msg.data = getBatteryVoltage();
  battery_pub.publish( &voltage_msg );
}

unsigned long last_battery_published = 0;

//// ----------------------------- ////

////          Encoders             ////

#define ENCODER_LEFT_PIN_A 3
#define ENCODER_LEFT_PIN_B 4
#define ENCODER_RIGHT_PIN_A 2
#define ENCODER_RIGHT_PIN_B 11

#define TICKS_PER_REV 379 // ticks for one wheel revolution
#define TICKS_PER_METER 3110 // 790 ticks for 10 inches, 790 ticks per .254m ~ 3110.23622 ticks/meter
#define METERS_PER_REV 0.12186495176848874 // meters per full revolution
#define REV_PER_METER 8.20580474934037 // revolutions per meter

#define WHEEL_VEL_PUBLISH_RATE 100 // ms (10Hz)
#define MOTOR_PID_RATE 20 // ms (50Hz) PID Motor update rate (Should be < WHEEL_VEL_PUBLISH_RATE)
#define MOTOR_SPEED_CUTOFF 20 // lowest motor speed at which to zero it

// encoder izquierdo
long encoder_left_ticks = 0;
long encoder_left_ticks_old = 0;
int encoder_left_pin_A_last = LOW;
int encoder_left_val = LOW;
float encoder_left_velocity = 0;

// encoder derecho
long encoder_right_ticks = 0;
long encoder_right_ticks_old = 0;
int encoder_right_pin_A_last = LOW;
int encoder_right_val = LOW;
float encoder_right_velocity = 0;

unsigned long encoder_last_velocity_time = 0; // microseconds
long last_wheel_ticks_published = 0;

// Publicador encoder en ROS
std_msgs::Float32 wheel_vel;
ros::Publisher lwheel_pub("wheel_velocity/left", &wheel_vel);
ros::Publisher rwheel_pub("wheel_velocity/right", &wheel_vel);

// Publicar ticks de codificador de rueda izquierda y derecha
void publishWheelVelocity() {
 
  // Las velocidades del codificador están en rev/s, conviértalas a m/s
  wheel_vel.data = encoder_left_velocity * METERS_PER_REV;
  lwheel_pub.publish(&wheel_vel);

  wheel_vel.data = encoder_right_velocity * METERS_PER_REV;
  rwheel_pub.publish(&wheel_vel);
}

//// ----------------------------- ////

////             ROS               ////

ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;

// Devolución de llamada de velocidad de comando
void cmd_vel_callback( const geometry_msgs::Twist& twist_msg) {
  // mensajes en metros por segundo
  diff_drive(twist_msg.linear.x, twist_msg.angular.z);
}

// Suscriptor de Command Velocity
ros::Subscriber<geometry_msgs::Twist> cmd_vel_topic("cmd_vel", &cmd_vel_callback);

//// ----------------------------- ////


////////////////////////////////////////////////////////////////////////////////

void setup()
{  
  Serial.begin(9600);
  
  // Initialize Encoders
  pinMode (ENCODER_LEFT_PIN_A, INPUT);
  pinMode (ENCODER_LEFT_PIN_B, INPUT);
  pinMode (ENCODER_RIGHT_PIN_A, INPUT);
  pinMode (ENCODER_RIGHT_PIN_B, INPUT);

  // Attach Encoder Interrupts
  attachInterrupt(1, measureLeftEncoder, CHANGE); // 1 = pin 3 interrupt
  attachInterrupt(0, measureRightEncoder, CHANGE); // 0 = pin 2 interrupt

  
  // Initial speed is zero
  drive(0, 0);

  nh.getHardware()->setBaud(ROS_SERIAL_BAUD_RATE);
  nh.initNode();
  nh.advertise(battery_pub); // Battery Voltage Publisher

  // Encoder publishers
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);

  nh.subscribe(cmd_vel_topic); // Command Velocity Subscriber

  last_battery_published = millis();
}

void loop()
{  
  // Publish battery voltage every second
  if (millis() - last_battery_published > BATTERY_VOLTAGE_PUBLISH_RATE)
  {
    publishBatteryVoltage();
    last_battery_published = millis();
  }

  // Publish encoder at 100 Hz
  if (millis() - last_wheel_ticks_published > WHEEL_VEL_PUBLISH_RATE)
  {
    publishWheelVelocity();
    last_wheel_ticks_published = millis();
  }

  if (millis() - last_motor_pid_updated > MOTOR_PID_RATE)
  {
    updateMotorPIDs();
    last_motor_pid_updated = millis();
  }

  nh.spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
// Motor drive

// cmd_x  : linear x velocity (forward velocity) m/s
// cmd_th : angular z velocity (rotation of heading) rad/s

    Serial.print(" FRONT LEFT MOTOR: ");
    // Assuming you have an encoder for each wheel, you can pass this RPM value to a PID controller 
    // as a setpoint and your encoder data as a feedback.
    Serial.print(rpm.motor1);

    Serial.print(" FRONT RIGHT MOTOR: ");
    Serial.print(rpm.motor2);

    Serial.print(" REAR LEFT MOTOR: ");
    Serial.print(rpm.motor3);

    Serial.print(" REAR RIGHT MOTOR: ");
    Serial.println(rpm.motor4);

    delay(5000);

void diff_drive(double linear_vel_x, double angular_vel_z) {
    
  float linear_vel_y = 0;
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);

  // velocities in m/s, convert to rev/s
  // pid_left_setpoint = vl * 0;
  // pid_right_setpoint = vr * 0;
  // values used by PID loop
}

void drive(int lspeed, int rspeed)
{
  // otors.setLeftSpeed(lspeed);
  // motors.setRightSpeed(rspeed);
}

// Battery Voltage
float getBatteryVoltage() {
  return 5.0 * analogRead(BATTERY_READ_PIN) / 1024.0;
}

// Encoders
void measureLeftEncoder()
{
    encoder_left_val = digitalRead(ENCODER_LEFT_PIN_A);
    if ((encoder_left_pin_A_last == LOW) && (encoder_left_val == HIGH))
    {
        if (digitalRead(ENCODER_LEFT_PIN_B) == LOW) {encoder_left_ticks--;}
        else {encoder_left_ticks++;}
    }
    encoder_left_pin_A_last = encoder_left_val;
}

void measureRightEncoder()
{
    encoder_right_val = digitalRead(ENCODER_RIGHT_PIN_A);
    if ((encoder_right_pin_A_last == LOW) && (encoder_right_val == HIGH))
    {
        if (digitalRead(ENCODER_RIGHT_PIN_B) == LOW) {encoder_right_ticks--;} // Transistor Flipped? 
        else {encoder_right_ticks++;}
    }
    encoder_right_pin_A_last = encoder_right_val;
}


void updateWheelPositionVelocity()
{
  long dt = micros() - encoder_last_velocity_time; // ms

  // ticks offset
  long dleft = encoder_left_ticks - encoder_left_ticks_old;
  long dright = encoder_right_ticks - encoder_right_ticks_old;

  // Velocity in rev/second
  // 1e6 because dt is in microseconds
  encoder_left_velocity = (float)(1000000*dleft) / (TICKS_PER_REV * dt);
  encoder_right_velocity = (float)(1000000*dright) / (TICKS_PER_REV * dt);

  // Update PID inputs
  pid_left_input = (double)encoder_left_velocity;
  pid_right_input = (double)encoder_right_velocity;

  // Update previous values
  encoder_left_ticks_old = encoder_left_ticks;
  encoder_right_ticks_old = encoder_right_ticks;
  encoder_last_velocity_time = micros();
}

void updateMotorPIDs()
{
  // Update wheel velocities (pid inputs)
  // updateWheelPositionVelocity();

  // run pid controller, setting pid outputs
  // doLeftPID();
  // doRightPID();

  // If desired motor speed is less than threshold, zero it out
  // if (abs(pid_left_output) < MOTOR_SPEED_CUTOFF) { pid_left_output = 0; }
  // if (abs(pid_right_output) < MOTOR_SPEED_CUTOFF) { pid_right_output = 0; }

  // Set motor speeds
  // motors.setLeftSpeed(pid_left_output);
  // motors.setRightSpeed(pid_right_output);
}

// Init PIDs for zumo motors
void initPIDs()
{
  // Init setpoints in case they aren't yet set
  // pid_left_setpoint = 0;
  // pid_right_setpoint = 0;
}

// #define PID_OUTMIN -400
// #define PID_OUTMAX 400

// #define KP 20
// #define KI 10 // 500 * 20ms = 10
// #define KD 0

// Had to do PIDs unclassed manually since we're running out of memory.
// double left_ITerm = 0;
// double left_lastInput = 0;
void doLeftPID()
{
    /* Compute all the working error variables */
    // double error = pid_left_setpoint - pid_left_input;
    // left_ITerm += (KI * error);
    // if(left_ITerm > PID_OUTMAX) left_ITerm= PID_OUTMAX;
    // else if(left_ITerm < PID_OUTMIN) left_ITerm= PID_OUTMIN;
    // double dInput = (pid_left_input - left_lastInput);

    /* Compute PID Output */
    // pid_left_output = KP * error + left_ITerm- KD * dInput;
      
    // if(pid_left_output > PID_OUTMAX) pid_left_output = PID_OUTMAX;
    // else if(pid_left_output < PID_OUTMIN) pid_left_output = PID_OUTMIN;
  
    /* Remember some variables for next time */
    // left_lastInput = pid_left_input;
}

// double right_ITerm = 0;
// double right_lastInput = 0;
void doRightPID()
{
    /* Compute all the working error variables */
    // double error = pid_right_setpoint - pid_right_input;
    // right_ITerm += (KI * error);
    // if(right_ITerm > PID_OUTMAX) right_ITerm= PID_OUTMAX;
    // else if(right_ITerm < PID_OUTMIN) right_ITerm= PID_OUTMIN;
    // double dInput = (pid_right_input - right_lastInput);

    /* Compute PID Output */
    // pid_right_output = KP * error + right_ITerm- KD * dInput;
      
    // if(pid_right_output > PID_OUTMAX) pid_right_output = PID_OUTMAX;
    // else if(pid_right_output < PID_OUTMIN) pid_right_output = PID_OUTMIN;
  
    /* Remember some variables for next time */
    // right_lastInput = pid_right_input;
}
