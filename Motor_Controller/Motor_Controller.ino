////ESE - 505 Control of systems////
/// Final Project - Motor Speed Control//

////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES - Pin Assignments Only (Always Best to avoid global variables)
////////////////////////////////////////////////////////////////////////////////
//
// Pin Usage (Global to allow access in setup()
//
const int pinMOTOR = 3;   // Motor control with PWM on pin 3 (uses Timer 2)
const int pinRPM = A5;    // Analog Input for RPM Sensor
const int pinPOT = A0;    // Analog Input for Potentiometer = User Input
////////////////////////////////////////////////////////////////////////////////
// the setup routine runs once when you press reset:
////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(pinMOTOR, OUTPUT);     
  analogWrite(pinMOTOR, 0);    // stop the motor

  Serial.begin(9600);
  Serial.println("Motor Control Program Running");
  delay(2000); // Give reader a chance to see the output.
}

////////////////////////////////////////////////////////////////////////////////
// the loop routine runs over and over again until reset is pressed
////////////////////////////////////////////////////////////////////////////////
void loop() {
///////
// Declarations
///////
static long RunTime = 0;                   // Time at end of each iteration through the "loop"
word User_Raw_Input = 0;
word Sensor_Raw_Input = 0;
double User_Scaled_Input = 0.0;                   // Use floating point value to avoid type conversion issues coding
double Sensor_Scaled_Input = 0.0;
double Control_Command = 0.0;              // return double from control code to avoid type conversion issues
byte MotorDutyCycleByte = 0;               // AnalogWrite requires byte argument

//
// get potentiometer setting
//
  User_Raw_Input = getPot();
  User_Scaled_Input = 0.09775*User_Raw_Input;  // Convert 0-1023 into 0-100 percent

//
// get RPM measurements
// With motor voltage at 6.0 volts, max input is roughly 50% of full scale
// So, scale by factor of 2.0 to get range near 100%
//
  Sensor_Raw_Input = getRPM();
  Sensor_Scaled_Input = constrain(2.0*0.09775*Sensor_Raw_Input,0.0,100.0);

//
// call control function & constrain motor command to avoid errors on conversion to byte
//
  Control_Command = My_Control_Function(User_Scaled_Input, Sensor_Scaled_Input);
  Control_Command = constrain(Control_Command,0.0,99.9);
  Control_Command = 2.55*Control_Command;
  MotorDutyCycleByte = Control_Command;

//
// send command to motor
//
  analogWrite(pinMOTOR, MotorDutyCycleByte);

//
// Force Constant digital update rate by waiting for fixed time change
// Note: 40000 microseconds = 40 milliseconds = 0.04 seconds ==> 25 Hz Update Rate
//
  while ( (RunTime + 40000) > micros()) {
  }
  RunTime = micros();
//
// Echo current data
//
  Serial.print(RunTime, DEC); Serial.print("    ");
  Serial.print(User_Raw_Input, DEC); Serial.print("    ");
  Serial.print(Sensor_Raw_Input, DEC); Serial.print("    ");
  Serial.println(MotorDutyCycleByte, DEC);

////////////////////////////////////////////////////////////////////////////////
// end of loop
////////////////////////////////////////////////////////////////////////////////
}

////////////////////////////////////////////////////////////////////////////////
// Read potentiometer setting (returns percentage 0.0 to 100.0)
////////////////////////////////////////////////////////////////////////////////
word getPot() {
  word Pot_Value;
  Pot_Value = analogRead(pinPOT);
  return Pot_Value;
}

////////////////////////////////////////////////////////////////////////////////
// Read actual RPM
////////////////////////////////////////////////////////////////////////////////
word getRPM() {
  word RPM_Value;
  
  RPM_Value = analogRead(pinRPM);
  return RPM_Value;
}

////////////////////////////////////////////////////////////////////////////////
// THIS IS WHERE STUDENT CODE GOES
// OUTPUT MUST BE 0.0 <= u <= 100.0
////////////////////////////////////////////////////////////////////////////////
double My_Control_Function(double y_desired, double y_actual)
{
  double Kp = 1.2;   //Declaring Proportionality constant
  double Kd = 0.001; //Declaring Derivative constant
  double Ki = 1;      //Declaring integral constant
  double dt = 0.04;    //Declaring Time Delay
  static double integral = 0;   
  double u_command;
  double error;
  double derivative;
  static double previous_error = 0;
  double out_max = 254;   //Defining maximum bonds inorder to implement integral windup
  double out_min = 0;     //Defining minimum bonds inorder to implement integral windup
  double Kff = 0.03;      //Defining forward loop gain to implement forward feedback control
  
  error = y_desired - y_actual;  //Calculating error in the system
  
  integral += error*dt;       //Integrating all the error over a period of time
  
  // Checking the maximum and minimum bounds of the integral error to implement integral windup condition
  if(integral > out_max)        
    integral = out_max;
  else if(integral < out_min)
    integral = out_min;
    
  derivative = (error - previous_error)/dt;  //Calculating the derivative error in the system
  
  u_command = Kp*error + Kd*derivative + Ki*integral; //Implementing PID Control on the system
  
   // Checking the maximum and minimum bounds of the output to implement integral windup condition
  if(u_command > out_max)   
    u_command = out_max;
  else if(u_command < out_min)
    u_command = out_min; 
  
  previous_error = error;
  u_command += Kff * y_desired;
  
 
//
// this is how we create digital states in the code
// current value will be "old" value next time through this routine
//
  return u_command;
}
