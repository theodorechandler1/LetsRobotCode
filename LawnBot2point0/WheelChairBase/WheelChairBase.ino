  //Configurable Values
  static int defaultDriveSpeed = 128; //Maximum speed we will allow the wheelchair to reach (range of 1 to 255)
  static int defaultTurnSpeed = 255; //Maximum speed we will allow the wheelchair to turn (range of 1 to 255)
  static int driveTimeMilliseconds = 0.5 * 1000; //Time in milliseconds that the wheelchair will move without any input
  static bool debug = false;
  static int motorRampSpeed = 5; //PWM value to change by during acceleration
  static int motorFastRampMultiplier = 5; //Number of times to increase the motorRampSpeed during deceleration
  static int motorRampTimeMillis = 100; //Time between acceleration steps
  static int leftMotorSpeedOffset = 0; //TODO: Implement speed offset in order to maintain a straight drive
  static int rightMotorSpeedOffset = 0; //TODO: Implement speed offset in order to maintain a straight drive
  
  
  //Nonconfigurable Values
  //Driving variables
  int leftWheelSpeed = 0; //Currently set speed of left wheel
  int rightWheelSpeed = 0; //Currently set speed of right wheel
  int leftWheelDesiredSpeed = 0; //Desired speed of left wheel. Used with ramping
  int rightWheelDesiredSpeed = 0; //Desired speed of right wheel. Used with ramping
  String lastDirection = "stop"; //Used to compare against serial input to decide what to do next
  unsigned long stopTime = 0; //Timestamp at which we should disable the motors
  unsigned long nextMotorRampTime = 0; //Timestamp at which we last changed the motor speed. Used for ramping the motor speed
  //Data for serial events
  String inputString = ""; // A string to hold incoming serial data
  bool stringComplete = false; // Indicates whether the string is complete

  //Pin Definitions
  static int motorGlobalBrake = 2;
  static int leftReverseEnable = 5;
  static int leftForwardEnable = 6;
  static int rightReverseEnable = 9;
  static int rightForwardEnable = 10;
  static int rightForwardPWM = 7;
  static int rightReversePWM = 8;
  static int leftForwardPWM = 3;
  static int leftReversePWM = 4;
  static int leftForwardOvercurrent = 52;
  static int leftReverseOvercurrent = 53;
  static int rightForwardOvercurrent = 50;
  static int rightReverseOvercurrent = 51;


void setup() {
  //Setup motor controller outputs
  pinMode(motorGlobalBrake, OUTPUT);
  //Setup left motor inputs & outputs and default to off
  pinMode(leftReverseEnable,OUTPUT);
  digitalWrite(leftReverseEnable, LOW);
  pinMode(leftForwardEnable,OUTPUT);
  digitalWrite(leftForwardEnable, LOW);
  pinMode(leftForwardPWM,OUTPUT);
  digitalWrite(leftForwardPWM, LOW);
  pinMode(leftReversePWM,OUTPUT);
  digitalWrite(leftReversePWM, LOW);
  pinMode(leftForwardOvercurrent,INPUT);
  pinMode(leftReverseOvercurrent,INPUT);
  digitalWrite(leftForwardEnable, HIGH);
  digitalWrite(leftReverseEnable, HIGH);
  
  //Setup right motor inputs & outputs and default to off
  pinMode(rightReverseEnable,OUTPUT);
  digitalWrite(rightReverseEnable, LOW);
  pinMode(rightForwardEnable,OUTPUT);
  digitalWrite(rightForwardEnable, LOW);
  pinMode(rightForwardPWM,OUTPUT);
  digitalWrite(rightForwardPWM, LOW);
  pinMode(rightReversePWM,OUTPUT);
  digitalWrite(rightReversePWM, LOW);
  pinMode(rightForwardOvercurrent,INPUT);
  pinMode(rightReverseOvercurrent,INPUT);
  digitalWrite(rightForwardEnable, HIGH);
  digitalWrite(rightReverseEnable, HIGH);

  //Built in led setup. Used to indicate error state
  pinMode(LED_BUILTIN, OUTPUT);
  //Serial setup
  inputString.reserve(200); //Save some space in memory for the serial data
  Serial.begin(115200);
  Serial.println("Startup Complete");
  //disable the brakes now that we are all initialized
  digitalWrite(motorGlobalBrake, LOW);

}

void loop() {
  if(debug){Serial.println("Current speed: L:" + String(leftWheelSpeed) + " R:" + String(rightWheelSpeed) + " LD:" + String(leftWheelDesiredSpeed) + " RD:" + String(rightWheelDesiredSpeed));}
  //If we have exceeded the maximum amount of drivetime stop.
  if(millis() > stopTime){
    if(debug){Serial.println("Max time reached. Stopping");}
    leftWheelDesiredSpeed = 0;
    rightWheelDesiredSpeed = 0;
  }
  setSpeedMotor();
  if(stringComplete){
    handleSerialMessage();
  }
}


void handleSerialMessage(){
    if(debug){Serial.println(inputString);}
    String nextDirection = inputString;
    // clear the string:
    inputString = "";
    stringComplete = false;
    nextDirection.replace("\r",""); //Get rid of return characters in the string. In place operation
    nextDirection.replace("\n",""); //Get rid of newline character in the string. In place operation
    nextDirection.toLowerCase(); //Make the string lowercase. In place operation
    if(debug){Serial.println(nextDirection);}
    stopTime = driveTimeMilliseconds + millis(); //Calculate and store future stop time
    if(nextDirection == "f"){ //Drive forward
      leftWheelDesiredSpeed = defaultDriveSpeed;
      rightWheelDesiredSpeed = defaultDriveSpeed;
    }else if(nextDirection == "b"){ //Drive backward
      leftWheelDesiredSpeed = defaultDriveSpeed * -1;
      rightWheelDesiredSpeed = defaultDriveSpeed * -1;
    }else if(nextDirection == "l"){ //Turn left at half speed
      leftWheelDesiredSpeed = defaultTurnSpeed;
      rightWheelDesiredSpeed = defaultTurnSpeed  * -1;
    }else if(nextDirection == "r"){ //Turn right at half speed
      leftWheelDesiredSpeed = defaultTurnSpeed * -1;
      rightWheelDesiredSpeed = defaultTurnSpeed;
    }else if(nextDirection == "stop"){ //Stop
      leftWheelDesiredSpeed = 0;
      rightWheelDesiredSpeed = 0;
    }else{ //Received an unknown command. Stop and fail safe
      haltSafe();
    }
    lastDirection = nextDirection;
    if(debug){Serial.println("Setting Left Wheel Speed to: " + String(leftWheelDesiredSpeed));}
    if(debug){Serial.println("Setting Right Wheel Speed to: " + String(rightWheelDesiredSpeed));}
}

//Called everytime serial has another character available. Handles getting data for the serial port
void serialEvent() {
  while (Serial.available()) { 
    char inChar = (char)Serial.read(); //Get the new byte
    inputString += inChar; //Add the character to the inputString
    if (inChar == '\n') { //If the incoming character is a newline, set a flag so the main loop can do something about it.
      stringComplete = true;
    }
  }
}


void setSpeedMotor(){
  if(digitalRead(motorGlobalBrake) != LOW){ //Something has gone wrong and the motors are moving while locked. Fail safely
    haltSafe();
  }else{
    //setSpeedLeftMotor(leftWheelDesiredSpeed);
    //setSpeedRightMotor(rightWheelDesiredSpeed);
    if(millis() >= nextMotorRampTime){
      setSpeedLeftMotor(getNextSpeedValue(leftWheelSpeed,leftWheelDesiredSpeed,motorRampSpeed));
      setSpeedRightMotor(getNextSpeedValue(rightWheelSpeed,rightWheelDesiredSpeed,motorRampSpeed));
      //Add a delay until changing the motor ramp time 
      nextMotorRampTime = millis() + motorRampTimeMillis;
    }
  }
}
int getNextSpeedValue(int currentSpeed, int desiredSpeed, int motorRampSpeed){ //TODO Make ramping to 0 happen very quickly (stop happens fast)
  int localMotorRampSpeed = motorRampSpeed;
  if((currentSpeed > 0 and desiredSpeed <= 0) or (currentSpeed < 0 and desiredSpeed >= 0)){ //Ramp quickly to stop
    localMotorRampSpeed = motorRampSpeed * motorFastRampMultiplier;
  }else{ //Normal Ramping
    //Do nothing
  }
  //Old code
  if(currentSpeed < desiredSpeed){
    if(currentSpeed + localMotorRampSpeed > desiredSpeed){ //If we were to overshoot just limit it to the desired speed
        return desiredSpeed;
    }else{
      return currentSpeed + localMotorRampSpeed;
    }
  }else if(currentSpeed > desiredSpeed){
        if(currentSpeed - localMotorRampSpeed < desiredSpeed){ //If we were to overshoot just limit it to the desired speed
          return desiredSpeed;
        }else{
          return currentSpeed - localMotorRampSpeed;
        }
  }else{ //currentSpeed must be equal to desiredSpeed
    return currentSpeed;
  }
}

//Sets the left motor speed to the parameter passed in (range -255 to 255)
void setSpeedLeftMotor(int speed){
  if(speed > 0 and speed <= 255){ //Forward
    analogWrite(leftReversePWM,0);
    analogWrite(leftForwardPWM,speed);
  }else if(speed < 0 and speed >= -255){ //Reverse
    analogWrite(leftForwardPWM,0);
    analogWrite(leftReversePWM,abs(speed));
  }else{ //Stop
    analogWrite(leftReversePWM,0);
    analogWrite(leftForwardPWM,0);
  }
  leftWheelSpeed = speed;
  }

//Sets the right motor speed to the parameter passed in (range -255 to 255)
void setSpeedRightMotor(int speed){
  if(speed > 0 and speed <= 255){ //Forward
    analogWrite(rightReversePWM,0);
    analogWrite(rightForwardPWM,speed);
  }else if(speed < 0 and speed >= -255){ //Reverse
    analogWrite(rightForwardPWM,0);
    analogWrite(rightReversePWM,abs(speed));
  }else{ //Stop
    analogWrite(rightReversePWM,0);
    analogWrite(rightForwardPWM,0);
  }
  rightWheelSpeed = speed;
  }


bool hasOvercurrent(){
  if(digitalRead(leftForwardOvercurrent)){
    Serial.println("Left Forward Overcurrent");
    return true;
  }else if(digitalRead(rightForwardOvercurrent)){
    Serial.println("Right Forward Overcurrent");
    return true;
  }else if(digitalRead(leftReverseOvercurrent)){
    Serial.println("Left Reverse Overcurrent");
    return true;
  }else if(digitalRead(rightReverseOvercurrent)){
    Serial.println("Right Reverse Overcurrent");
    return true;
  }else{
    //No Overcurrent
    return false;
  }
}

//Fail safe mode. Will disable the motors and lock the wheels.
//Enters infinate loop and requires arduino restart to exit
void haltSafe(){
  Serial.println("Entered halt safe. Shutting off all motor outputs");
  digitalWrite(leftReverseEnable, LOW);
  digitalWrite(rightReverseEnable, LOW);
  digitalWrite(leftForwardEnable, LOW);
  digitalWrite(rightForwardEnable, LOW);
  digitalWrite(rightForwardPWM, LOW);
  digitalWrite(leftForwardPWM, LOW);
  digitalWrite(rightReversePWM, LOW);
  digitalWrite(leftReversePWM, LOW);
  digitalWrite(motorGlobalBrake, LOW);
  Serial.println("All motor pins set to zero");
  Serial.println("Entering inf loop");
  while(true){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    //do nothing and wait for reset
  }
}
