//PID-related
float previousTime = 0; //for calculating delta t
float previousError = 0; //for calculating the derivative (edot)
float errorIntegral = 0; //integral error
float currentTime = 0; //time in the moment of calculation
float deltaTime = 0; //time difference
float errorValue = 0; //error
float edot = 0; //derivative (de/dt)

//PID parameters - tuned by the user
float proportional = 0.1; //k_p = 0.5
float integral = 0.0000005; //k_i = 3
float derivative = 0.025; //k_d = 1
float controlSignal = 0; //u - Also called as process variable (PV)


//int target = 180;
float mulConst = 17401/360;
//float targetPosition = target*mulConst; //the PID will try to reach this value

const int PWMPin = 9; //this is an analog pin (with the tilde (~) symbol), this pin can also do higher frequency + independent from millis()'s timer
int PWMValue = 0; //0-255 PWM value for speed, external PWM boards can go higher (e.g. PCA9685: 12-bit => 0-4095)


const int directionPin1 = 5; //digital pin, output, sets the direction
const int directionPin2 = 6; //digital pin, output, sets the direction


int motorDirection = 0; //direction value 0: CCW, 1: CW. - Stored value

//Motor encoder
const int encoderPin1 = 2; //this pin is also the interrupt pin!
const int encoderPin2 = 3; //this pin is a normal pin, read upon the interrupt
int encoderPin2Value = 0; //value of the encoder pin (0 or 1), this pin is read inside the interrupt - used for direction determination

volatile float motorPosition = 0; //position based on the encoder

int target = 0 ;
int targetPrev = 0;
float targetPosition = target*mulConst; //the PID will try to reach this value

void setup(){
  pinMode(encoderPin1, INPUT); //A
  pinMode(encoderPin2, INPUT); //B

  pinMode(directionPin1,OUTPUT);
  pinMode(directionPin2,OUTPUT);
  pinMode(PWMPin,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), checkEncoder, RISING);

  Serial.begin(2000000);
}


void loop()
{
  while(Serial.available() == 0){
    target = Serial.parseInt();
    targetPosition = target*mulConst;
    calculatePID();
    //Serial.println("loopp");
  }
  //Serial.println(target);
  while(abs(errorValue) > 50){
    calculatePID();

    driveMotor();
  }
  digitalWrite(directionPin1, HIGH);
  digitalWrite(directionPin2, HIGH);
  //targetPrev = target;
  //Serial.println("Doneee");
  
  target = Serial.parseInt();
  calculatePID();

  //delay(1000);
  //Serial.println(errorValue);
  //Serial.println(motorPosition/mulConst);
  //printValues();
}

void checkEncoder()
{
  //We need to read the other pin of the encoder which will be either 1 or 0 depending on the direction
  encoderPin2Value = digitalRead(encoderPin2);

  if (encoderPin2Value == 1) //CW direction
  {
    motorPosition++;
  }
  else //else, it is zero... -> CCW direction
  {
    motorPosition--;
  }
}

void driveMotor()
{
  //Determine speed and direction based on the value of the control signal
  //direction
  //Serial.println("drive");
  if (controlSignal < 0) //negative value: CCW
  {
    motorDirection = 1;
  }
  else{
    motorDirection = -1;
  }
  //else if(controlSignal > 0) //positive: CW
 // {
 //   motorDirection = -1;
 // }
  //else //0: STOP - this might be a bad practice when you overshoot the setpoint
 //{
 //   motorDirection = 0;
// }
  //---------------------------------------------------------------------------
  //Speed
  PWMValue = (int)fabs(controlSignal); //PWM values cannot be negative and have to be integers
  if (PWMValue > 150) //fabs() = floating point absolute value
  {
    PWMValue = 150; //capping the PWM signal - 8 bit
  }

  if (PWMValue < 40 && errorValue != 0)
  {
    PWMValue = 40;
  }
  //A little explanation for the "bottom capping":
  //Under a certain PWM value, there won't be enough current flowing through the coils of the motor
  //Therefore, despite the fact that the PWM value is set to the "correct" value, the motor will not move
  //The above value is an empirical value, it depends on the motors perhaps, but 30 seems to work well in my case

  //we set the direction - this is a user-defined value, adjusted for TB6612FNG driver
  if (motorDirection == 1) //-1 == CCW
  {
    digitalWrite(directionPin1, LOW);
    digitalWrite(directionPin2, HIGH);
  }
  else if (motorDirection == -1) // == 1, CW
  {
    digitalWrite(directionPin1, HIGH);
    digitalWrite(directionPin2, LOW);
  }
  else // == 0, stop/break
  {
    digitalWrite(directionPin1, HIGH);
    digitalWrite(directionPin2, HIGH);
    PWMValue = 0;
    //In this block we also shut down the motor and set the PWM to zero
  }
  //----------------------------------------------------
  //Then we set the motor speed
  analogWrite(PWMPin, PWMValue);

  //Optional printing on the terminal to check what's up
    //Serial.println(motorPosition);
    Serial.print(errorValue);
    Serial.print(" ");
    Serial.print(PWMValue);
    Serial.print(" ");
    Serial.print(targetPosition);
    Serial.print(" ");
    Serial.println(motorPosition);
    Serial.println();
  
}

void calculatePID()
{
  //Determining the elapsed time
  currentTime = micros(); //current time
  deltaTime = (currentTime - previousTime) / 1000000.0; //time difference in seconds
  previousTime = currentTime; //save the current time for the next iteration to get the time difference
  //---
  errorValue = motorPosition - targetPosition; //Current position - target position (or setpoint)

  edot = (errorValue - previousError) / deltaTime; //edot = de/dt - derivative term

  errorIntegral = errorIntegral + (errorValue * deltaTime); //integral term - Newton-Leibniz, notice, this is a running sum!

  controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral); //final sum, proportional term also calculated here

  previousError = errorValue; //save the error for the next iteration to get the difference (for edot)
}
