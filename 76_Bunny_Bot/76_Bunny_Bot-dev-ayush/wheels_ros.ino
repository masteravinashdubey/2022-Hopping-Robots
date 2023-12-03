#include <PWM.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

/* init node*/
ros::NodeHandle nh;

/* publish {theta, theta_dot} of motor angle */
std_msgs::Float64MultiArray encoderData;
ros::Publisher publish_data("encoder_state", &encoderData);

/* subscribe to lqr output of pwm (u = -kx) */
void pwm_callback(const std_msgs::Float64MultiArray &pwmData);
ros::Subscriber<std_msgs::Float64MultiArray> sub("pwmData", pwm_callback);

/* Pins for Right Wheel Motor*/
int brkRight = 4;
uint8_t pwmRight = 10;
int dirRight = 5;
int chARight = 2;
int chBRight = 6;
int rightDirection = 0;

/* Pins for Left Wheel Motor*/
int brkLeft = 7;
uint8_t pwmLeft = 9;
int dirLeft = 8;
int chALeft = 3;
int chBLeft = 9;
int leftDirection = 0;

volatile float leftCounter, rightCounter;
volatile float absLeftCounter, absRightCounter;

float angle, omega, absAngle;
float prevAngle = 0;
int cpr = 100;

float tempEncoder = 0;
float currentTime, prevTime, leftCurrentCount, rightCurrentCount, leftPrevCount = 0, rightPrevCount = 0;
double leftRPS, rightRPS;

volatile long encoderValue = 0;
int u1 = 0, u2 = 0;

float encoder_data[2];

/*************************************************************************************
 * Function Name: setup
 * Input  : no inputs.
 * Output : no outputs. 
 * Logic  : Init ros node, setup publisher to enocder_state and subcriber for pwmData
 *          Changing the frequency of pwm pins on arduino to 25kHz from default 490Hz
 *          using SetPinFrequencySafe function in PWM.h lib
 *          Set the gpio pin modes and attach interrupt to channel A pin for measuring 
 *          encoder data
 *          store current time and init length of encoder data to 2 {theta, omega}
 *************************************************************************************/ 
void setup()
{
    nh.initNode();
    nh.advertise(publish_data);
    nh.subscribe(sub);

    InitTimersSafe();
    SetPinFrequencySafe(pwmLeft, 25000);
    SetPinFrequencySafe(pwmRight, 25000);

    pinMode(brkRight, OUTPUT);
    pinMode(pwmRight, OUTPUT);
    pinMode(dirRight, OUTPUT);
    pinMode(chBRight, INPUT);

    pinMode(brkLeft, OUTPUT);
    pinMode(pwmLeft, OUTPUT);
    pinMode(dirLeft, OUTPUT);
    pinMode(chBLeft, INPUT);

    attachInterrupt(digitalPinToInterrupt(chARight), readEncoderRight, RISING);
    attachInterrupt(digitalPinToInterrupt(chALeft), readEncoderLeft, RISING);

    prevTime = millis();

    encoderData.data_length = 2;

    digitalWrite(brkLeft, HIGH);
    digitalWrite(brkRight, HIGH);

    Serial.begin(57600);
}

/*************************************************************************************
 * Function Name: pwm_callback
 * Input  : no inputs.
 * Output : no outputs. 
 * Logic  : callback function subscribing to topic pwmData updating the current pwm
 *          feedback in u1,u2 and calling driveMotors to actuate with reqd pwm
 *************************************************************************************/ 
void pwm_callback(const std_msgs::Float64MultiArray &pwmData)
{
    u1 = (int)pwmData.data[0]; // left
    u2 = (int)pwmData.data[1]; // right

    driveMotors();
}

/*************************************************************************************
 * Function Name: driveMotors
 * Input  : no inputs.
 * Output : no outputs. 
 * Logic  : applies the pwm feedback received from control node in the given direction 
 *          using pwmWrite function and digitalWrite for direcn
 *          For nidec bldc motors 255 pwm -> zero speed 0 pwm -> max speed
 *************************************************************************************/ 
void driveMotors()
{
    if (u1 < 0)
    {
        leftDirection = 1;
        // nh.loginfo("CCW");
    }
    else if (u1 > 0)
    {
        leftDirection = 0;
        // nh.loginfo("CW");
    }
    //  else if(u1 == 0){
    //    //nh.loginfo("breakleft");
    //    pinMode(brkLeft,LOW);
    //  }

    if (u2 < 0)
    {
        rightDirection = 1;
    }
    else if (u2 > 0)
    {
        rightDirection = 0;
    }
    //  else if(u2 == 0){
    //    //nh.loginfo("breakright");
    //    pinMode(brkRight,LOW);
    //  }

    if (abs(u1) > 255)
    {
        u1 = 255;
    }
    if (abs(u2) > 255)
    {
        u2 = 255;
    }

    digitalWrite(dirRight, rightDirection);
    digitalWrite(dirLeft, leftDirection);
    pwmWrite(pwmLeft, 255 - abs(u1));
    pwmWrite(pwmRight, 255 - abs(u2));

}

/*************************************************************************************
 * Function Name: loop
 * Input  : no inputs.
 * Output : no outputs. 
 * Logic  : calculates the net angular rotation of motors at a frequency of 150 Hz
 *          (T = 6.66 ms) and also calculate the angular velocity omega for each motor
 *          and takes average for both angle and omega
 *          publishes the encoder_data {angle, omega} to topic encoder_state
 *************************************************************************************/ 
void loop()
{
    leftCurrentCount = leftCounter;
    rightCurrentCount = rightCounter;

    currentTime = millis();

    if ((currentTime - prevTime) > 6.66)
    {
        leftRPS = (leftCurrentCount - leftPrevCount) * 1.501;
        rightRPS = (rightCurrentCount - rightCurrentCount) * 1.501;

        leftPrevCount = leftCurrentCount;
        rightPrevCount = rightCurrentCount;

        prevTime = currentTime;

        angle = (leftCurrentCount + rightCurrentCount) * 0.06283;
        omega = (leftRPS + rightRPS) * 0.5 * 6.28;
    }

    encoder_data[0] = angle;
    encoder_data[1] = omega;

    encoderData.data = encoder_data;

    publish_data.publish(&encoderData);

    nh.spinOnce();
}

/*************************************************************************************
 * Function Name: readEncoderRight, readEncoderLeft
 * Input  : no inputs.
 * Output : no outputs. updates global variables rightCounter and leftCounter 
 * Logic  : used to count encoder ticks in a partciular direction, 
 *          which is used to calculate the net angular rotation of motors
 *************************************************************************************/ 
void readEncoderRight()
{
    if (digitalRead(chBRight))
    {
        rightCounter--;
    }
    else
    {
        rightCounter++;
    }
    encoderValue++;
}

void readEncoderLeft()
{
    if (digitalRead(chBLeft))
    {
        leftCounter++;
    }
    else
    {
        leftCounter--;
    }
    encoderValue++;
}
