#include "GY87.h"
#include <string>
#include <vector>
#include <cmath>
#include <unistd.h>

// Constructor
GY87::GY87(ros::NodeHandle nh)
{
	// Init publishers
	ROS_INFO("GY87: Init Publishers...");
	pub_gy87_ = nh.advertise<gy_87::rpy>("rpy_data", 10);

	// Init I2C
	ROS_INFO("GY87: Init I2C...");
	I2Cdev::initialize();

	// Init sensors
	ROS_INFO("GY87: Init sensors...");
	mpu_ = MPU6050(0x68);
	/*
	* This function also sets both the accelerometer and the gyroscope
 	* to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 	* the clock source to use the X Gyro for reference, which is slightly better than
 	* the default internal clock source
	*/
	mpu_.initialize();

	// Enable I2C bypass to be able to connect to magnet sensor
	// (connected to auxiliary I2C of MPU)
//	mpu_.setI2CMasterModeEnabled(false);
	mpu_.setI2CBypassEnabled(true);
//	mpu_.setSleepEnabled(false);

	hmc_ = HMC5883L(0x1E);
	hmc_.initialize();
	
	offset_accel[3] = {0};
	offset_gyro[3] = {0};
	dt = 0.01;
}

// Destructor
GY87::~GY87() {}

void GY87::publish()
{
	mpu_.getMotion6(&axi, &ayi, &azi, &gxi, &gyi, &gzi);
	hmc_.getHeading(&mxi, &myi, &mzi);
	complementary_filter();

	rpy_msg_.roll.data = complementary_angle[0];
	rpy_msg_.pitch.data = complementary_angle[1];
	rpy_msg_.yaw.data = complementary_angle[2];
	pub_gy87_.publish(rpy_msg_);
}

void GY87::calibrate()
{
	float ag[6] = {0,0,0,0,0,0}; // 3*acc, 3*gyro
	
  	for(int i = 0; i < CALIB_OFFSET_NB_MES; i++){
		
		mpu_.getMotion6(&axi, &ayi, &azi, &gxi, &gyi, &gzi);
		hmc_.getHeading(&mxi, &myi, &mzi);
		ag[0] += axi;
		ag[1] += ayi;
		ag[2] += azi;
		ag[3] += gxi;
		ag[4] += gyi;
		ag[5] += gzi;
		sleep(0.1); // wait a little bit between 2 measurements
  	}

	for(int i = 0; i < 6; i++){
		if(i<3){
			offset_accel[i] = ag[i] / CALIB_OFFSET_NB_MES;
		}
		else {
			offset_gyro[i] = ag[i] / CALIB_OFFSET_NB_MES;
		}

		ROS_INFO("Calibrated Accelerometer and Gyroscope");
	}
}

void GY87::compute_acc_rp(float *_accel_angle)
{
	_accel_angle[0] = atan2(ayi, sqrt(pow(axi, 2) + pow(azi, 2))) * RAD_TO_DEG;
	_accel_angle[1] = atan2(-axi, sqrt(pow(ayi, 2) + pow(azi, 2))) * RAD_TO_DEG;
	_accel_angle[2] = 0;
}

void GY87::compute_gyro_rpy(float *gyro_angle)
{
	// (1 / 131) sensitivity factor of Gyroscope: 1 degree rotation gives a reading of 131 units

	gyro_angle[0] = (gxi * dt)/ 131;
 	gyro_angle[1] = (gyi * dt)/ 131;
 	gyro_angle[2] = (gzi * dt)/ 131;

 	/*
 		In case the roll angle varies widely when only the pitch angle changes, activate the following equations.
 		gyro_angle[0] = dt * (gx + gy * sin(gyro_angle[0]) * tan(gyro_angle[1]) + gz * cos(gyro_angle[0]) * tan(gyro_angle[1]));
 		gyro_angle[1] = dt * (gy * cos(gyro_angle[0]) - gz * sin(gyro_angle[0]));
 	*/
}

void GY87::compute_mag_y(float *_mag_angle)
{
	*_mag_angle = (atan2(-myi,mxi)) * RAD_TO_DEG;
}

void GY87::complementary_filter()
{
	float _gyro_angle[3], _accel_angle[3], _mag_angle_yaw;
	
	compute_acc_rp(_accel_angle);
	compute_gyro_rpy(_gyro_angle);
	compute_mag_y(&_mag_angle_yaw);
	
	if (_first_run)
 	{
		// Set the gyroscope angle reference point if this is the first function run
 		for (int i = 0; i <= 1; i++)
 		{
 			_gyro_angle[i] = _accel_angle[i]; // Start off with angle from accelerometer (absolute angle since gyroscope is relative)
 		}
 		_gyro_angle[2] = 0; // Set the yaw axis to zero (because the angle cannot be calculated with the accelerometer when vertical)
 		_first_run = 0;
		return;
 	}
	
	
// 	float asum = abs(*ax) + abs(*ay) + abs(*az); // Calculate the sum of the accelerations
// 	float gsum = abs(*gx) + abs(*gy) + abs(*gz); // Calculate the sum of the gyro readings

 	for (int i = 0; i <= 1; i++)
 	{ // Loop through roll and pitch axes
// 		// if (abs(_gyro_angle[i] - _accel_angle[i]) > 5)
// 		// { // Correct for very large drift (or incorrect measurment of gyroscope by longer loop time)
// 		// 	_gyro_angle[i] = _accel_angle[i];
// 		// }

// 		// Create result from either complementary filter or directly from gyroscope or accelerometer depending on conditions
// 		// if (asum > 0.1 && asum < 3 && gsum > 0.3)
// 		// {																					   // Check that th movement is not very high (therefore providing inacurate angles)
 			complementary_angle[i] = TAU * (complementary_angle[i] + _gyro_angle[i]) + (1 - TAU) * (_accel_angle[i]); // Calculate the angle using a complementary filter
// 		// }
// 		// else if (gsum > 0.3)
// 		// { // Use the gyroscope angle if the acceleration is high
// 		// 	complementary_angle[i] = _gyro_angle[i];
// 		// }
// 		// else if (gsum <= 0.3)
// 		// { // Use accelerometer angle if not much movement
// 		// 	complementary_angle[i] = _accel_angle[i];
// 		// }
 	}

 	complementary_angle[2] = TAU* _gyro_angle[2] + (1 - TAU)* _mag_angle_yaw;
}

