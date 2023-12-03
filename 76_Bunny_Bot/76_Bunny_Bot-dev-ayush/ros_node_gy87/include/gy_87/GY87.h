// ROS libraries
#include "ros/ros.h"
#include "gy_87/rpy.h"

// Sensor libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#include <time.h>

#define RAD_TO_DEG 57.2957795130
#define TAU 0.05
#define CALIB_OFFSET_NB_MES 500

class GY87 {
    private:
        ros::Publisher pub_gy87_;

        MPU6050 mpu_;
        HMC5883L hmc_;

        int16_t axi, ayi, azi, gxi, gyi, gzi, mxi, myi, mzi;
	float offset_accel[3], offset_gyro[3];
        gy_87::rpy rpy_msg_;

	bool _first_run = 1;
	
	float dt;
	float complementary_angle[3];
	
    public:
        GY87(ros::NodeHandle nh);
        ~GY87();
        void publish();
	void calibrate();
        void compute_acc_rp(float *_accel_angle);
        void compute_gyro_rpy(float *gyro_angle);
        void compute_mag_y(float *_mag_angle);
        void complementary_filter();
};

