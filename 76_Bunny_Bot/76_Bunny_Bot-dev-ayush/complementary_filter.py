import math
import time
import smbus
from ropsy import Float64MultiArray

class IMU:
    """
        1. Read from accelerometer and gyroscope of gy87 module and apply complemetary filter to get euler angles.
        2. ROS node to calculate state variables and publish [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot]    

        Attributes:
            ax, ay, az, gx, gy, gz: raw readings of accelerometer and gyroscope
            gyroXoffset, gyroYoffset, gyroZoffset: initial offset of gyro while calibration
            gyroRoll, gyroPitch, gyroYaw: angles obtained from gyro
            roll, pitch, yaw: angles to be published after applying complemetary filter
            dtTimer: timer variable used to calculate time difference between two iterations
            tau: parameter in complementary filter [0.98]
            gyroScaleFactor: sensitivity of the gyrsocope [250 deg/s]
            accScaleFactor: sensitivity of the accelerometer [2 g]
            imu_publisher: publisher of state variables to topic /gy_87/rpy_data, queue size 100
            address: i2c address of gy87 device [0x68]
    """
    def __init__(self, gyro, acc, tau):
        self.gx = None; self.gy = None; self.gz = None;
        self.ax = None; self.ay = None; self.az = None;

        self.gyroXoffset = 0
        self.gyroYoffset = 0
        self.gyroZoffset = 0
        
        self.gyroRoll = 0
        self.gyroPitch = 0
        self.gyroYaw = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.rollPrev = 0
        self.pitchPrev = 0
        self.yawPrev = 0

        self.dtTimer = 0
        self.tau = tau

        self.gyroScaleFactor, self.gyroHex = self.gyroSensitivity(gyro)
        self.accScaleFactor, self.accHex = self.accelerometerSensitivity(acc)

        self.imu_publisher = rospy.Publisher("/gy_87/rpy_data", Float64MultiArray, queue_size=100)

        self.bus = smbus.SMBus(1)
        self.address = 0x68

    def gyroSensitivity(self, x):
        """
            Setter function for gyroScaleFactor: create dictionary with standard value of 500 deg/s

            Args:
                x: desired gyroscope sensitivity

            Returns:
                corresponding gyroScaleFactor and gyroHex notation
        """
        return {
            250:  [131.0, 0x00],
            500:  [65.5,  0x08],
            1000: [32.8,  0x10],
            2000: [16.4,  0x18]
        }.get(x,  [65.5,  0x08])

    def accelerometerSensitivity(self, x):
        """
            Setter function for accScaleFactor: create dictionary with standard value of 4 g

            Args:
                x: desired accelerometer sensitivity

            Returns:
                corresponding accScaleFactor and accHex notation
        """
        return {
            2:  [16384.0, 0x00],
            4:  [8192.0,  0x08],
            8:  [4096.0,  0x10],
            16: [2048.0,  0x18]
        }.get(x,[8192.0,  0x08])

    def setUp(self):
        """
            Setup i2c communication with sensor, and configure accel and gyro sensitivity

            Args:
                none

            Returns:
                none
        """
        # Activate the MPU-6050
        self.bus.write_byte_data(self.address, 0x6B, 0x00)

        # Configure the accelerometer
        self.bus.write_byte_data(self.address, 0x1C, self.accHex)

        # Configure the gyro
        self.bus.write_byte_data(self.address, 0x1B, self.gyroHex)

        print("MPU set up:")
        print('\tAccelerometer: ' + str(self.accHex) + ' ' + str(self.accScaleFactor))
        print('\tGyro: ' + str(self.gyroHex) + ' ' + str(self.gyroScaleFactor) + "\n")
        time.sleep(2)

    def eightBit2sixteenBit(self, reg):
        """
            Utility function reading high and low 8 bit register values and convert into signed 16 bit values

            Args:
                none

            Returns:
                none
        """
        h = self.bus.read_byte_data(self.address, reg)
        l = self.bus.read_byte_data(self.address, reg+1)
        val = (h << 8) + l  # shifts into 16 bit

        # Make 16 bit unsigned value to signed value (0 to 65535) to (-32768 to +32767)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def getRawData(self):
        """
            Setter function for raw readings from accel and gyro

            Args:
                none

            Returns:
                none
        """
        self.gx = self.eightBit2sixteenBit(0x43)
        self.gy = self.eightBit2sixteenBit(0x45)
        self.gz = self.eightBit2sixteenBit(0x47)

        self.ax = self.eightBit2sixteenBit(0x3B)
        self.ay = self.eightBit2sixteenBit(0x3D)
        self.az = self.eightBit2sixteenBit(0x3F)
    
    def calibrateGyro(self, N):
        """
            Calibrate gyro to get initial offset along the three axes

            Args:
                N: number of readings to be taken while calibration

            Returns:
                none
        """
        print("Calibrating gyro with " + str(N) + " points. Do not move!")

        for ii in range(N):
            self.getRawData()
            self.gyroXoffset += self.gx
            self.gyroYoffset += self.gy
            self.gyroZoffset += self.gz

        # Find average offset value
        self.gyroXoffset /= N
        self.gyroYoffset /= N
        self.gyroZoffset /= N

        print("Calibration complete")
        print("\tX axis offset: " + str(round(self.gyroXoffset,1)))
        print("\tY axis offset: " + str(round(self.gyroYoffset,1)))
        print("\tZ axis offset: " + str(round(self.gyroZoffset,1)) + "\n")
        time.sleep(2) 
        self.dtTimer = time.time() # start timer for getting readings

    def processIMUvalues(self):
        """
            Process raw readings of accel and gyro by applying scaling factor and offset 

            Args:
                none

            Returns:
                none
        """
        self.getRawData()

        self.gx -= self.gyroXoffset
        self.gy -= self.gyroYoffset
        self.gz -= self.gyroZoffset

        # Convert to instantaneous degrees per second
        self.gx /= self.gyroScaleFactor
        self.gy /= self.gyroScaleFactor
        self.gz /= self.gyroScaleFactor

        # Convert to g force
        self.ax /= self.accScaleFactor
        self.ay /= self.accScaleFactor
        self.az /= self.accScaleFactor

    def compFilter(self):
        """
            1. Calculate euler angles from accelerometer and gyroscope
            2. Apply complementary filter to roll and pitch values of accel and gyro
            3. Calculate derivatives of euler angles

            comp_angle[i] = tau* (comp_angle[i] + gyro_angle[i]) + (1 - tau)* accel_angle[i]

            Args:
                none

            Returns:
                [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot]
        """
        self.processIMUvalues()
        roll_dot, pitch_dot, yaw_dot = 0.0, 0.0, 0.0

        # Get delta time and record time for next call
        dt = time.time() - self.dtTimer
        self.dtTimer = time.time()

        # Acceleration vector angle
        accPitch = math.degrees(math.atan2(self.ay, self.az))
        accRoll = math.degrees(math.atan2(self.ax, self.az))

        # Integrating gyro velcoity readings to get euler angles
        self.gyroRoll = self.gyroRoll - self.gy * dt
        self.gyroPitch = self.gyroPitch + self.gx * dt
        self.gyroYaw = self.gyroYaw + self.gz * dt

        self.yaw = self.gyroYaw
        self.roll = (self.tau)*(self.roll - self.gy*dt) + (1-self.tau)*(accRoll)
        self.pitch = (self.tau)*(self.pitch + self.gx*dt) + (1-self.tau)*(accPitch)

        roll_dot = (self.roll - self.rollPrev)/dt
        pitch_dot = (self.pitch - self.pitchPrev)/dt
        yaw_dot = (self.yaw - self.yawPrev)/dt

        self.rollPrev = self.roll
        self.pitchPrev = self.pitch
        self.yawPrev = self.yaw

        return [self.roll, self.pitch, self.yaw, roll_dot, pitch_dot, yaw_dot]* 0.017453
        

    def publish_imu(self):
        """
            Publish [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot] as Float64MutiArray to topic /gy_87/rpy_data

            Args:
                none

            Returns:
                none
        """
        imu_state = Float64MutiArray()
        imu_state.data = self.compFilter()

        self.imu_publisher.publish(imu_state)

def main():
    """
        1. Setup IMU class with required accel and gyro scale factor and calibrate gyro
        2. Init ros node gy_87 which uses rosTimer to publish state variables at frequency of 150 Hz

        Args:
            none

        Returns:
            none
    """
    rospy.init_node("gy_87")
    print("ROS Node Init")

    gyro, acc, tau = 250, 2, 0.98    # 250, 500, 1000, 2000 [deg/s] 2, 4, 7, 16 [g]
    imu = IMU(gyro, acc, tau)

    imu.setUp()
    imu.calibrateGyro(500)
    
    rospy.Timer(rospy.Duration(1.0/150.0), imu.publish_imu)
    
    rospy.spin()

# Main loop
if __name__ == '__main__':
    main()
