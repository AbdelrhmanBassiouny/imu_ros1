#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import serial
import string
import math
import sys

#from time import time
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

def imu_update( gx,  gy,  gz,  ax,  ay,  az, quat, SamplePeriod=1/50.0, Beta=0.1):
    print(gx, gy, gz, ax, ay, az)
    Quaternion = quat
    q1 = Quaternion[0]
    q2 = Quaternion[1]
    q3 = Quaternion[2]
    q4 = Quaternion[3]   # short name local variable for readability
    norm = 0.0
    s1 = 0.0
    s2 = 0.0
    s3 = 0.0
    s4 = 0.0
    qDot1 = 0.0
    qDot2 = 0.0
    qDot3 = 0.0 
    qDot4 = 0.0

    # Auxiliary variables to avoid repeated arithmetic
    _2q1 = 2 * q1
    _2q2 = 2 * q2
    _2q3 = 2 * q3
    _2q4 = 2 * q4
    _4q1 = 4 * q1
    _4q2 = 4 * q2
    _4q3 = 4 * q3
    _8q2 = 8 * q2
    _8q3 = 8 * q3
    q1q1 = q1 * q1
    q2q2 = q2 * q2
    q3q3 = q3 * q3
    q4q4 = q4 * q4

    # Normalise accelerometer measurement
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if (norm == 0.0): return # handle NaN
    norm = 1 / norm          # use reciprocal for division
    ax *= norm
    ay *= norm
    az *= norm

    # Gradient decent algorithm corrective step
    s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
    s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
    s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
    s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay
    norm = 1 / math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
    s1 *= norm
    s2 *= norm
    s3 *= norm
    s4 *= norm

    # Compute rate of change of quaternion
    qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1
    qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2
    qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3
    qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4

    # Integrate to yield quaternion
    q1 += qDot1 * SamplePeriod
    q2 += qDot2 * SamplePeriod
    q3 += qDot3 * SamplePeriod
    q4 += qDot4 * SamplePeriod
    norm = 1 / math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    # normalise quaternion
    Quaternion[0] = q1 * norm
    Quaternion[1] = q2 * norm
    Quaternion[2] = q3 * norm
    Quaternion[3] = q4 * norm
    
    return Quaternion


def imu_update_with_mag(gx,  gy,  gz,  ax,  ay,  az,  mx,  my,  mz, quat, SamplePeriod=1/50.0, Beta=0.1):
    Quaternion = quat
    q1 = Quaternion[0]
    q2 = Quaternion[1]
    q3 = Quaternion[2]
    q4 = Quaternion[3]   # short name local variable for readability
    norm = 0.0
    hx = 0.0
    hy = 0.0
    _2bx = 0.0
    _2bz = 0.0
    s1 = 0.0
    s2 = 0.0
    s3 = 0.0
    s4 = 0.0
    qDot1 = 0.0
    qDot2 = 0.0
    qDot3 = 0.0
    qDot4 = 0.0

    # Auxiliary variables to avoid repeated arithmetic
    _2q1mx = 0.0
    _2q1my = 0.0
    _2q1mz = 0.0
    _2q2mx = 0.0
    _4bx = 0.0
    _4bz = 0.0
    _2q1 = 2.0 * q1
    _2q2 = 2.0 * q2
    _2q3 = 2.0 * q3
    _2q4 = 2.0 * q4
    _2q1q3 = 2.0 * q1 * q3
    _2q3q4 = 2.0 * q3 * q4
    q1q1 = q1 * q1
    q1q2 = q1 * q2
    q1q3 = q1 * q3
    q1q4 = q1 * q4
    q2q2 = q2 * q2
    q2q3 = q2 * q3
    q2q4 = q2 * q4
    q3q3 = q3 * q3
    q3q4 = q3 * q4
    q4q4 = q4 * q4

    # Normalise accelerometer measurement
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if (norm == 0.0): return # handle NaN
    norm = 1 / norm        # use reciprocal for division
    ax *= norm
    ay *= norm
    az *= norm

    # Normalise magnetometer measurement
    norm = math.sqrt(mx * mx + my * my + mz * mz)
    if (norm == 0.0): return # handle NaN
    norm = 1 / norm        # use reciprocal for division
    mx *= norm
    my *= norm
    mz *= norm

    # Reference direction of Earth's magnetic field
    _2q1mx = 2.0 * q1 * mx
    _2q1my = 2.0 * q1 * my
    _2q1mz = 2.0 * q1 * mz
    _2q2mx = 2.0 * q2 * mx
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
    _2bx = math.sqrt(hx * hx + hy * hy)
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
    _4bx = 2.0 * _2bx
    _4bz = 2.0 * _2bz

    # Gradient decent algorithm corrective step
    s1 = -_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    s2 = _2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    s3 = -_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    s4 = _2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
    norm = 1 / math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
    s1 *= norm
    s2 *= norm
    s3 *= norm
    s4 *= norm

    # Compute rate of change of quaternion
    qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1
    qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2
    qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3
    qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4

    # Integrate to yield quaternion
    q1 += qDot1 * SamplePeriod
    q2 += qDot2 * SamplePeriod
    q3 += qDot3 * SamplePeriod
    q4 += qDot4 * SamplePeriod
    norm = 1 / math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
    Quaternion[0] = q1 * norm
    Quaternion[1] = q2 * norm
    Quaternion[2] = q3 * norm
    Quaternion[3] = q4 * norm
    return Quaternion
    

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config

rospy.init_node("razor_node")

imuMsg = Imu()

# Orientation covariance estimation:
# Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
# Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
# Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
# cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
# i.e. variance in yaw: 0.0025
# Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
# static roll/pitch error of 0.8%, owing to gravity orientation sensing
# error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
# so set all covariances the same.
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

# Angular velocity covariance estimation:
# Observed gyro noise: 4 counts => 0.28 degrees/sec
# nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
# Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
# observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
# nonliniarity spec: 0.5% of full scale => 0.2m/s^2
# Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]

# read basic information
port = rospy.get_param('~port', '/dev/ttyUSB0')
topic = rospy.get_param('~topic', 'imu')
frame_id = rospy.get_param('~frame_id', 'base_imu_link')

# read calibration parameters

# accelerometer
accel_x_min = rospy.get_param('~accel_x_min', -250.0)
accel_x_max = rospy.get_param('~accel_x_max', 250.0)
accel_y_min = rospy.get_param('~accel_y_min', -250.0)
accel_y_max = rospy.get_param('~accel_y_max', 250.0)
accel_z_min = rospy.get_param('~accel_z_min', -250.0)
accel_z_max = rospy.get_param('~accel_z_max', 250.0)

# magnetometer
magn_x_min = rospy.get_param('~magn_x_min', -600.0)
magn_x_max = rospy.get_param('~magn_x_max', 600.0)
magn_y_min = rospy.get_param('~magn_y_min', -600.0)
magn_y_max = rospy.get_param('~magn_y_max', 600.0)
magn_z_min = rospy.get_param('~magn_z_min', -600.0)
magn_z_max = rospy.get_param('~magn_z_max', 600.0)
calibration_magn_use_extended = rospy.get_param('~calibration_magn_use_extended', False)
magn_ellipsoid_center = rospy.get_param('~magn_ellipsoid_center', [0, 0, 0])
magn_ellipsoid_transform = rospy.get_param('~magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]])
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# gyroscope
gyro_average_offset_x = rospy.get_param('~gyro_average_offset_x', 0.0)
gyro_average_offset_y = rospy.get_param('~gyro_average_offset_y', 0.0)
gyro_average_offset_z = rospy.get_param('~gyro_average_offset_z', 0.0)

#rospy.loginfo("%f %f %f %f %f %f", accel_x_min, accel_x_max, accel_y_min, accel_y_max, accel_z_min, accel_z_max)
#rospy.loginfo("%f %f %f %f %f %f", magn_x_min, magn_x_max, magn_y_min, magn_y_max, magn_z_min, magn_z_max)
#rospy.loginfo("%s %s %s", str(calibration_magn_use_extended), str(magn_ellipsoid_center), str(magn_ellipsoid_transform[0][0]))
#rospy.loginfo("%f %f %f", gyro_average_offset_x, gyro_average_offset_y, gyro_average_offset_z)

pub = rospy.Publisher(topic, Imu, queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time()

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=57600, timeout=1)
    #ser = serial.Serial(port=port, baudrate=57600, timeout=1, rtscts=True, dsrdtr=True) # For compatibility with some virtual serial ports (e.g. created by socat) in Python 2.7
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(2)

roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

### configure board ###
#stop datastream
ser.write(('#o0').encode("utf-8"))

#discard old input
#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working
discard = ser.readlines() 

#set output mode
ser.write(('#ox').encode("utf-8")) # To start display angle and sensor reading in text

rospy.loginfo("Writing calibration values to razor IMU board...")
#set calibration values
ser.write(('#caxm' + str(accel_x_min)).encode("utf-8"))
ser.write(('#caxM' + str(accel_x_max)).encode("utf-8"))
ser.write(('#caym' + str(accel_y_min)).encode("utf-8"))
ser.write(('#cayM' + str(accel_y_max)).encode("utf-8"))
ser.write(('#cazm' + str(accel_z_min)).encode("utf-8"))
ser.write(('#cazM' + str(accel_z_max)).encode("utf-8"))

if (not calibration_magn_use_extended):
    ser.write(('#cmxm' + str(magn_x_min)).encode("utf-8"))
    ser.write(('#cmxM' + str(magn_x_max)).encode("utf-8"))
    ser.write(('#cmym' + str(magn_y_min)).encode("utf-8"))
    ser.write(('#cmyM' + str(magn_y_max)).encode("utf-8"))
    ser.write(('#cmzm' + str(magn_z_min)).encode("utf-8"))
    ser.write(('#cmzM' + str(magn_z_max)).encode("utf-8"))
else:
    ser.write(('#ccx' + str(magn_ellipsoid_center[0])).encode("utf-8"))
    ser.write(('#ccy' + str(magn_ellipsoid_center[1])).encode("utf-8"))
    ser.write(('#ccz' + str(magn_ellipsoid_center[2])).encode("utf-8"))
    ser.write(('#ctxX' + str(magn_ellipsoid_transform[0][0])).encode("utf-8"))
    ser.write(('#ctxY' + str(magn_ellipsoid_transform[0][1])).encode("utf-8"))
    ser.write(('#ctxZ' + str(magn_ellipsoid_transform[0][2])).encode("utf-8"))
    ser.write(('#ctyX' + str(magn_ellipsoid_transform[1][0])).encode("utf-8"))
    ser.write(('#ctyY' + str(magn_ellipsoid_transform[1][1])).encode("utf-8"))
    ser.write(('#ctyZ' + str(magn_ellipsoid_transform[1][2])).encode("utf-8"))
    ser.write(('#ctzX' + str(magn_ellipsoid_transform[2][0])).encode("utf-8"))
    ser.write(('#ctzY' + str(magn_ellipsoid_transform[2][1])).encode("utf-8"))
    ser.write(('#ctzZ' + str(magn_ellipsoid_transform[2][2])).encode("utf-8"))

ser.write(('#cgx' + str(gyro_average_offset_x)).encode("utf-8"))
ser.write(('#cgy' + str(gyro_average_offset_y)).encode("utf-8"))
ser.write(('#cgz' + str(gyro_average_offset_z)).encode("utf-8"))

#print calibration values for verification by user
ser.flushInput()
ser.write(('#p').encode("utf-8"))
calib_data = ser.readlines()
calib_data_print = "Printing set calibration values:\r\n"
for row in calib_data:
    line = bytearray(row).decode("utf-8")
    calib_data_print += line
rospy.loginfo(calib_data_print)

#start datastream
ser.write(('#o1').encode("utf-8"))

#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working - it breaks the serial connection
rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = bytearray(ser.readline()).decode("utf-8")
rospy.loginfo("Publishing IMU data...")
#f = open("raw_imu_data.log", 'w')

errcount = 0
q = [1.0, 0.0, 0.0, 0.0]
while not rospy.is_shutdown():
    if (errcount > 10):
        break
    line = bytearray(ser.readline()).decode("utf-8")
    if ((line.find("#YPRAG=") == -1) or (line.find("\r\n") == -1)): 
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    #f.write(line)                     # Write to the output log file
    line = line.replace("\r\n","")   # Delete "\r\n"
    words = line.split(",")    # Fields split
    if len(words) != 9:
        rospy.logwarn("Bad IMU data or bad sync")
        errcount = errcount+1
        continue
    else:
        errcount = 0
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        yaw_deg = -float(words[0])
        yaw_deg = yaw_deg + imu_yaw_calibration
        if yaw_deg > 180.0:
            yaw_deg = yaw_deg - 360.0
        if yaw_deg < -180.0:
            yaw_deg = yaw_deg + 360.0
        yaw = yaw_deg*degrees2rad
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        pitch = -float(words[1])*degrees2rad
        roll = float(words[2])*degrees2rad

        # Publish message
        # AHRS firmware accelerations are negated
        # This means y and z are correct for ROS, but x needs reversing
        imuMsg.linear_acceleration.x = -float(words[3]) * accel_factor
        imuMsg.linear_acceleration.y = float(words[4]) * accel_factor
        imuMsg.linear_acceleration.z = float(words[5]) * accel_factor

        imuMsg.angular_velocity.x = float(words[6])
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        imuMsg.angular_velocity.y = -float(words[7])
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
        imuMsg.angular_velocity.z = -float(words[8])

    #q = quaternion_from_euler(roll,pitch,yaw) #CHANGE: uncomment this and comment below 5 lines of code
    q = imu_update(imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z,
                   imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z, quat=q)
    # q = imu_update_with_mag(imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z,
    #                imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z, roll, pitch, yaw, quat=q)
    q = [q[1], q[2], q[3], q[0]]
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = frame_id
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)
    q = [q[3], q[0], q[1], q[2]] #CHANGE: comment this as well
    if (diag_pub_time < rospy.get_time()) :
        diag_pub_time += 1
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = rospy.get_rostime()
        diag_arr.header.frame_id = '1'
        diag_msg = DiagnosticStatus()
        diag_msg.name = 'Razor_Imu'
        diag_msg.level = DiagnosticStatus.OK
        diag_msg.message = 'Received AHRS measurement'
        diag_msg.values.append(KeyValue('roll (deg)',
                                str(roll*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('pitch (deg)',
                                str(pitch*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('yaw (deg)',
                                str(yaw*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('sequence number', str(seq)))
        diag_arr.status.append(diag_msg)
        diag_pub.publish(diag_arr)
        
ser.close

#f.close

if (errcount > 10):
    sys.exit(10)
