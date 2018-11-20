'''
IMU data publisher:
take file name as input, publishes IMU data.
gyro --> angular_velocity
accel --> linear_acceleration
mag --> orientation
Note:
    run imu_puh.py before launch robot_pose_ekf in case topics may have name collision
'''

#!/usr/bin/env python
import csv
import sys
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler

DATA_FOLDER = ''
DATA_FOLDER1 = '../../../data/imu'
DATA_FOLDER2 = '../../../data/lab_run_ninja_imu_data'

def publish():
    '''
    read imu information from file
    '''
    # open imu file
    with open(DATA_FOLDER + '/accel.txt') as f_accel, \
        open(DATA_FOLDER + '/gyro.txt') as f_gyro, \
        open(DATA_FOLDER + '/mag.txt') as f_mag, \
        open(DATA_FOLDER + '/timestamp.txt') as f_time:
        # read the files as csv files
        accels = csv.reader(f_accel)
        gyros = csv.reader(f_gyro)
        mags = csv.reader(f_mag)
        timestamps = csv.reader(f_time)

        # initial the publihser
        pub = rospy.Publisher('imu_data', Imu, queue_size=10)
        # pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        rospy.init_node('imu_msg_publisher', anonymous=False)
        rate = rospy.Rate(20)
        # if we use rate=20, dt is enlarged, so we need to divide angular velocity
        if DATA_FOLDER == DATA_FOLDER1:
            divisor = 0.1
        else:
            divisor = 0.2

        # set covariance
        if DATA_FOLDER == DATA_FOLDER2:
            # set covariance to -1 indicate no measurement for orientation
            ori_cov = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
        else:
            ori_cov = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        ang_cov = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        lin_cov = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        # data id
        msg_id = 0

        activated = False

        # publish the data one by one
        for accel, gyro, mag, timestamp in zip(accels, gyros, mags, timestamps):
            # set time stamp
            stp = rospy.Time.from_sec(float(timestamp[0]))
            # set orientation value
            qut = quaternion_from_euler(float(mag[0]), float(mag[1]), float(mag[2]))
            ori = Quaternion(x=qut[0], y=qut[1], z=qut[2], w=qut[3])
            # set angular velocity
            ang = Vector3(x=float(gyro[0])*divisor, y=float(gyro[1])*divisor, z=float(gyro[2])*divisor)
            # set linear acceleration
            lin = Vector3(x=float(accel[0]), y=float(accel[1]), z=float(accel[2]))

            # activate imu sensor
            if not activated:
                # set the stamp smaller than current time, so that imu sensor can be activated
                time_header = Header(stamp=stp, frame_id='base_footprint')
                imu_msg = Imu(header=time_header, linear_acceleration=lin, \
                        angular_velocity=ang, orientation=ori,\
                        orientation_covariance=ori_cov, angular_velocity_covariance=ang_cov,\
                        linear_acceleration_covariance=lin_cov)
                pub.publish(imu_msg)
                activated = True

            # construct message
            time_header = Header(stamp=rospy.get_rostime(), frame_id='base_footprint')
            imu_msg = Imu(header=time_header, linear_acceleration=lin, \
                        angular_velocity=ang, orientation=ori,\
                        orientation_covariance=ori_cov, angular_velocity_covariance=ang_cov,\
                        linear_acceleration_covariance=lin_cov)

            msg_id += 1
            print 'published data ', msg_id

            # publish message
            if not rospy.is_shutdown():
                pub.publish(imu_msg)
                print imu_msg
                rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print 'Usage: imu_pub.py folder_name'
        sys.exit(0)
    else:
        if sys.argv[1] == 'imu':
            DATA_FOLDER = DATA_FOLDER1
        else:
            DATA_FOLDER = DATA_FOLDER2
    try:
        publish()
    except rospy.ROSInterruptException as err:
        print err.message
