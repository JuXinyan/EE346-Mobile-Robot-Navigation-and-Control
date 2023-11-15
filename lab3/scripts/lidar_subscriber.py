#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import statistics

distance_data = []
distance0 = 0.47 # 单位为米


def lidar_callback(data):
    # 处理LiDAR数据
    # 您可以访问LiDAR数据，如data.ranges，data.intensities等

    # Minimum and maximum ranges
    ranges = data.ranges

    # Task1: Calculate and print the minimum and maximum ranges
    min_range = min(ranges)
    max_range = max(ranges)
    # print(f"Minimum Range: {min_range}")
    # print(f"Maximum Range: {max_range}")

    # Store the distanec in 90 degree into a list
    distance = data.ranges[88:92]
    # print(f"Distance in 90 degree:{distance}")
    distance_data.extend(distance)

    # Task2: Distance accuracy
    measured_mean = sum(distance_data) / len(distance_data)
    accuracy = abs((measured_mean - distance0) / distance0) * 100
    print(f"Lenth of data:{len(distance_data)}, Accuracy: {accuracy:.2f}%")
        
    # Task3: Distance precision
    if len(distance_data) >= 2:
        errors = [abs(distance - distance0) for distance in distance_data] 
        precision = statistics.stdev(errors)
        print(f"Lenth of data:{len(distance_data)}, Precision: {precision:.7f}")

    with open('output.txt', 'w') as file:
        for item in distance_data:
            file.write(str(item) + '\n')

if __name__ == '__main__':
    rospy.init_node('lidar_subscriber', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()



