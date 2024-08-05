#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from pynput import keyboard
import os

# Define the initial servo angles
servo_angles = [165.0, 10.0, 90.0, 170.0]

# Define publishers for each servo
pub1 = None
pub2 = None
pub3 = None
pub4 = None


# Create publishers
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Create Twist and String messages
twist = Twist()

def print_menu():
    os.system('clear')
    print("\nReading from the keyboard and Publishing to Twist!")
    print("---------------------------")
    print("Moving around:")
    print("    w    ")
    print("a   s   d")
    print("\nMoving the servos:")
    print("---------------------------")
    print("   u    i    o")
    print("   j    k    l")
    print("\nn - Open gripper")
    print("\nm - Close grippper")
    print("\nanything else : stop")
    print("\nCTRL-C to quit")

def on_press(key):
    global servo_angles
    try:
        if key.char == 'w':
            twist.linear.x = 0.25
            twist.angular.z = 0.0
        elif key.char == 's':
            twist.linear.x = -0.25
            twist.angular.z = 0.0
        elif key.char == 'a':
            twist.linear.x = 0.0
            twist.angular.z = -2.5
        elif key.char == 'd':
            twist.linear.x = 0.0
            twist.angular.z = 2.5
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish Twist message
        cmd_vel_pub.publish(twist)

        if key.char == 'u':
            servo_angles[0] += 1
            pub1.publish(servo_angles[0])
        elif key.char == 'j':
            servo_angles[0] -= 1
            pub1.publish(servo_angles[0])
        elif key.char == 'i':
            servo_angles[1] += 1
            pub2.publish(servo_angles[1])
        elif key.char == 'k':
            servo_angles[1] -= 1
            pub2.publish(servo_angles[1])
        elif key.char == 'o':
            servo_angles[2] += 1
            pub3.publish(servo_angles[2])
        elif key.char == 'l':
            servo_angles[2] -= 1
            pub3.publish(servo_angles[2])
        elif key.char == 'n':
            servo_angles[3] = 170
            pub4.publish(servo_angles[3])
        elif key.char == 'm':
            servo_angles[3] = 125
            pub4.publish(servo_angles[3])

        # Limit the angles between 0 and 180
        for i in range(4):
            servo_angles[i] = max(0, min(180, servo_angles[i]))

    except AttributeError:
        pass
    finally:
        print_menu()

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    print_menu()

if __name__ == '__main__':
    try:
        print_menu()

        # Set up the keyboard event listener
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        
        rospy.init_node('servo_angle_publisher', anonymous=True)
        
        # Initialize publishers for each servo
        pub1 = rospy.Publisher('servo1_angle', Float32, queue_size=10)
        pub2 = rospy.Publisher('servo2_angle', Float32, queue_size=10)
        pub3 = rospy.Publisher('servo3_angle', Float32, queue_size=10)
        pub4 = rospy.Publisher('servo4_angle', Float32, queue_size=10)

        # Set up keyboard listener
  

        # Keep the script running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
