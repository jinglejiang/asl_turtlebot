#!/usr/bin/env python

"""order.py : process keyboard input and publish to topic '/fruits', giving the names for the fruits to collect
rospy.Subscriber('/fruits', Int32MultiArray, self.fruits_cmd_callback)
"""

import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

APPLE = 1
PIZZA = 2
CAKE = 3
DEFAULT_ORDER = [APPLE,PIZZA,CAKE]

def readInput():
    """
    read input from terminal
    """
    input_string = raw_input()
    if not input_string:
        return DEFAULT_ORDER

    input_strings = input_string.split(' ')

    input_list = []
    for i in input_string:
        try:
            int_i = int(i)
            input_list.append(int_i)
        except ValueError:
            pass
    
    input_list = list(set(input_list))
    return input_list

def readRequest():
    """
    read requests
    """
    # print("Deliver to:")    
    # guest_names = readInput()
    print("Menu: APPLE: 1 PIZZA: 2 CAKE: 3 \nOrder food:")
    list_foods = readInput()
    print("You have ordered:")
    print(list_foods)

    return list_foods #, guest_names


 

def processRequest():    
    rospy.init_node('order', anonymous=True)    
    fruits_publisher = rospy.Publisher('/fruits', Int32MultiArray, queue_size=1)
    rate = rospy.Rate(10) # 10hz

    # list_foods, guest_name = readRequest()
    list_foods = readRequest()

    fruits_to_collect = Int32MultiArray()
    fruits_to_collect.data = list_foods

    dim = MultiArrayDimension()
    dim.label = 'fruits'
    dim.size = len(list_foods)
    dim.stride = len(list_foods)

    fruits_to_collect.layout.dim.append(dim)    


    #while not rospy.is_shutdown():
    fruits_publisher.publish(fruits_to_collect)
    rospy.loginfo(fruits_to_collect)       
    #rate.sleep()
    # else:
    #     rospy.loginfo("ROS service not found!")    

if __name__ == '__main__':

    try:
        processRequest()
    except rospy.ROSInterruptException:
        pass
