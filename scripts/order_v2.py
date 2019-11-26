#!/usr/bin/env python

"""order_v2.py : process keyboard input and publish to topic '/fruits', giving the names for the fruits to collect
                 arrange the order of fruit names
rospy.Subscriber('/fruits', Int32MultiArray, self.fruits_cmd_callback)
"""

import rospy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension
import numpy as np
import tf

APPLE = 1
PIZZA = 2
CAKE = 3
HOME = 0
DEFAULT_ORDER = [APPLE,PIZZA,CAKE]

class Order():
    def __init__(self):
        rospy.init_node('order', anonymous=True)
        self.fruits_publisher = rospy.Publisher('/fruits', Int32MultiArray, queue_size=1)        
        rospy.Subscriber('/fruit_locations', Int32MultiArray, self.fruit_locations_callback)
        
        self.trans_listener = tf.TransformListener()
        (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        self.x = translation[0]
        self.y = translation[1]
        
        # self.fruit_locations = {APPLE:(1, 3), PIZZA:(3, 1), CAKE:(3, 2)}
        

    def publish_fruits(self):
        self.arrange_orders()
        
        fruits_to_collect = Int32MultiArray()
        fruits_to_collect.data = self.fruits_list

        dim = MultiArrayDimension()
        dim.label = 'fruits'
        dim.size = len(self.fruits_list)
        dim.stride = len(self.fruits_list)

        fruits_to_collect.layout.dim.append(dim)
        rate = rospy.Rate(20)    


        while not rospy.is_shutdown():
            self.fruits_publisher.publish(fruits_to_collect)
            rospy.loginfo(fruits_to_collect)       
            rate.sleep()


        

    def fruit_locations_callback(self, msg):
        self.fruit_locations = msg.data
        # TODO: not defined structure

    def arrange_orders(self):
        """
        rearrange goals order
        """
        input_fruits = readRequest()
        input_fruit_list = input_fruits.copy()   
        self.fruits_list = []
        
        start = (self.x, self.y)
        home_location = (0, 0)
        while len(input_fruit_list) is not 0:            
            min_dis = np.inf
            min_idx = 0            

            for fruit_name in input_fruit_list:
                cur_dis = dis(start, self.fruit_locations[fruit_name])
                if cur_dis < min_dis:
                    min_dis = cur_dis
                    min_idx = fruit_name
       
           
            self.fruits_list.append(min_idx)
            input_fruit_list.remove(min_idx) 
            start =  self.fruit_locations[min_idx] 
             




def dis(a, b):
    """
    distance between two points ????
    TODO: How to calculate? make use of map info ?
    """
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def readInput():
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
    # print("Deliver to:")    
    # guest_names = readInput()
    print("Menu: APPLE: 1 PIZZA: 2 CAKE: 3 \nOrder food:")
    list_foods = readInput()
    print("You have ordered:")
    print(list_foods)
    return list_foods #, guest_names


if __name__ == '__main__':
    order = Order()

    try:
        order.publish_fruits()        
    except rospy.ROSInterruptException:
        pass