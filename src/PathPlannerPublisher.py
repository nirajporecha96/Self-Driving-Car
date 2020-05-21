#!/usr/bin/env python

import os.path
import rospy
import csv
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
import utils


PLAN_PUB_TOPIC = "/planner_node/final_plan"
INITIAL_PUB_TOPIC = "/initialpose"
GOAL_PUB_TOPIC = "/move_base_simple/goal"
MAP_TOPIC = "static_map" 
WP_VIZ_TOPIC = "/planner_node/waypoints"

class PathPlanner:

    def __init__(self, plan, waypoints):
        self.plan = plan
        self.waypoints = waypoints

        self.plan_pub = rospy.Publisher(PLAN_PUB_TOPIC, PoseArray, queue_size=10)
        self.initial_pub = rospy.Publisher(INITIAL_PUB_TOPIC, PoseWithCovarianceStamped, queue_size=10)
        self.goal_pub = rospy.Publisher(GOAL_PUB_TOPIC, PoseStamped, queue_size=10)

        self.waypoint_pub = rospy.Publisher(WP_VIZ_TOPIC, PoseArray, queue_size=10)
        
    '''
    Given two poses, publish them as a start and goal pose to the appropriate topics.
    '''
    def get_segment_plan(self, start_pose, waypoint_pose, time_stamp):     

        start_x,start_y ,start_theta = start_pose[0],start_pose[1], start_pose[2]
        wp_x,wp_y,wp_theta = waypoint_pose[0], waypoint_pose[1], waypoint_pose[2]

        # set up and publish the start pose startMsg
        startMsg = PoseWithCovarianceStamped()
        quaternion = utils.angle_to_quaternion(start_theta)
        startMsg.header.stamp = time_stamp
        startMsg.header.frame_id = "/map"
        startMsg.pose.pose.position.x = start_x
        startMsg.pose.pose.position.y = start_y
        startMsg.pose.pose.orientation = quaternion

        # set up and publish the goal pose goalMsg
        goalMsg = PoseStamped()
        quaternion = utils.angle_to_quaternion(wp_theta)
        goalMsg.header.stamp = time_stamp 
        goalMsg.header.frame_id = "/map"
        goalMsg.pose.position.x = wp_x
        goalMsg.pose.position.y = wp_y
        goalMsg.pose.orientation = quaternion

        self.initial_pub.publish(startMsg) 
        self.goal_pub.publish(goalMsg)


def map_to_world(poses,map_info):

    poses[:,1]=2*618-poses[:,1]

    scale = map_info.resolution
    angle = utils.quaternion_to_angle(map_info.origin.orientation)

    # Rotation
    c, s = np.cos(angle), np.sin(angle)
    
    # Store the x coordinates since they will be overwritten
    temp = np.copy(poses[:,0])
    poses[:,0] = c*poses[:,0] - s*poses[:,1]
    poses[:,1] = s*temp       + c*poses[:,1]

    # Scale
    poses[:,:2] *= float(scale)

    # Translate
    poses[:,0] += map_info.origin.position.x
    poses[:,1] += map_info.origin.position.y
    poses[:,2] += angle
    return poses


if __name__ == "__main__":

    rospy.init_node("PathPlanner", anonymous=True) 

    # Default values
    plan_topic = "/planner_node/car_plan"

    #set up waypoints and poses automatically; this does not always give
    #the most straightforward paths, so customized poses were used 
    #instead Logic used from a code found on Github

    start_pose = np.array([[2500, 640, 6.0]])	#Start point

    waypoints = np.array([[2550, 680, 6.2 ], 	
    					  [2600, 660, 6.7 ],	#Waypoint1
    					  [2600, 560, 8.0 ],
                          [2400, 490, 2.8 ],
                          [2050, 370, 3.0 ],
                    	  [1880, 440, 3.0 ],	#Waypoint2
                       	  [1699, 450, 4.0 ],
                          [1620, 510, 4.0 ],
                          [1500, 575, 2.0 ],
                          [1435, 545, 2.7 ],	#Waypoint3
                          [1250, 460, 3.0 ],	#Waypoint4
                          [1150, 460, 3.2 ],
                          [950 , 480, 3.6 ],
                          [600 , 700, 3.9 ],
                          [540 , 835, 4.8 ]])	#Waypoint5

    #Waypoints used for Simulation Only (Section 6)
    '''start_pose = np.array([[1490, 570, 1.85]])

    waypoints = np.array([[600, 835, 4.0 ],
    					  [600, 700, 5.0 ],
                          [2500, 640, 6.0]])'''
    
    
    start_pose1 = np.array([32.7, 11.3, 4])

    waypoints1 = np.array([30.52, 11.05, 2])

    plan=[]
    map_img, map_info = utils.get_map(MAP_TOPIC)

    '''Below 8 lines were used to find the intermediate waypoints to create an optimal path'''
    map_to_world(start_pose,map_info)
    map_to_world(waypoints,map_info)
    start_pose1 = utils.world_to_map(start_pose1,map_info)
    waypoints1  = utils.world_to_map(waypoints1,map_info)

    print ('start_pose1:')
    print(start_pose1)
    print('waypoints1:')
    print(waypoints1)

    pp = PathPlanner(plan,waypoints)
    
    # offline plan location
    offline_path = '/home/car-user/racecar_ws/src/final/offline_paths/final_refined.npy'

    # check if offline plan already exists. if yes, do not continue
    if os.path.isfile(offline_path):
      
      print "Offline plan found"
      offline_plan = np.load(offline_path)
      plan = offline_plan
      

    else:
      print "Offline plan not found: A new plan will be computed"

      #print('Waypoints Published')
      #pp.print_waypoints(waypoints)
      # loop over waypoints to get path plans between them
      start_pose = start_pose[0,:]
      for waypoint_pose in waypoints: 

          string = str(waypoint_pose[0])+str(waypoint_pose[1])
          rospy.sleep(1)

          pp.get_segment_plan(start_pose, waypoint_pose, rospy.get_rostime())
          
          print "Waiting for a plan..."

	  rospy.sleep(1)

          #add segment plan to the plan
          plan_msg = rospy.wait_for_message(plan_topic, PoseArray)
          plan.extend(plan_msg.poses)
          start_pose = waypoint_pose

    
    np.save(offline_path,plan)
    print "done planning."


    rospy.sleep(5)
    poseArrayMsg = PoseArray()
    poseArrayMsg.header.frame_id = "/map"
    poseArrayMsg.poses = plan

    while 6==6:
      rospy.sleep(5)
      pp.plan_pub.publish(poseArrayMsg) 
      print "Plan Published"
    

    rospy.spin()  
