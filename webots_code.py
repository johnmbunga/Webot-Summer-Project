"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor


from controller import Robot



import math
import numpy as np
from numpy import random

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 64
robwidth=0.075
wheelwidth=0.01
rob_width=robwidth+wheelwidth
MAXSPEED=6

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

right_motor=robot.getDevice('motor_1')
left_motor=robot.getDevice('motor_2')
right_motor.setPosition(float('inf'))
left_motor.setPosition(float('inf'))
right_motor.setVelocity(0)
left_motor.setVelocity(0)
gps_right=robot.getDevice('gps_1')
gps_left=robot.getDevice('gps_2')
gps_right.enable(timestep)
gps_left.enable(timestep)
#open map
file_data=open('C:/Users/John Bless Mbunga/Downloads/OccupancyGrid1.csv')
my_data=np.genfromtxt(file_data, delimiter=',')
my_data[np.isnan(my_data)] = 0
def magnitude(x):
    #magnitude of vector
    return math.sqrt(sum(i ** 2 for i in x))
def global_pose_calculation():
    #calculate pose from gps

    return pose_calculated, angle_vector
def direction_movement(pose_calculated,angle_vector_calculated,point):
    #calculate rotation to face desired point

    return angular
def matrix_bounded(the_matrix,point_chosen):
#ensure points are with maze or map dimension

    return point_chosen_2 
def map_to_gps(the_matrix,point_chosen):
#convert map co-ordinates to gps co-ordinates
    return point_generated
def gps_to_map(the_matrix,point_chosen):
#convert gps co-ordinates to map co-ordinates    
    return point_generated
    
def object_potential(the_matrix,distance):
#U potential added using neighbouring points certain distance away from objects
    
    
    return the_matrix
def collisionavoid(the_matrix,point_chosen,point_desired):
#checks path between two points for obstacles

    return Sumobstacles
    
def thick_line_collision_avoid(the_matrix,point_chosen,point_desired):
#using collision avoid function path is checked in a thicker line
    return Sumob
        
        
def path_distance(nodes,point):
#calculates path length between a node and start node
    return distance  

def parent_arrange(points,nodes_explored,my_data,index):
#changes parent of nodes within a certain range if the distance is shorter
#also can generate list of nodes within a certain range of a node
    return points, nodes_explored

def reconstructpath(nodes,goal,start,goal_index):
#using parent info and starting from goal node the path is calculated
#index of each parent is also recorded for beacon function
    return path,parent_path
      
def path_connection(nodes_explored,goal_node,start_node,goal_index,my_data):
#using the reconstrucrtion and thick_line_collision_avoid nodes on the path 
#which are connected to other nodes of the path are recorded in two ways starting from goal node and 
#starting from start node
    return path_connections,connect_back,beacon_smart

    
def make_list(path_connections,connect_back):
#makes beacon lists using path_connections function and 
#removing unneccessary connections
    

    return path_op,path_op_2
                
    
def beacon_function(nodes_explored,goal_node,start_node,goal_index,my_data):
#using path_connections and make_list function beacon parents are assigned to nodes
    
    return nodes_explored
                
        
def find_unoccupied(the_matrix,unoccupied_points):
#finds number of unoccupied points
    return unoccupied_points     

def normal_sampling(nodes_explored,distance,the_matrix,unoccupied_points,goal_reached,goal_node,count):
#normal sampling occurs from random points also
#detects when goal reached
    return nodes_explored,unoccupied_points,count,goal_reached
        
    
def parent_arrangement(nodes_explored,the_matrix):
#using parent arrange function the parent arrangement of node and 
#within certain range is performed
    return nodes_explored   
def near_beacon_check(nodes_explored,goal_node,start_node,goal_index,the_matrix,near_beacon_unoccupied,unoccupied_points,distance_beacon,near_beacon_list):
#assesses if there are nodes not explored or occupied in vicinity of beacons
#outputs points within range if present
    return near_beacon_unoccupied,near_beacon_list
                
                  
def near_beacon_sampling(nodes_explored,the_matrix,goal_node,start_node,goal_index,beacon_list,near_beacon_unoccupied,unoccupied_points,distance_beacon,near_beacon_list):
#sample nodes not explored near one of the beacons
                    
    return near_beacon_unoccupied,unoccupied_points,nodes_explored,near_beacon_list
        
      
def biasing_ratio_and_check(n,percentage,constant):
#calculates current bias ratio and
#deciding whether bias sampling is used
    return bias_sample_enable
    
         
                
def RRTstar(the_matrix,pose,distance):

    #intialise variables some parts extracted
    nodes_explored=[]
    nodes_explored.append({"row":start_node["row"],"column":start_node["column"],"p":0,"ds":0})
    goal_reached=0
    count=0
    unoccupied_points=float("inf")
    #RRt*
    while goal_reached==0:
        nodes_explored,unoccupied_points,count,goal_reached=normal_sampling(nodes_explored,distance,the_matrix,unoccupied_points,goal_reached,goal_node,count)
        nodes_explored=parent_arrangement(nodes_explored,the_matrix)
    #intialise new variables
    goal_index=len(nodes_explored)-1
    previous_cost=path_distance(nodes_explored,goal_index)  
    
  
    #first beacon function used an dinitialise other variables
    nodes_explored=beacon_function(nodes_explored,goal_node,start_node,goal_index,my_data)
    near_beacon_unoccupied=float("inf")
    total_unoccupied=find_unoccupied(the_matrix,0)
    iterations=total_unoccupied-unoccupied_points
    constant=0.02
    near_beacon_list=[]
    for i in range(1000):
        iterations+=1
        
        bias_enable=biasing_ratio_and_check(iterations,total_unoccupied,constant)
        if current_cost<=0.98*previous_cost:
            #beacon function is run and cost updates

        near_beacon_unoccupied,near_beacon_list=near_beacon_check(nodes_explored,goal_node,start_node,goal_index,the_matrix,near_beacon_unoccupied,unoccupied_points,5,near_beacon_list)
        if near_beacon_unoccupied>0 and bias_enable==1 and current_cost>102:
           #beacon sampling used
 
        elif current_cost>100:
           #normal sampling used
        else:
            #optimal path assumed reached
            break


    return nodes_explored,start_node,goal_node,goal_index

first_step=1            
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1: 
    pose,angle_vector=global_pose_calculation()     
    if first_step==1:
        #intialise map, path, and first point
        first_step=0        
    #speed and rotaion
    rob_angular_SPEED=direction_movement(pose,angle_vector,point)
    SPEED_rob=MAXSPEED
    navdist=math.sqrt((pose[0][0]-point[0])**2+(pose[1][0]-point[1])**2)
    #check near point
    if navdist<0.4:
    #changes node when close to current
    #desired node

    #set speeds
    left_speed=SPEED_rob+rob_angular_SPEED
    right_speed=SPEED_rob-rob_angular_SPEED
    right_motor.setVelocity(right_speed)
    left_motor.setVelocity(left_speed)
    pass

# Enter here exit cleanup code.
