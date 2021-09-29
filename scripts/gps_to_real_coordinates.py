#!/usr/bin/env python
import math
import tf
import rospy
import actionlib
import importlib
import math
import geonav_conversions as gc
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class WayPoint:
    def __init__(self):
        pass
    
    def count_waypoints_in_file(self,file):
        '''
        count the number of gps locations in the file
        '''
        file = open(file, 'r')
        lines = file.readlines()
        return len(lines)

    def get_waypoints(self,file):
        '''
        get way points as a list 
        '''
        file = open(file, 'r')
        gps_way_point_list = []
        
        lines = file.readlines()
        for line in lines:
            line = line.strip()
            lat,lon = line.split()
            gps_way_point_list.append((float(lat),float(lon)))
        # print('gps_way_point_list',gps_way_point_list)
        return gps_way_point_list
    
    def latLongtoUTM(self,lat_log_pair):
        '''
        convert latitude and longitude to utm '''
    
        utm_point_output = PointStamped()
        # print(utm_point_output)
        utm_y, utm_x, utmzone = gc.LLtoUTM(lat_log_pair[0],lat_log_pair[1])
        # print(type(utm_y))
        # print(utm_y)
        # print(utm_x)
        utm_point_output.header.frame_id = "utm"
        utm_point_output.header.stamp = rospy.Time.now()
        utm_point_output.point.x = utm_x
        utm_point_output.point.y = utm_y
        utm_point_output.point.z = 0
        # print(utm_point_output)
        return utm_point_output

    def UTMtoMapPoint(self,utm_point):
        '''
        Utm to map points
        '''
        listener = tf.TransformListener()
        is_done = False
        while not is_done:
            try:
                listener.waitForTransform("/odom", "/utm", rospy.Time(0),rospy.Duration(4.0))
                p = listener.transformPoint("odom",utm_point)
                is_done = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('No transform between odom to utm')
                rospy.sleep(0.01)
           
        return p

    def buildGoal(self,map_point,map_next,is_last_point):
        '''
        Build goal to send to move_base
        '''
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'odom'
        move_base_goal.target_pose.header.stamp = rospy.Time(0)
        move_base_goal.target_pose.pose.position.x = map_point.point.x
        move_base_goal.target_pose.pose.position.y = map_point.point.y
        if not is_last_point :
            delta_x = map_next.point.x - map_point.x
            delta_y = map_next.point.y - map_point.y
            yaw_curr = math.atan2(delta_y,delta_x)       
            move_base_goal.target_pose.orientation = Quaternion(*quaternion_from_euler(yaw_curr, 0.0, 0.0))
        else:
            move_base_goal.target_pose.pose.orientation.w = 1.0
        
        return move_base_goal






# if __name__ == "__main__":
#     rospy.init_node('gps_waypoint')
#     client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
#     client.wait_for_server()
#     waypoint=WayPoint()
#     gps_way_point_file= rospy.get_param("/outdoor_waypoint_nav/coordinates_file",'/home/ramanb/gps_ws/src/husky_gps/way_points/points_sim.txt')
#     num_of_waypoints = waypoint.count_waypoints_in_file(gps_way_point_file)
#     way_points = waypoint.get_waypoints(gps_way_point_file)
#     is_final_point = False
#     for i,lan,log in enumerate(way_points):
#         pass
        

'''






if __name__ == '__main__':
    rospy.init_node('gps_to_utm_to_real_coordinates_node')

    listener = tf.TransformListener()


    try:
        (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    listener.transformPoint("odom", UTM_input, map_point_output);

    angular = 4 * math.atan2(trans[1], trans[0])
    linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    turtle_vel.publish(cmd)

    rate.sleep()



#!/usr/bin/env python
import importlib
import geonav_conversions as gc
from rospy.core import loginfo
importlib.reload(gc)
# Import AlvinXY transformation module
import alvinxy as axy
importlib.reload(axy)
import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

def newOdom(data):
    global lat,lon

    lat = data.latitude
    lon = data.longitude 

# rospy.Subscriber("/gps/filtered",NavSatFix,newOdom)

def get_xy_based_on_lat_long(lat,lon, name):
    # Define a local orgin, latitude and longitude in decimal degrees
    # GPS Origin

    olat = 13.5  #49.8999986196
    olon = 80.003669

    xg2, yg2 = gc.ll2xy(lat,lon,olat,olon)
    utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
    xa,ya = axy.ll2xy(lat,lon,olat,olon)

    return xa, ya

if __name__ == '__main__':
    
    rospy.init_node('gps_to_xyz_node')
    xg2, yg2 = get_xy_based_on_lat_long(lat=(14.589993),lon=(80.003492), name="MAP")
    print(xg2,yg2)                                                                          

##commenting it out

import rospy
import actionlib
import importlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geonav_transform.geonav_conversions as gc
importlib.reload(gc)
import alvinxy.alvinxy as axy
importlib.reload(axy)

def get_xy_based_on_lat_long(lat,lon, name):

    # Define a local orgin, latitude and longitude in decimal degrees
    # GPS Origin

    olat = (13.59)  #49.8999986196
    olon = (80.003)
    xg2, yg2 = gc.ll2xy(lat,lon,olat,olon)
    utmy, utmx, utmzone = gc.LLtoUTM(lat,lon)
    xa,ya = axy.ll2xy(lat,lon,olat,olon)

    return xa, ya

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    Target_lat = 13.589635
    Target_lon = 80.003317
    a,b = get_xy_based_on_lat_long(Target_lat,Target_lon,name="MAP")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = a
    goal.target_pose.pose.position.y = b
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
'''