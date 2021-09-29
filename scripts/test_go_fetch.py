from  gps_to_real_coordinates import WayPoint
import rospy
from geometry_msgs.msg import PointStamped, Quaternion
import actionlib
import importlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

rospy.init_node('go_test')
gc = WayPoint()
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()
a= gc.count_waypoints_in_file('/home/ramanb/gps_ws/src/husky_gps/way_points/points_sim.txt')
print(a)
b= gc.get_waypoints('/home/ramanb/gps_ws/src/husky_gps/way_points/points_sim.txt')
print(b)
for val in b:
    # gc.latLongtoUTM(val)
   u_y=gc.latLongtoUTM(val)
#    print(u_y)
   real_point = gc.UTMtoMapPoint(u_y)
   print('real_point',real_point)

   build_goal = gc.buildGoal(real_point,real_point,True)
   print(build_goal)
   client.send_goal(build_goal)
   wait = client.wait_for_result()
   if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
   else:
         print(client.get_result())
