#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped,Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus

def pose_gen():
    path = [[14, 14.0, 0],
    [13, 14, 0],
    [14, 14.0, 0],
    [13.24698, 15.563663, 0],
    [12.62349, 14.781831, 0],
    [13.24698, 15.563663, 0],
    [11.554958, 15.949856, 0],
    [11.777479, 14.974928, 0],
    [11.554958, 15.949856, 0],
    [10.198062, 14.867767, 0],
    [11.099031, 14.433884, 0],
    [10.198062, 14.867767, 0],
    [10.198062, 13.132233, 0],
    [11.099031, 13.566116, 0],
    [10.198062, 13.132233, 0],
    [11.554958, 12.050144, 0],
    [11.777479, 13.025072, 0],
    [11.554958, 12.050144, 0],
    [13.24698, 12.436337, 0],
    [12.62349, 13.218169, 0],
    [13.24698, 12.436337, 0]]
    orientation = [[0,0,1,0],[0,0,1,0],[0,0,1,0],
                    [0,0,0.846724,0.532032],[0,0,0.900969,-0.433884],[0,0,0.900969,-0.433884],
                    [0,0,0.993712,0.111964],[0,0,0.62349,-0.781831],[0,0,0.62349,-0.781831],
                    [0,0,0.943883,-0.330279],[0,0,0.222521,-0.974928],[0,0,0.222521,-0.974928],
                    [0,0,0.707107,-0.707107],[0,0,0.222521,0.974928],[0,0,0.222521,0.974928],
                    [0,0,0.330279,-0.943883],[0,0,0.62349,0.781831],[0,0,0.62349,0.781831],
                    [0,0,0.111964,0.993712],[0,0,0.900969,0.433884],[0,0,0.900969,0.433884]]
    for i,j in zip(path,orientation):
        q = Quaternion(j[0],j[1],j[2],j[3])
        p = Point(i[0],i[1],i[2])
        yield PoseStamped(header=None,pose=Pose(position=p,orientation=q))



class position_example():
    def __init__(self):
        self.pub = rospy.Publisher("/jackal0/orientation_control", PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber("/jackal0/jackal_velocity_controller/status", GoalStatus, self.callback)

        self.path_generator = pose_gen()

    def callback(self,msg):
        if msg.status == 3:
            pose = next(self.path_generator)
            if pose is not None:
                self.pub.publish(pose)





def main():
    rospy.init_node('position_example')
    position_example()
    rospy.spin()
#     while not rospy.is_shutdown():
       
        # pub.publish(PStamped)
        # step +=1 
        # rate.sleep()

if __name__ == '__main__':
    main()