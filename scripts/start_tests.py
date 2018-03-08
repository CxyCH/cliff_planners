#!/usr/bin/python
import rospy
import subprocess
import os
import signal
import time
import sys
import math
from functools import partial
from geometry_msgs.msg import PoseStamped as gPoseStamped

argn = len(sys.argv)
mapname = str(sys.argv[1])

print('Starting tests')
rospy.init_node("run_tests")
pose_pub = rospy.Publisher('/pose', gPoseStamped, queue_size=1)
start = gPoseStamped()
start.header.frame_id = "map"
start.pose.position.x = 2.5;
start.pose.position.y = 9.5;
start.pose.orientation.w = math.cos(-math.pi/4.0);
start.pose.orientation.z = math.sin(-math.pi/4.0);

end = gPoseStamped()
end.header.frame_id = "map"
end.pose.position.x = 2.5;
end.pose.position.y = 0.5;
end.pose.orientation.w = math.cos(-math.pi/4.0);
end.pose.orientation.z = math.sin(-math.pi/4.0);

time.sleep(1)
rate = rospy.Rate(10);

iteration = 0;

while iteration < 25 and not rospy.is_shutdown():
    print('Starting demo')
    bagStart_p = subprocess.Popen('rosbag record -o n_' + mapname + ' /performance_measures /path', shell=True, preexec_fn=os.setsid)
    pose_pub.publish(start)
    time.sleep(1)
    pose_pub.publish(end)
    time.sleep(1)

    i = 0;
    print('Run: ' + str(iteration))
    while i < 61 and not rospy.is_shutdown():
        time.sleep(1)
        i = i + 1;
    print('Demo finished')
    os.killpg(bagStart_p.pid, signal.SIGINT)
    print('Killed rosbag')
    print('OK')
    time.sleep(2)
    iteration = iteration + 1
print('50 runs completed for ' + mapname)

