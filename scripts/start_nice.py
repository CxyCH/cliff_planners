#!/usr/bin/python
import subprocess
import os
import signal
import time
import sys
import math
from functools import partial

mapnames = ['intersect'];

print('Starting all tests')

for mapname in mapnames:
    print('Starting ' + mapname)
    roslaunchStart_p = subprocess.Popen('roslaunch cliff_planners test_intersect.launch cliffmap:=' + mapname, shell=True, preexec_fn=os.setsid);
    time.sleep(4);
    rosrunStart_p = subprocess.Popen('rosrun cliff_planners start_tests.py ' + mapname, shell=True, preexec_fn=os.setsid);

    try:
        rosrunStart_p.wait();
    except KeyboardInterrupt:
        os.killpg(roslaunchStart_p.pid, signal.SIGINT);
        os.killpg(rosrunStart_p.pid, signal.SIGINT);
        break;

    os.killpg(roslaunchStart_p.pid, signal.SIGINT)
    roslaunchStart_p.wait();
    print('One map done.')
    time.sleep (2)

print("All maps done.");

