#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger

def start_painting():
    rospy.wait_for_service('paint_start')
    print("Start painting!")
    try:
        paint_srv_start = rospy.ServiceProxy('paint_start', Trigger)
        resp = paint_srv_start()
        print(resp.message)
        return resp.success
    except rospy.ServiceException, e:
        print("Service call failed: %s", e)

def stop_painting():
    rospy.wait_for_service('paint_stop')
    print("Stop painting!")
    try:
        paint_srv_stop = rospy.ServiceProxy('paint_stop', Trigger)
        resp = paint_srv_stop()
        print(resp.message)
        return resp.success
    except rospy.ServiceException, e:
        print("Service call failed: %s", e)

def main():
    # rospy.init_node("test_paint_relay")
    while not rospy.is_shutdown():
        print("Start painting process ...")
        start_painting()
        rospy.sleep(0.5)
        stop_painting()
        rospy.sleep(0.5)

if __name__ == "__main__":
    main()