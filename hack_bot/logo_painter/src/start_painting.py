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

if __name__ == "__main__":
    start_painting()