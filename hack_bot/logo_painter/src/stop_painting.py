#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger

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

if __name__ == "__main__":
    stop_painting()