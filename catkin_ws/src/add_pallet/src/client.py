#!/usr/bin/env python

PACKAGE = "add_pallet"

import rospy

import dynamic_reconfigure.client

if __name__=="__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("server", timeout=30)

    r = rospy.Rate(10)
    x = 0
    b = False
    while not rospy.is_shutdown():
        # print("input:")
        x = input("INTEGER: ")
        b = not b
        client.update_configuration({"int_param":x, "bool_param":b})
        r.sleep()