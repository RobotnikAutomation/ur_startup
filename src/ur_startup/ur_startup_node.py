#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys

if sys.version_info.major == 3:
    from ur_startup.startup import UrStartup
else:
    from startup import UrStartup


def main():

    rospy.init_node("ur_startup_node")

    rc_node = UrStartup()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
