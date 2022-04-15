#!/usr/bin/env python
import re
import rospy

def num_drones():
    topics = rospy.get_published_topics()
    expression = re.compile("/drone./takeoff")

    topic = []
    for t in topics:
        topic.append(t[0])

    filtered = list(filter(expression.match, topic))
    return len(filtered)