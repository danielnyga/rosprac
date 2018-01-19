#!/usr/bin/env python
import argparse
import json
import sys
import rospy
from rosprac.srv import *


def pracquery(instructions):
    rospy.wait_for_service('pracquery')
    try:
        pracquery_ = rospy.ServiceProxy('pracquery', Instructions)
        resp = pracquery_(instructions)
        return resp.steps
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


parser = argparse.ArgumentParser(description=' %s [instructions]' % (sys.argv[0]))
parser.add_argument("instructions", nargs='+', type=str, help="The instruction.")

if __name__ == "__main__":
    args = parser.parse_args()
    print 'Querying PRAC for instruction(s):'
    for i in args.instructions:
        print ' - "%s"' % i
    res = pracquery(args.instructions)
    if not res:
        print 'PRAC did not return any result :-('
    else:
        print 'PRAC proposes the following action steps:'
        print json.dumps(json.loads(res.steps), indent=2)
