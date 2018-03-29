#!/usr/bin/env python
import argparse
import json
import sys
import rospy
from rosprac.srv import *


CONSTRAINTS = {
    '!obj_to_be_put': 'apple.n.01'
}


ABSTRACTS = [
    'minute.n.01',
    'number.n.02',
    'measure.n.02'
]


def pracquery(instructions):
    rospy.wait_for_service('pracquery')
    try:
        pracquery_ = rospy.ServiceProxy('pracquery', Instructions)
        resp = json.loads(pracquery_(json.dumps({'instructions': instructions, 'constraints': CONSTRAINTS, 'abstracts': ABSTRACTS})).response)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


parser = argparse.ArgumentParser(description=' %s [instructions]' % (sys.argv[0]))
parser.add_argument("instructions", nargs='+', type=str, help="The instruction.")


if __name__ == "__main__":
    args = parser.parse_args()
    print 'Querying PRAC for instruction(s):'
    for i in args.instructions:
        print ' - "%s"' % i
    response = pracquery(args.instructions)
    if not response:
        print 'PRAC did not return any result :-('
    else:
        print 'PRAC proposes the following action steps:'
        print json.dumps(response, indent=2)
