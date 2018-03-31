#!/usr/bin/env python
import argparse
import json
import sys
import rospy
from rosprac.srv import Instructions


CONSTRAINTS = {
    '!obj_to_be_put': 'apple.n.01'
}


ABSTRACTS = [
    'minute.n.01',
    'number.n.02',
    'measure.n.02'
]


def practell(howto, steps, save=False):
    rospy.wait_for_service('practell', 2)
    try:
        practell_ = rospy.ServiceProxy('practell', Instructions)
        resp = json.loads(practell_(json.dumps({'howto': howto, 'steps': steps, 'save': save})).response)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


parser = argparse.ArgumentParser()
parser.add_argument("howto", type=str, help="The howto.")
parser.add_argument("steps", nargs='+', type=str, help="The instruction single instruction steps.")
parser.add_argument("--save", action='store_true', default=False, help="Whether the instruction sheet should be saved persitently in the PRAC database.")


if __name__ == "__main__":
    args = parser.parse_args()
    print 'Querying PRAC-TELL:'
    print ' - "%s"' % args.howto
    response = practell(args.howto, args.steps, args.save)
    if not response:
        print 'PRAC did not return any result :-('
    else:
        print 'PRAC proposes the following action steps:'
        print json.dumps(response, indent=2)
