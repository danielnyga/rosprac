#!/usr/bin/env python
import json
import os
import traceback

from rosprac.srv import *
import rospy

try:
    from prac.core import locations
    from prac.core.base import PRAC
    from prac.core.inference import PRACInference
except ImportError:
    print 'PRAC was not found. Please make sure it is in your $PYTHONPATH.'

verbose = 2
DEFAULT_CONFIG = os.path.join(locations.user_data, '.pracconf')

prac = PRAC()
prac.verbose = verbose


def prac_inference(req):
    infer = PRACInference(prac, req.instructions)
    try:
        infer.run()
    except:
        traceback.print_exc()
    tasks = []
    actioncores = []
    for node in infer.steps():
        actioncores.append({"actioncore": node.frame.actioncore,
                            "actionroles": [{k: v.type} for k, v in node.frame.actionroles.items()]})
    return InstructionsResponse(json.dumps(actioncores))


def run_pracserver():
    rospy.init_node('pracserver')
    s = rospy.Service('pracquery', Instructions, prac_inference)
    print "Waiting for instructions to interpret..."
    rospy.spin()

if __name__ == "__main__":
    run_pracserver()