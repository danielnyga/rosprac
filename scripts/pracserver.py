#!/usr/bin/env python
import json
import os
import traceback
from pprint import pprint

from collections import defaultdict
from dnutils import out, logs
from rosprac.srv import *
import rospy
from std_msgs.msg import String

import prac
from tools import RStorage

try:
    from prac.core import locations
    from prac.core.base import PRAC, PRACDatabase
    from prac.core.inference import PRACInference
except ImportError:
    print 'PRAC was not found. Please make sure it is in your $PYTHONPATH.'

verbose = 1
DEFAULT_CONFIG = os.path.join(locations.user_data, '.pracconf')


wmlogger = logs.getlogger('/pracserver/worldmodel')


class PRACServer:

    def __init__(self):
        self.prac = prac.PRAC()
        self.prac.verbose = verbose
        self.worldmodel = None
        rospy.init_node('pracserver')
        rospy.Subscriber("/world_model", String, self.update_worldmodel)

    def prac_inference(self, req):
        infer = PRACInference(self.prac, req.instructions)
        try:
            infer.run()
            # when done, load the grounding module to ground the concepts
            ground = self.prac.module('grounding')
            objdb = PRACDatabase(self.prac)
            for obj in self.worldmodel.objs:
                objdb << 'instance_of(%s,%s)' % (obj.name, obj.type)
            objdb.write()
            actioncores = []
            for node in infer.steps():
                actionroles = defaultdict(list)
                actioncores.append({'actioncore': node.frame.actioncore, 'actionroles': actionroles})
                for role, concept in ground(node, objdb, {}):
                    actionroles[role].append(concept)
                out('grounding finished')
        except:
            traceback.print_exc()
        return InstructionsResponse(json.dumps(actioncores))

    def serve_forever(self):
        s = rospy.Service('pracquery', Instructions, self.prac_inference)
        print "Waiting for instructions to interpret..."
        rospy.spin()

    def update_worldmodel(self, data):
        # self.worldmodel = RStorage(json.loads(data.data))
        self.worldmodel = RStorage({'objs': [
            {'name': 'o1', 'type': 'banana.n.01'},
            {'name': 'w2', 'type': 'blender.n.01'},
            {'name': 'o3', 'type': 'push_button.n.01'},
            {'name': 'w3', 'type': 'strawberry.n.01'},
            {'name': 'w4', 'type': 'apple.n.01'},
            {'name': 'w5', 'type': 'cup.n.01'},
            {'name': 'w6', 'type': 'chair.n.01'},
            {'name': 'w7', 'type': 'milk.n.01'},
            {'name': 'w8', 'type': 'blueberry.n.02'},
            {'name': 'w9', 'type': 'tomato.n.01'}
        ]})
        wmlogger.debug('---------------\nupdated world model:')
        for obj in self.worldmodel.objs:
            wmlogger.debug(obj.name, 'is of type', obj.type)
        wmlogger.debug('---------------')


logs.loggers({
    '/pracserver/worldmodel': logs.newlogger(logs.console, level=logs.INFO),
    '/prac/grounding': logs.newlogger(logs.console, level=logs.DEBUG),
})

if __name__ == "__main__":
    server = PRACServer()
    server.serve_forever()
