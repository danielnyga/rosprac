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

    def prac_inference(self, r):
        request = RStorage(json.loads(r.request), utf8=True)
        print 'Received request:'
        pprint(request)
        infer = PRACInference(self.prac, request.instructions)
        constraints = request.constraints
        abstracts = map(str, request.get('abstracts', []))
        try:
            infer.run()
            # when done, load the grounding module to ground the concepts
            ground = self.prac.module('grounding')
            objdb = PRACDatabase(self.prac)
            for obj in self.worldmodel.objs:
                objdb << 'instance_of(%s,%s)' % (str(obj.name), str(obj.type))
            objdb.write()
            actioncores = []
            for node in infer.steps():
                newactionroles = defaultdict(list)
                for role, concept in ground(node, objdb, constraints):
                    newactionroles[role].append(concept)
                actionroles = {role: object.type for role, object in node.frame.actionroles.items() if any([self.prac.wordnet.ishyponym(object.type, a) for a in abstracts])}
                actionroles.update(newactionroles)
                actioncores.append({'actioncore': node.frame.actioncore, 'actionroles': actionroles})

                out('grounding finished')
        except:
            traceback.print_exc()
        return InstructionsResponse(json.dumps(actioncores))

    def serve_forever(self):
        s = rospy.Service('pracquery', Instructions, self.prac_inference)
        print "Waiting for instructions to interpret..."
        rospy.spin()

    def update_worldmodel(self, data):
        self.worldmodel = RStorage(json.loads(data.data))
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
