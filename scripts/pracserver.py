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

from tools import RStorage

try:
    from prac.core import locations
    from prac.core.base import PRAC, PRACDatabase
    from prac.core.inference import PRACInference
    from prac.db.ies.models import Worldmodel
    from prac.db.ies.models import Object
except ImportError:
    print 'PRAC was not found. Please make sure it is in your $PYTHONPATH.'

verbose = 1
DEFAULT_CONFIG = os.path.join(locations.user_data, '.pracconf')


wmlogger = logs.getlogger('/pracserver/worldmodel')


class PRACServer:

    def __init__(self):
        self.prac = PRAC()
        self.prac.verbose = verbose
        # self.worldmodel = Worldmodel(self.prac)
        # poopulate the world model manually
        # wm = self.worldmodel
        # wm.add(Object(self.prac, 'juice', 'carton.n.02', props={'fill_level': 'empty.s.01'}))
        # wm.add(Object(self.prac, 'basket', 'basket.n.01'))
        # wm.add(Object(self.prac, 'fridge', 'refrigerator.n.01'))
        # wm.add(Object(self.prac, 'trash', 'ashcan.n.01'))
        # # wm.add(Object(self.prac, 'cereals-unused', 'carton.n.02', props={'used_state': 'unused.s.01'}))
        # wm.add(Object(self.prac, 'milk-box-full', 'carton.n.02', props={'fill_level': 'full.s.01'}))
        # wm.add(Object(self.prac, 'cereals-box', 'carton.n.02', props={'used_state': 'secondhand.s.01'}))
        # wm.add(Object(self.prac, 'banana', 'banana.n.02'))
        # wm.add(Object(self.prac, 'apple', 'apple.n.01'))
        # wm.add(Object(self.prac, 'orange', 'orange.n.01'))

        rospy.init_node('pracserver')
        rospy.Subscriber("/world_model", String, self.update_worldmodel)

    def prac_query(self, r):
        request = RStorage(json.loads(r.request), utf8=True)
        print 'Received request:'
        pprint(request)
        infer = PRACInference(self.prac, request.instructions, self.worldmodel)
        constraints = request.constraints
        abstracts = map(str, request.get('abstracts', []))
        try:
            infer.run()
            actioncores = []
            # when done, load the grounding module to ground the concepts
            # ground = self.prac.module('grounding')
            # objdb = PRACDatabase(self.prac)
            # for obj in self.worldmodel.available.values():
            #     objdb << 'instance_of(%s,%s)' % (str(obj.id), str(obj.type))
            # objdb.write()
            for node in infer.steps():
                # actioncores.append(node.frame.tojson())
                # newactionroles = defaultdict(list)
                # for role, concept in ground(node, objdb, constraints):
                #     newactionroles[role].append(concept)
                pprint(node.frame.tojson())
                # actionroles = {role: object.type for role, object in node.frame.actionroles.items() if any([self.prac.wordnet.ishyponym(object.type, a) for a in abstracts])}
                # actionroles.update(newactionroles)
                # actioncores.append({'actioncore': node.frame.actioncore, 'actionroles': list(set(actionroles))})
                out('grounding finished')
                # end grounding process
            return InstructionsResponse(json.dumps([s.frame.toplan('json') for s in infer.steps()]))
        except Exception as e:
            traceback.print_exc()
            return InstructionsResponse(json.dumps({'error': type(e).__name__, 'reason': str(e)}))

    def prac_tell(self, r):
        '''
        The ``r`` is expected to be a JSON encoded dictionary string containing the following elements:

        {
            "howto": "Make breakfast.",
            "steps": [
                "Prepare cereals.",
                "Boil an egg."
                "Put milk on the table."
            ]
            "save": true,
        }

        If ``save`` is ``true``, the howto is persistently stored in the PRAC database.

        :param r:
        :return:
        '''
        request = RStorage(json.loads(r.request), utf8=True)
        print 'Received request:'
        pprint(request)
        try:
            howto = self.prac.tell(request.howto, request.constraints, save=request.save)
            return InstructionsResponse(json.dumps(howto.tojson()))
        except Exception as e:
            traceback.print_exc()
            return InstructionResponse(json.dumps({'error': type(e).__name__, 'reason': str(e)}))

    def serve_forever(self):
        rospy.Service('pracquery', Instructions, self.prac_query)
        rospy.Service('practell', Instructions, self.prac_tell)
        print "Waiting for instructions to interpret..."
        rospy.spin()

    def update_worldmodel(self, data):
        wm = Worldmodel(self.prac)
        for obj in RStorage(json.loads(data.data)).world:
            obj = RStorage(obj)
            wm.add(Object(self.prac, obj.name, obj.wordnet))
        self.worldmodel = wm
        # self.worldmodel = RStorage(json.loads(data.data))
        # wmlogger.debug('---------------\nupdated world model:')
        # for obj in self.worldmodel.objs:
        #     wmlogger.debug(obj.name, 'is of type', obj.type)
        # wmlogger.debug('---------------')



logs.loggers({
    '/pracserver/worldmodel': logs.newlogger(logs.console, level=logs.INFO),
    '/prac/grounding': logs.newlogger(logs.console, level=logs.DEBUG),
})

if __name__ == "__main__":
    server = PRACServer()
    server.serve_forever()
