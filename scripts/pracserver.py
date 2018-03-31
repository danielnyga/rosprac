#!/usr/bin/env python
import copy
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
from collections import namedtuple

try:
    from prac.core import locations
    from prac.core.base import PRAC, PRACDatabase
    from prac.core.inference import PRACInference
    from prac.db.ies.models import Worldmodel, toplan
    from prac.db.ies.models import Object
except ImportError:
    print 'PRAC was not found. Please make sure it is in your $PYTHONPATH.'

verbose = 1
DEFAULT_CONFIG = os.path.join(locations.user_data, '.pracconf')


wmlogger = logs.getlogger('/pracserver/worldmodel')

PseudoRequest = namedtuple('Request', 'request')

class PRACServer:

    def make_dummy_worldmodel(self):
        # poopulate the world model manually
        wm = Worldmodel(self.prac, cw=True)
        wm.add(Object(self.prac, 'juice', 'carton.n.02', props={'fill_level': 'empty.a.01'}))
        wm.add(Object(self.prac, 'basket', 'basket.n.01'))
        wm.add(Object(self.prac, 'fridge', 'electric_refrigerator.n.01'))
        wm.add(Object(self.prac, 'trash', 'ashcan.n.01'))
        # wm.add(Object(self.prac, 'cereals-unused', 'carton.n.02', props={'used_state': 'unused.s.01'}))
        wm.add(Object(self.prac, 'milk-box-full', 'carton.n.02', props={'fill_level': 'full.a.01'}))
        wm.add(Object(self.prac, 'cereals-box', 'carton.n.02', props={'used_state': 'secondhand.s.01'}))
        wm.add(Object(self.prac, 'banana', 'banana.n.02'))
        wm.add(Object(self.prac, 'apple', 'apple.n.01'))
        wm.add(Object(self.prac, 'orange', 'orange.n.01'))
        # out(wm.getall(Object(self.prac, 'id', type_='container.n.01', props={'fill_level': 'empty.a.01'})))
        # out(Object(self.prac, 'orange', 'orange.n.01').matches(Object(self.prac, 'id', type_='fruit.n.01')))
        return wm

    def __init__(self):
        self.prac = PRAC()
        self.prac.verbose = verbose

        rospy.init_node('pracserver')
        rospy.Subscriber("/world_model", String, self.update_worldmodel)

    def prac_query(self, r):
        request = RStorage(json.loads(r.request), utf8=True)
        if not hasattr(self, 'worldmodel'):
            self.worldmodel = self.make_dummy_worldmodel()
            wmlogger.warning('no worldmodel coming in from the topic. using dummy worldmodel.')
        print 'Received request:'
        pprint(request)
        infer = PRACInference(self.prac, request.instructions, self.worldmodel)
        constraints = request.constraints
        abstracts = map(str, request.get('abstracts', []))
        try:
            infer.run()
            frames = [n.frame for n in infer.steps()]
            newframes = []
            for frame in frames:
                skipframe = False
                for role, obj in frame.actionroles.items():
                    frame.actionroles[role].type = list(set([o.type for o in self.worldmodel.getall(obj)]))
                    # out(frame.toplan(), frame.actionroles[role].type)
                    if not frame.actionroles[role].type:
                        if not frame.mandatory:
                            out('skipping frame', frame.toplan(), 'because it is not mandatory')
                            skipframe = True
                            break
                        else:
                            raise Exception(str(frame.toplan()) + ' is not executable.')
                if skipframe:
                    continue
                queue = [frame]
                while queue:
                    f = queue.pop(0)
                    modified = False
                    for role, obj in f.actionroles.items():
                        if type(obj.type) is list:
                            f_ = None
                            if len(obj.type) == 1:
                                obj.type = obj.type[0]
                            else:
                                f_ = f.copy()
                                f_.actionroles[role].type = obj.type.pop()
                                modified = True
                        if modified:
                            queue.append(f)
                            queue.append(f_)
                            break
                    if not modified:
                        newframes.append(f)
            return InstructionsResponse(json.dumps(toplan(newframes, 'json')))
        except Exception as e:
            traceback.print_exc()
            return InstructionsResponse(json.dumps([]))
            # return InstructionsResponse(json.dumps({'error': type(e).__name__, 'reason': str(e)}))

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
            self.prac.tell(request.howto, request.steps, save=request.save)
            return self.prac_query(PseudoRequest(json.dumps({'request': {'instructions': [request.howto]}})))
        except Exception as e:
            traceback.print_exc()
            return InstructionsResponse(json.dumps({'error': type(e).__name__, 'reason': str(e)}))

    def serve_forever(self):
        rospy.Service('pracquery', Instructions, self.prac_query)
        rospy.Service('practell', Instructions, self.prac_tell)
        print "Waiting for instructions to interpret..."
        rospy.spin()

    def props_from_facts(self, fact):
        if fact in ('empty', 'full'):
            return {'fill_level': {'empty': 'empty.a.01', 'full': 'full.a.01'}[fact]}
        elif fact in ('used', 'unused'):
            return {'used_state': {'used': 'secondhand.s.01', 'unused': 'unused.s.01'}[fact]}
        else:
            return {}

    def update_worldmodel(self, data):
        wm = Worldmodel(self.prac)
        for obj in RStorage(json.loads(data.data)).world:
            obj = RStorage(obj)
            if obj.wordnet is None:
                continue
            wm.add(Object(self.prac, obj.name, obj.wordnet, self.props_from_facts(obj.get('fact'))))
        self.worldmodel = wm
        wmlogger.debug('---------------\nupdated world model:')
        for obj in self.worldmodel.available.values():
            wmlogger.debug(obj.toplan())
        wmlogger.debug('---------------')



logs.loggers({
    '/pracserver/worldmodel': logs.newlogger(logs.console, level=logs.INFO),
    '/prac/grounding': logs.newlogger(logs.console, level=logs.DEBUG),
})

if __name__ == "__main__":
    server = PRACServer()
    server.serve_forever()
