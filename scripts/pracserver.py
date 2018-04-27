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
import rosparam
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
    world_model_channel = None
    to_prac_channel = None
    from_prac_channel = None
    augment_prac_channel = None

    def make_dummy_worldmodel(self):
        # poopulate the world model manually
        wm = Worldmodel(self.prac, cw=True)
        wm.add(Object(self.prac, 
                      'juice', 
                      'carton.n.02', 
                      props={'fill_level': 'empty.a.01'}))
        wm.add(Object(self.prac, 'basket', 'basket.n.01'))
        wm.add(Object(self.prac, 'fridge', 'electric_refrigerator.n.01'))
        wm.add(Object(self.prac, 'trash', 'ashcan.n.01'))
        wm.add(Object(self.prac, 'milk-box-full', 'carton.n.02', 
                      props={'fill_level': 'full.a.01'}))
        wm.add(Object(self.prac, 'cereals-box', 'carton.n.02', 
                      props={'used_state': 'secondhand.s.01'}))
        wm.add(Object(self.prac, 'banana', 'banana.n.02'))
        wm.add(Object(self.prac, 'apple', 'apple.n.01'))
        wm.add(Object(self.prac, 'orange', 'orange.n.01'))
        return wm

    def __init__(self):
        self.prac = PRAC()
        self.prac.verbose = verbose

        rospy.init_node('pracserver')
        rospy.Subscriber(PRACServer.world_model_channel, 
                         String, 
                         self.update_worldmodel)
        rospy.Subscriber(PRACServer.to_prac_channel, 
                         String, 
                         self.prac_query)
        rospy.Subscriber(PRACServer.augment_prac_channel, 
                         String, 
                         self.prac_tell)
        self.from_prac_pub = rospy.Publisher(
            PRACServer.from_prac_channel,
            String,
            queue_size=10)

    def prac_query(self, r):
        request = json.loads(r.data)
        print 'PRAC queried with', request
        if not hasattr(self, 'worldmodel'):
            self.worldmodel = self.make_dummy_worldmodel()
            wmlogger.warning('no worldmodel coming in from the topic.'
                             'using dummy worldmodel.')
        infer = PRACInference(self.prac,
                              request['instructions'], 
                              self.worldmodel,
                              similarity=.8)
        grounding = self.prac.module('grounding')
        try:
            infer.run()
            gndframes = grounding(infer, self.worldmodel)
            returnval = json.dumps(toplan(gndframes, 'json'))
        except Exception as e:
            traceback.print_exc()
            returnval = json.dumps([])

        print 'PRAC replies', returnval
        self.from_prac_pub.publish(returnval)

    def prac_tell(self, r):
        '''
        The ``r`` is expected to be a JSON encoded dictionary string containing 
        the following elements:

        {
            "howto": "Make breakfast.",
            "steps": [
                "Prepare cereals.",
                "Boil an egg."
                "Put milk on the table."
            ]
            "save": true,
        }

        If ``save`` is ``true``, the howto is persistently stored in 
        the PRAC database.

        :param r:
        :return:
        '''
        request = json.loads(r.data)
        print 'PRAC received request:', request
        try:
            out(request['howto'], request['steps'], save=request['save'])
            self.prac.tell(request['howto'], 
                           request['steps'], 
                           save=request['save'])
            prac_query_string = String(data=json.dumps(
                {'instructions': [request['howto']]}))
            self.prac_query(prac_query_string)
        except Exception as e:
            traceback.print_exc()
            print 'PRAC replied []'
            self.from_prac_pub.publish(json.dumps([]))

    def serve_forever(self):
        print "Waiting for instructions to interpret..."
        rospy.spin()

    def props_from_facts(self, fact):
        if fact in ('empty', 'full'):
            return {'fill_level': {'empty': 'empty.a.01', 
                                   'full': 'full.a.01'}[fact]}
        elif fact in ('used', 'unused'):
            return {'used_state': {'used': 'secondhand.s.01', 
                                   'unused': 'unused.s.01'}[fact]}
        else:
            return {}

    def update_worldmodel(self, data):
        wm = Worldmodel(self.prac)
        for obj in RStorage(json.loads(data.data)).world:
            obj = RStorage(obj)
            if obj.wordnet is None:
                continue
            wm.add(Object(self.prac, obj.name, obj.wordnet, 
                          self.props_from_facts(obj.get('fact'))))
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
    PRACServer.world_model_channel = rosparam.get_param('world_channel')
    PRACServer.to_prac_channel = rosparam.get_param('to_prac')
    PRACServer.from_prac_channel = rosparam.get_param('from_prac')
    PRACServer.augment_prac_channel = rosparam.get_param('augment_prac')
    server = PRACServer()
    server.serve_forever()
