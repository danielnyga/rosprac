#!/usr/bin/env python
# PROBABILISTIC ROBOT ACTION CORES - ROS INTERFACE 
#
# (C) 2013 by Daniel Nyga (nyga@cs.tum.edu)
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import roslib; roslib.load_manifest('rosprac')

from rosprac.srv import *
from rosprac.msg import *
import rospy

import java
import os
import jpype
from actioncore import PRAC
from actioncore.inference import *


prac = PRAC()

def pracinfer_handler(param):
    print 'Running PRAC Inference on sentence "%s"' % (param.instruction)
    
    if not jpype.isThreadAttachedToJVM():
        jpype.attachThreadToJVM()
    pracinit = PRACInit(param.action_core)
    result = PRACResult()
    pracinit(param.instruction) >> actionroles >> result 
    roles = []
    
    for bind in result.pracinference.databases['core'].query('action_role(?w,?r) ^ has_sense(?w, ?s) ^ !is_a(?s, NULL)'):
        feature = result.pracinference.features.get('wordsenses')
        senses = feature.words2senses[bind['?w']]
        tick = senses.index(bind['?s'])
        roles.append(action_role(bind['?r'], map(lambda x: feature.senses2concepts[x], senses)[tick]))
    for role in result.pracinference.features.get('missingroles').missingRoles:
        for bind in result.pracinference.databases['missingroles'].query('action_role(?w,%s) ^ has_sense(?w, ?s) ^ is_a(?s,?c)' % role):
            roles.append(action_role(role, bind['?c']))
            
    return (roles,)
#    for bind in result.pracinference.databases['core'].query('action_role(?w,?r) ^ has_sense(?w,?s) ^ is_a(?s,?c) ^ !(?r=NULL)'):
#    return ([action_role('ActionVerb', 'flip.v.01'), action_role('Theme', 'pancake.n.01'), action_role('Instrument', 'spatula.n.01')],)

def prac_server():
    rospy.init_node('pracinfer')
    s = rospy.Service('pracinfer', pracinfer, pracinfer_handler)
    print "PRAC service is running. Waiting for queries..."
    rospy.spin()

if __name__ == "__main__":
    java.startJvm()
    prac_server()
    java.shutdownJvm()