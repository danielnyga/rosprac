#!/usr/bin/env python
#
# (C) 2011-2014 by Daniel Nyga (nyga@cs.uni-bremen.de)
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

#!/usr/bin/env python
import roslib; roslib.load_manifest('rosprac')

import sys

import rospy
from rosprac.srv import *
from rospracutils import rosMsgFromDatabases, DatabasesFromROSMsg
from rosmln.msg import *
import os

PROBCOG_HOME = os.environ.get('PROBCOG_HOME')
sys.path.append(os.path.join(PROBCOG_HOME, 'python'))




def pracinfer_client():
    rospy.wait_for_service('PRACInfer')
    try:
        pracinfer = rospy.ServiceProxy('PRACInfer', PRACInfer)
        
#         result_dbs = pracinfer('nl_parsing', None, None, ['Add some water.'])
#         result_dbs = pracinfer('ac_recognition', None, result_dbs.output_dbs, None)
        
        e = '''action_role(Flip-1, ActionVerb)
        action_role(pancake-3, Theme)
        has_sense(Flip-1, flip-1-8)
        is_a(flip-1-8, flip.v.08)
        !is_a(flip-1-8, pancake.n.01)
        is_a(pancake-3-1, pancake.n.01) 
        !is_a(pancake-3-1, flip.v.08) 
        has_sense(pancake-3, pancake-3-1)
        action_core(Flip-1, Flipping)
        '''
        rosMsg = MLNDatabase()
        atoms = e.split('\n')
        for a in atoms:
            if a.strip() == '': continue
            atomValue = AtomProbPair()
            atomValue.atom = a.strip()
            atomValue.prob = 1.0
            rosMsg.evidence.append(atomValue)
        
        result_dbs = pracinfer('senses_and_roles', None, [rosMsg], None)

        print result_dbs
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print "Requesting PRAC..."
    pracinfer_client()
    
    