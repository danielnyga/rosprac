# 
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

from rosmln.msg import *
from mln.database import Database

def rosMsgFromDatabases(*db):
    '''
    Helper function that converts an MLN Database object into
    a rosmln/MLNDatabase ROS message. Returns a generator yielding
    the MLNDatabase objects.
    '''
    for db in db:
        db_msg = MLNDatabase()
        atom_prob_pairs = []
        for atom, value in dbs.evidence.iteritems():
            pair = AtomProbPair()
            pair.atom = atom
            pair.prob = value
            atom_prob_pairs.append(pair)
        db_msg.results = atom_prob_pairs
        yield db_msg
        
        
def DatabasesFromROSMsg(mln, *msgs):
    '''
    Helper function that converts a rosmln/MLNDatabase message into
    an MLN Database object. Returns a generator yielding
    the ROS Message objects.
    '''
    if msgs is None:
        return
    for msg in msgs:
        db = Database(mln)
        for truth_atom_pair in msg.evidence:
            db.addGroundAtom(truth_atom_pair.atom, truth_atom_pair.prob)
        yield db
    
    
    