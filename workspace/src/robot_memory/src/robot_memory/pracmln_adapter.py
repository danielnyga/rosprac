from pracmln.mln.base import parse_mln
from pracmln.mln.methods import LearningMethods, InferenceMethods
from pracmln.mln.database import Database, parse_db
from StringIO import StringIO

LOGIC = "FirstOrderLogic"
GRAMMAR = "StandardGrammar"
LEARNING_METHOD = LearningMethods.BPLL


def learn(mln, databases):
    """
    Learns the weights of the given MLN.
    :param mln: The MLN whose weights should be learnt as string (i.e. read from a .mln file)
    :type mln: str
    :param databases: The databases to use as string (i.e. read from a .db file)
    :type databases: str
    :return: The learnt MLN as string (writable to a .mln file)
    :rtype: str
    """
    mln_to_learn = parse_mln(text=mln, logic=LOGIC, grammar=GRAMMAR)
    dbs = parse_db(mln_to_learn, databases)
    learned_mln = mln_to_learn.learn(dbs, LEARNING_METHOD)
    mln_as_string = StringIO()
    learned_mln.write(mln_as_string)
    return mln_as_string.getvalue()