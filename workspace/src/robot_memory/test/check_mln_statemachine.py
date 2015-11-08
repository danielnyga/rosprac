#!/usr/bin/python2

import functools
from pracmln.mln.methods import LearningMethods, InferenceMethods
from check import *

def main():
    mln = open_mln("learnt_mlns/StateMachine.mln")
    tests = TestSuite()
    addTests(tests, mln)
    tests.execute()
    print_to_cmd_line(mln)
    tests.print_results()


def addTests(tests, mln):
    add_test = functools.partial(tests.add_test, mln=mln)
    cw_preds_query_next_task = ["currentParameter", "currentTaskFinished", "currentLocation", "currentParentTask", "error", "goal", "perceivedObject", "usedObject", "objectProperty", "parentParameter"]
    add_test("nextTask(0, Perceive)",\
                   "currentTask(0, Displace)\n"+\
                   "currentParameter(0, Goal, Objectatlocation)\n"+\
                   "currentParentTask(0, Start)",\
                   0.9, 1.0, cw_preds_query_next_task)
    add_test("nextTask(0, Place)",\
                   "currentTask(0, Displace)\n"+\
                   "currentParameter(0, Goal, Objectatlocation)\n"+\
                   "currentParentTask(0, Start)",\
                   0.0, 0.1, cw_preds_query_next_task)
    add_test("nextTask(0, Pick)",\
                   "currentTask(0, Perceive)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "parentParameter(0, Goal,Objectatlocation)\n"+\
                   "currentParentTask(0, Displace)",\
                   0.9, 1.0, cw_preds_query_next_task)
    add_test("nextTask(0, Perceive)",\
                   "error(0, Objectnotfound)\n"+\
                   "currentTask(0, Perceive)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "parentParameter(0, Goal,Objectatlocation)\n"+\
                   "currentParentTask(0, Displace)",\
                   0.9, 1.0, cw_preds_query_next_task)
    add_test("nextTask(0, Move)",\
                   "currentTask(0, Pick)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "currentParameter(0, Goal,Objectpicked)\n"+\
                   "parentParameter(0, Goal,Objectatlocation)\n"+\
                   "currentParentTask(0, Displace)",\
                   0.9, 1.0, cw_preds_query_next_task)
    add_test("nextTask(0, Place)",\
                   "currentTask(0, Move)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "parentParameter(0, Goal,Objectatlocation)\n"+\
                   "currentParentTask(0, Displace)",\
                   0.4, 0.6, cw_preds_query_next_task)


if __name__ == "__main__":
    main()
