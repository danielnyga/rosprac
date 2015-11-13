#!/usr/bin/python2

import functools
from pracmln.mln.methods import LearningMethods, InferenceMethods
from check import *

def main():
    mln = open_mln("learnt_mlns/Task.mln")
    tests = TestSuite()
    addTests(tests, mln)
    tests.execute()
    print_to_cmd_line(mln)
    tests.print_results()


def addTests(tests, mln):
    add_test = functools.partial(tests.add_test, mln=mln)
    cw_preds_query_error = ["duration", "currentTask", "currentTaskFinished", "currentLocation", "currentParentTask", "currentParameter", "nextParameter", "parentParameter" "perceivedObject", "nextTask", "nextTaskFinished", "usedObject", "objectProperty", "childTask"]
    add_test("error(0, Objectnotfound)",\
                   "currentTask(0, Perceive)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "error={Objectnotfound, Destinationposeunreachable}\n"+\
                   "error(0, Objectnotfound) v error(o, Destinationposeunreachable)",\
                   0.9, 1.0, [])
    add_test("error(0, Destinationposeunreachable)",\
                   "currentTask(0, Perceive)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "error={Objectnotfound, Destinationposeunreachable}\n"+\
                   "error(0, Objectnotfound) v error(0, Destinationposeunreachable)",\
                   0.0, 0.1, [])
    cw_preds_query_success = ["duration", "currentTask", "currentTaskFinished", "currentParentTask", "currentLocation", "currentParameter", "nextParameter", "parentParameter" "perceivedObject", "nextTask", "nextTaskFinished", "usedObject", "objectProperty","childTask"]
    add_test("!error(0, Objectnotfound) ^ !error(0, Destinationposeunreachable)",\
                   "error={Objectnotfound, Destinationposeunreachable}\n"+\
                   "currentTaskFinished(0)\n"+\
                   "currentTask(0, Perceive)",\
                   0.4, 0.6, [])
    add_test("!error(0, Objectnotfound) ^ !error(0, Destinationposeunreachable)",\
                   "error={Objectnotfound, Destinationposeunreachable}\n"+\
                   "currentTaskFinished(0)\n"+\
                   "currentTask(0, Pick)",\
                   0.9, 1.0, [])
    cw_preds_query_child_task = ["currentTaskFinished", "currentLocation", "nextTask", "nextTaskFinished", "perceivedObject", "usedObject", "objectProperty"]
    add_test("childTask(0, Perceive)",\
                   "currentTask(0, Displace)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "currentParameter(0, Goal, Objectatlocation)",\
                   0.9, 1.0, [])
    add_test("childTask(0, Place)",\
                   "currentTask(0, Displace)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "currentParameter(0, Goal, Objectatlocation)",\
                   0.9, 1.0, [])
    add_test("childTask(0, Move)",\
                   "currentTask(0, Displace)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "currentParameter(0, Goal, Objectatlocation)",\
                   0.9, 1.0, [])
    add_test("childTask(0, Pick)",\
                   "currentTask(0, Displace)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "currentParameter(0, Goal, Objectatlocation)",\
                   0.9, 1.0, [])
    add_test("childTask(0, Displace)",\
                   "currentTask(0, Displace)\n"+\
                   "currentTaskFinished(0)\n"+\
                   "currentParameter(0, Goal, Objectatlocation)",\
                   0.0, 0.1, [])
    cw_preds_query_duration = ["currentTask", "currentTaskFinished", "currentLocation", "currentParentTask", "error", "currentParameter", "nextParameter", "parentParameter" "perceivedObject", "nextTask", "nextTaskFinished", "usedObject", "objectProperty", "childTask"]
    add_test("duration(0, Short)",\
                   "currentTaskFinished(0)\n"+\
                   "currentTask(0, Perceive)",\
                   0.9, 1.0, [])
    add_test("duration(0, Medium)",\
                   "currentTaskFinished(0)\n"+\
                   "currentTask(0, Perceive)",\
                   0.0, 0.1, [])
    add_test("duration(0, Medium)",\
                   "currentTaskFinished(0)\n"+\
                   "currentTask(0, Move)",\
                   0.4, 0.6, [])


if __name__ == "__main__":
    main()
