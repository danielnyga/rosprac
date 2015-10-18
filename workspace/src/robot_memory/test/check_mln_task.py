#!/usr/bin/python2

import functools
from pracmln.mln.methods import LearningMethods, InferenceMethods
from check import *

def main():
    mln = learn("task.mln", "train.db")
    tests = TestSuite()
    addTests(tests, mln)
    tests.execute()
    print_to_cmd_line(mln)
    tests.print_results()


def addTests(tests, mln):
    add_test = functools.partial(tests.add_test, mln=mln)
    cw_preds_query_error = ["duration", "currentTask", "currentTaskFinished", "currentLocation", "currentParentTask", "parameter", "perceivedObject", "nextTask", "nextTaskFinished", "usedObject", "objectProperty", "childTask"]
    add_test("error(0, Objectnotfound)",\
                   "currentTask(0, Perceive)\n"+\
                   "error={Objectnotfound, Destinationposeunreachable}\n"+\
                   "error(0, Objectnotfound) v error(o, Destinationposeunreachable)",\
                   0.9, 1.0, cw_preds_query_error)
    add_test("error(0, Destinationposeunreachable)",\
                   "currentTask(0, Perceive)\n"+\
                   "error={Objectnotfound, Destinationposeunreachable}\n"+\
                   "error(0, Objectnotfound) v error(0, Destinationposeunreachable)",\
                   0.0, 0.1, cw_preds_query_error)
    cw_preds_query_success = ["duration", "currentTask", "currentTaskFinished", "currentParentTask", "currentLocation", "parameter", "perceivedObject", "nextTask", "nextTaskFinished", "usedObject", "objectProperty","childTask"]
    add_test("!error(0, Objectnotfound) ^ !error(0, Destinationposeunreachable)",\
                   "error={Objectnotfound, Destinationposeunreachable}\n"+\
                   "currentTask(0, Perceive)",\
                   0.4, 0.6, cw_preds_query_success)
    add_test("!error(0, Objectnotfound) ^ !error(0, Destinationposeunreachable)",\
                   "error={Objectnotfound, Destinationposeunreachable}\n"+\
                   "currentTask(0, Pick)",\
                   0.9, 1.0, cw_preds_query_success)
    cw_preds_query_child_task = ["duration", "currentTaskFinished", "currentLocation", "currentParentTask", "nextTask", "nextTaskFinished", "perceivedObject", "usedObject", "objectProperty"]
    add_test("childTask(0, Perceive)",\
                   "currentTask(0, Displace)",\
                   0.9, 1.0, cw_preds_query_child_task)
    add_test("childTask(0, Place)",\
                   "currentTask(0, Displace)",\
                   0.9, 1.0, cw_preds_query_child_task)
    add_test("childTask(0, Move)",\
                   "currentTask(0, Displace)",\
                   0.9, 1.0, cw_preds_query_child_task)
    add_test("childTask(0, Pick)",\
                   "currentTask(0, Displace)",\
                   0.9, 1.0, cw_preds_query_child_task)
    add_test("childTask(0, Displace)",\
                   "currentTask(0, Displace)",\
                   0.0, 0.1, cw_preds_query_child_task)
    cw_preds_query_parameter = ["duration", "currentTask", "currentTaskFinished", "currentLocation", "currentParentTask", "error", "nextTask", "nextTaskFinished", "perceivedObject", "usedObject", "objectProperty", "childTask"]
    add_test("parameter(0, Goal, Objectatlocation)",\
                   "currentTask(0, Displace)",\
                   0.9, 1.0, cw_preds_query_parameter)
    add_test("parameter(0, Goal, Objectplaced)",\
                   "currentTask(0, Displace)",\
                   0.0, 0.1, cw_preds_query_parameter)
    cw_preds_query_current_task = ["currentTaskFinished", "currentLocation", "currentParentTask", "error", "parameter", "perceivedObject", "nextTask", "nextTaskFinished", "usedObject", "objectProperty", "childTask"]
    add_test("currentTask(0, Displace)",\
                   "parameter(0, Goal, Objectatlocation)",\
                   0.9, 1.0, cw_preds_query_current_task)
    add_test("currentTask(0, Displace)",\
                   "parameter(0, Goal, Objectpicked)",\
                   0.0, 0.1, cw_preds_query_current_task)
    cw_preds_query_duration = ["currentTask", "currentTaskFinished", "currentLocation", "currentParentTask", "error", "parameter", "perceivedObject", "nextTask", "nextTaskFinished", "usedObject", "objectProperty", "childTask"]
    add_test("duration(0, Short)",\
                   "currentTask(0, Perceive)",\
                   0.9, 1.0, cw_preds_query_duration)
    add_test("duration(0, Medium)",\
                   "currentTask(0, Perceive)",\
                   0.0, 0.1, cw_preds_query_duration)
    add_test("duration(0, Medium)",\
                   "currentTask(0, Move)",\
                   0.4, 0.6, cw_preds_query_duration)


if __name__ == "__main__":
    main()
