#!/usr/bin/python2

import functools
from pracmln.mln.methods import LearningMethods, InferenceMethods
from check import *

def main():
    mln = learn("objects.mln", "train.db")
    tests = TestSuite()
    addTests(tests, mln)
    tests.execute()
    print_to_cmd_line(mln)
    tests.print_results()


def addTests(tests, mln):
    add_test = functools.partial(tests.add_test, mln=mln)
    cw_preds_query_objectType = ["currentTask", "currentTaskFinished", "currentParentTask", "error", "goal", "nextTask", "nextTaskFinished", "usedObject", "objectProperty"]
    cw_preds_query_objectProperty = ["objectType"]
    add_test("objectProperty(O, Form, Long)",\
                    "perceivedObject(0, O)\n"+\
                    "objectType(O, Spatula)",\
                    0.4, 0.6, cw_preds_query_objectProperty)
    add_test("objectProperty(O, Color, Yellow)",\
                    "perceivedObject(0, O)\n"+\
                    "objectType(O, Spatula)",\
                    0.4, 0.6, cw_preds_query_objectProperty)
    add_test("objectProperty(O, Form, Round)",\
                    "perceivedObject(0, O)\n"+\
                    "objectType(O, Plate)",\
                    0.9, 1.0, cw_preds_query_objectProperty)   
    cw_preds_query_objectType = ["objectProperty"]
    add_test("objectType(O, Plate)",\
                    "perceivedObject(0, O)\n"+\
                    "objectProperty(O, Form, Round)",\
                    0.9, 1.0, cw_preds_query_objectType)   
    add_test("objectType(O, Spatula)",\
                    "perceivedObject(0, O)\n"+\
                    "objectProperty(O, Form, Round)",\
                    0.0, 0.1, cw_preds_query_objectType)   

if __name__ == "__main__":
    main()
