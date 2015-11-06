#!/usr/bin/python2

from pracmln.mln.methods import LearningMethods, InferenceMethods
from check import *
import check_mln_statemachine
import check_mln_object
import check_mln_task


def main(learn_weights=True):
    output_prefix = "learnt_mlns/"
    mln_names = ["StateMachine", "Task", "Object"]
    mlns = {}
    for mln_name in mln_names:
        if learn_weights:
            mlns[mln_name] = learn(output_prefix+mln_name+"_before_learning.mln", output_prefix+"train.db")
        else:
            mlns[mln_name] = open_mln(output_prefix+mln_name+".mln")
    tests = TestSuite()
    check_mln_task.addTests(tests, mlns["Task"])
    check_mln_statemachine.addTests(tests, mlns["StateMachine"])
    check_mln_object.addTests(tests, mlns["Object"])
    tests.execute()
    for mln_name in mlns:
        print_to_cmd_line(mlns[mln_name])
    tests.print_results()


if __name__ == "__main__":
    main()
