#!/usr/bin/env python

import sys
import csv


class Columns(object):
    object_name = 0
    success = 1
    estimated_success = 2
    tries = 3


class Statistics:
    def __init__(self, results):
        successful = list(filter(lambda l: l[Columns.success] == "T", results))
        unsuccessful = list(filter(lambda l: l[Columns.success] == "NIL", results))
        assert (len(successful) + len(unsuccessful) == len(results))
        self.grasp_success = len(successful) / len(results)
        self.estimated_success = sum([float(l[Columns.estimated_success]) for l in results]) / len(results)
        self.successful_tries = None if len(successful) == 0 \
            else sum([float(l[Columns.tries]) for l in successful]) / len(successful)
        self.unsuccessful_tries = None if len(unsuccessful) == 0 \
            else sum([float(l[Columns.tries]) for l in unsuccessful]) / len(unsuccessful)
        self.correctly_guessed = len([l for l in results if
                                      float(l[Columns.estimated_success]) > 0.5 and l[Columns.success] == "T" or
                                      float(l[Columns.estimated_success]) < 0.5 and l[Columns.success] == "NIL"]) \
                                 / len(results)

    def __str__(self):
        return "{0:50}{1}\n".format("Successful grasps (%):", self.grasp_success) + \
               "{0:50}{1}\n".format("Average estimated success rate (%):", self.estimated_success) + \
               "{0:50}{1}\n".format("Average tries for successful grasps (%):", self.successful_tries) + \
               "{0:50}{1}\n".format("Average tries for unsuccessful grasps (%):", self.unsuccessful_tries) + \
               "{0:50}{1}\n".format("Correctly guessed success (threshold: 0.5):", self.correctly_guessed)

def main(completion_file_name, comparison_file_name):
    with open(completion_file_name, "r") as completion_file, open(comparison_file_name, "r") as comparison_file:
        completion_stats = Statistics([list(line) for line in csv.reader(completion_file, delimiter=";")])
        comparison_stats = Statistics([list(line) for line in csv.reader(comparison_file, delimiter=";")])
        assert (abs(completion_stats.estimated_success - comparison_stats.estimated_success) < 0.0001)
        print("completion results:")
        print("===================")
        print(str(completion_stats))
        print("comparison results:")
        print("===================")
        print(str(comparison_stats))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Two arguments are required: the completion file name and the comparison file name!")
    else:
        main(sys.argv[1], sys.argv[2])
