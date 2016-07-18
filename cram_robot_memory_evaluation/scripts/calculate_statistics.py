#!/usr/bin/env python

import sys
import os.path
import csv


class Indices(object):
    informed_object_type = 0
    informed_success = 1
    informed_tries = 2
    mln_completion_object_type = 3
    mln_completion_success = 4
    mln_completion_tries = 5
    naive_completion_object_type = 6
    naive_completion_success = 7
    naive_completion_tries = 8
    theoretical_object_type = 9
    theoretical_object_location_name = 10
    theoretical_success_estimate = 11
    theoretical_object_position = 12
    theoretical_naive_success = 13
    theoretical_naive_tries = 14
    theoretical_mln_success = 15
    theoretical_mln_tries = 16


class Statistics:
    def __init__(self, data, success_index=None, tries_index=None, estimated_success_index=None, location_index=None):
        self.__data = data
        self.__success_index = success_index
        self.__tries_index = tries_index
        self.__estimated_success_index = estimated_success_index
        self.__location_index = location_index

    def print_success_rate(self):
        successful = list(filter(lambda l: l[self.__success_index] == "T", self.__data))
        grasp_success = len(successful) * 1.0 / len(self.__data)
        print("{0:60}{1}".format("Successful grasps (%):", grasp_success))

    def print_tries(self):
        successful = list(filter(lambda l: l[self.__success_index] == "T", self.__data))
        unsuccessful = list(filter(lambda l: l[self.__success_index] == "NIL", self.__data))
        successful_tries = None if len(successful) == 0 \
            else (sum([float(l[self.__tries_index]) for l in successful]) * 1.0 / len(successful))
        unsuccessful_tries = None if len(unsuccessful) == 0 \
            else (sum([float(l[self.__tries_index]) for l in unsuccessful]) * 1.0 / len(unsuccessful))
        print("{0:60}{1}".format("Average tries for successful grasps:", successful_tries) )
        print("{0:60}{1}".format("Average tries for unsuccessful grasps:", unsuccessful_tries))

    def print_estimate(self):
        estimated_success = sum([float(l[self.__estimated_success_index]) for l in self.__data]) / len(self.__data)
        correctly_guessed = (len([l for l in self.__data if
                                 float(l[self.__estimated_success_index]) > 0.5 and l[self.__success_index] == "T" or
                                 float(l[self.__estimated_success_index]) < 0.5 and l[self.__success_index] == "NIL"]))\
                                 * 1.0 / len(self.__data)
        print("{0:60}{1}".format("Average estimated success rate (%):", estimated_success))
        print("{0:60}{1}".format("Correctly guessed success assuming threshold 0.5 (%):", correctly_guessed))

    def print_location_rate(self):
        successful = [float(l[self.__location_index]) for l in self.__data if l[self.__location_index] != "NIL"]
        location_success = float(len(successful)) / len(self.__data)
        average_position = None if len(successful)==0 else (sum(successful) / len(successful))
        print("{0:60}{1}".format("Successfully guessed objects for a location (%):", location_success))
        print("{0:60}{1}".format("Average rank:", average_position))


def main(csv_file_directory):
    csv_files = ["informed.csv", "mln_completion.csv", "naive_completion.csv", "theoretical.csv"]
    data = read_csv_files(csv_file_directory, csv_files)
    print_informed_statistics(data)
    print_mln_completion_results(data)
    print_naive_completion_results(data)
    print_theoretical_mln_completion_results(data)
    print_theoretical_naive_results(data)


def print_informed_statistics(data):
    print("")
    print("==================================")
    print("FULLY INFORMED OBJECT DISPLACEMENT")
    print("==================================")
    print("")
    stats = Statistics(data, success_index=Indices.informed_success,
                       estimated_success_index=Indices.theoretical_success_estimate,
                       location_index=Indices.theoretical_object_position)
    stats.print_success_rate()
    stats.print_estimate()
    stats.print_location_rate()


def print_mln_completion_results(data):
    print("")
    print("==============")
    print("MLN COMPLETION")
    print("==============")
    print("")
    stats = Statistics(data, success_index=Indices.mln_completion_success, tries_index=Indices.mln_completion_tries)
    stats.print_success_rate()
    stats.print_tries()


def print_naive_completion_results(data):
    print("")
    print("================")
    print("NAIVE COMPLETION")
    print("================")
    print("")
    stats = Statistics(data, success_index=Indices.naive_completion_success, tries_index=Indices.naive_completion_tries)
    stats.print_success_rate()
    stats.print_tries()


def print_theoretical_mln_completion_results(data):
    print("")
    print("==========================")
    print("MLN COMPLETION (IN THEORY)")
    print("==========================")
    print("")
    stats = Statistics(data, success_index=Indices.theoretical_mln_success, tries_index=Indices.theoretical_mln_tries)
    stats.print_success_rate()
    stats.print_tries()


def print_theoretical_naive_results(data):
    print("")
    print("============================")
    print("NAIVE COMPLETION (IN THEORY)")
    print("============================")
    print("")
    stats = Statistics(data, success_index=Indices.theoretical_naive_success,
                       tries_index=Indices.theoretical_naive_tries)
    stats.print_success_rate()
    stats.print_tries()


def read_csv_files(directory, file_names):
    to_return = []
    for file_name in file_names:
        with open(os.path.join(directory, file_name)) as csv_file:
            to_return.append([list(line) for line in csv.reader(csv_file, delimiter=";")])
    return [[column for lst in line for column in lst] for line in zip(*to_return)]


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("One argument is required: the folder containing the csv files generated during the evaluation!")
    else:
        main(sys.argv[1])
