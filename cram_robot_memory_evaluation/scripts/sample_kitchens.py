#!/usr/bin/env python

import math
import random
import argparse
import csv
from functools import reduce


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dpmin", nargs="?", default=0, type=int, help="minimum number of dummy properties per object")
    parser.add_argument("--dpmax", nargs="?", default=2, type=int, help="maximum number of dummy properties per object")
    parser.add_argument("--dkpop", nargs="?", default=5, type=int, help="population of dummy keys")
    parser.add_argument("--dvpop", nargs="?", default=5, type=int, help="population of dummy values")
    parser.add_argument("--kitchens", nargs="?", default=10, type=int, help="number of kitchens to sample")
    parser.add_argument("--objects", nargs="?", default=10, type=int, help="number of objects per kitchen")
    parser.add_argument("--locations", nargs="?", default="locations.csv",
                        type=str, help="csv file specifying the existing locations")
    parser.add_argument("--objtypes", nargs="?", default="objects.csv",
                        type=str, help="csv file specifying the objects")
    parser.add_argument("--outdir", nargs="?", default="kitchens", type=str, help="output directory")
    r = vars(parser.parse_args())
    sampler = SimpleKitchenSampler(r["dpmin"], r["dpmax"], r["dkpop"], r["dvpop"], r["objtypes"], r["locations"])
    sample_kitchens(sampler, r["kitchens"], r["objects"], r["outdir"])


def sample_kitchens(kitchen_sampler, number_of_kitchens, objects_per_kitchen, output_directory):
    for i in range(0, number_of_kitchens):
        kitchen = LISPConverter.convert_to_lisp(kitchen_sampler.sample_kitchen_objects(objects_per_kitchen))
        digits = 1+int(math.log10(number_of_kitchens))
        filename = ("{0}/kitchen-{1:" + str(digits) + "d}.lisp").format(output_directory, i)
        with open(filename, "w") as f:
            f.write(kitchen)


class SimpleKitchenSampler(object):
    def __init__(self, min_num_dummy_props, max_num_dummy_props, dummy_key_population,
                 dummy_value_population, object_type_file, location_file):
        self.__random = random.Random()
        self.__random.seed(0)
        self.__min_num_dummy_props = min_num_dummy_props
        self.__max_num_dummy_props = max_num_dummy_props
        self.__dummy_keys = ["train-dummy-key-" + str(i) for i in range(0, dummy_key_population)]
        self.__dummy_values = ["train-dummy-val-" + str(i) for i in range(0, dummy_value_population)]
        self.__locations = self.__get_kitchen_locations(location_file)
        self.__object_types = self.__get_kitchen_object_types(object_type_file)

    def sample_kitchen_objects(self, number_of_objects):
        return [KitchenObject(self.__random.choice(self.__object_types),
                              self.__sample_dummy_properties(),
                              self.__random.choice(self.__locations))
                for _ in range(0, number_of_objects)]

    def __get_kitchen_locations(self, location_file_name):
        with open(location_file_name, "r") as file:
            loc_lists = [list(line) for line in csv.reader(file, delimiter=";")]
            if not all(len(columns)==3 for columns in loc_lists):
                raise Exception("The location file must have three columns!")
            locations = [KitchenLocation(l[0], l[1], l[2]) for l in loc_lists]
        for location in locations:
            if len(list(filter(lambda l: l.location_type == location.location_type, locations))) == 1:
                location.location_type_is_unique = True
        return locations

    def __get_kitchen_object_types(self, object_file):
        with open(object_file, "r") as file:
            return [list(line)[0] for line in csv.reader(file)]

    def __sample_dummy_properties(self):
        number_of_properties = self.__random.randint(self.__min_num_dummy_props, self.__max_num_dummy_props)
        return {self.__random.choice(self.__dummy_keys): self.__random.choice(self.__dummy_values)
                for _ in range(0, number_of_properties)}


class KitchenLocation(object):
    def __init__(self, preposition, location_type, location_name):
        self.preposition = preposition
        self.location_type = location_type
        self.location_name = location_name
        self.location_type_is_unique = False

    def __repr__(self):
        return str(vars(self))


class KitchenObject:
    def __init__(self, object_type, dummy_properties, location):
        self.object_type = object_type
        self.location = location
        self.dummy_properties = dummy_properties

    def __repr__(self):
        return str(vars(self))


class LISPConverter:
    make_designator_str = "(cram-designators::make-designator "

    @staticmethod
    def convert_to_lisp(kitchen_objects):
        return reduce(lambda rest, o: rest+"\n,"+ LISPConverter.convert_object_designator(o),kitchen_objects,"`(")+")"

    @staticmethod
    def convert_object_designator(kitchen_object):
        return LISPConverter.make_designator_str + ":object `(" +\
               "(:type :" + kitchen_object.object_type + ")" +\
               reduce(lambda rest, kv: rest + "(:"+kv[0]+" "+kv[1]+")", kitchen_object.dummy_properties.items(),"") +\
               "(:at ," + LISPConverter.convert_location_designator(kitchen_object.location) + ")))"

    @staticmethod
    def convert_location_designator(object_location):
        return LISPConverter.make_designator_str + ":location '(" +\
               "(:" + object_location.preposition + " \"" + object_location.location_type + "\")" +\
               "(:name \"" + object_location.location_name + "\")))"


if __name__ == "__main__":
    main()