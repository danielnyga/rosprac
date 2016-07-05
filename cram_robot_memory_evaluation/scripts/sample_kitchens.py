#!/usr/bin/env python

import math
import random
import argparse
import csv
from functools import reduce


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dpmin", nargs="?", default=0, type=int, help="minimum number of test dummy obj properties")
    parser.add_argument("--dpmax", nargs="?", default=2, type=int, help="maximum number of test dummy obj properties")
    parser.add_argument("--dkpop", nargs="?", default=5, type=int, help="population of dummy keys")
    parser.add_argument("--dvpop", nargs="?", default=5, type=int, help="population of dummy values")
    parser.add_argument("--train-kitchens", nargs="?", default=10, type=int, help="sampled kitchens for training")
    parser.add_argument("--test-kitchens", nargs="?", default=10, type=int, help="sampled kitchens for test")
    parser.add_argument("--objects", nargs="?", default=10, type=int, help="number of objects per kitchen")
    parser.add_argument("--locations", nargs="?", default="locations.csv",
                        type=str, help="csv file specifying the existing locations")
    parser.add_argument("--objtypes", nargs="?", default="objects.csv",
                        type=str, help="csv file specifying the objects")
    parser.add_argument("--outdir", nargs="?", default="kitchens", type=str, help="output directory")
    r = vars(parser.parse_args())
    sampler = SimpleKitchenSampler(r["dkpop"], r["dvpop"], r["objtypes"], r["locations"])
    sample_kitchens(sampler, r["train_kitchens"], r["test_kitchens"], r["objects"], r["dpmin"], r["dpmax"], r["outdir"])


def sample_kitchens(kitchen_sampler, train_kitchens, test_kitchens, objects_per_kitchen,
                    min_test_dummy_props, max_test_dummy_props, output_directory):
    for prefix, number_of_kitchens, min_dummy, max_dummy in \
            [("training-", train_kitchens, 0, 0),
             ("test-", test_kitchens, min_test_dummy_props, max_test_dummy_props)]:
        for i in range(1, number_of_kitchens+1):
            kitchen = LISPConverter.convert_to_lisp(
                kitchen_sampler.sample_kitchen_objects(objects_per_kitchen, min_dummy, max_dummy))
            digits = 1+int(math.log10(number_of_kitchens))
            filename = ("{0}/" + prefix + "kitchen-{1:0" + str(digits) + "d}.lisp").format(output_directory, i)
            with open(filename, "w") as f:
                f.write(kitchen)


class SimpleKitchenSampler(object):
    def __init__(self, dummy_key_population,
                 dummy_value_population, object_type_file, location_file):
        self.__random = random.Random()
        self.__random.seed(0)
        self.__dummy_keys = ["dummy-key-" + str(i) for i in range(0, dummy_key_population)]
        self.__dummy_values = ["dummy-val-" + str(i) for i in range(0, dummy_value_population)]
        self.__locations = self.__get_kitchen_locations(location_file)
        self.__object_types_and_locations = self.__get_kitchen_object_types(object_type_file)

    def sample_kitchen_objects(self, number_of_objects, min_dummy_props, max_dummy_props):
        return [self.__sample_kitchen_object(min_dummy_props, max_dummy_props) for _ in range(0, number_of_objects)]

    def __get_kitchen_locations(self, location_file_name):
        with open(location_file_name, "r") as file:
            loc_lists = [list(line) for line in csv.reader(file, delimiter=";")]
            if not all(len(columns) == 8 for columns in loc_lists):
                raise Exception("The location file must have eight columns!")
            locations = [KitchenLocation(l[0],l[1],l[2],l[3],l[4],l[5],l[6],l[7],self.__random) for l in loc_lists]
        for location in locations:
            if len(list(filter(lambda l: l.location_type == location.location_type, locations))) == 1:
                location.location_type_is_unique = True
        return locations

    def __get_kitchen_object_types(self, object_file):
        with open(object_file, "r") as file:
            lines = [list(line) for line in csv.reader(file, delimiter=";")]
            if not all(len(columns) > 1 for columns in lines):
                raise Exception("The object file must have at least two columns!")
            return [(o[0], [l for l in self.__locations if l.location_type in o[1:]]) for o in lines]

    def __sample_dummy_properties(self, min_dummy_props, max_dummy_props):
        number_of_properties = self.__random.randint(min_dummy_props, max_dummy_props)
        return {self.__random.choice(self.__dummy_keys): self.__random.choice(self.__dummy_values)
                for _ in range(0, number_of_properties)}

    def __sample_kitchen_object(self, min_dummy_props, max_dummy_props):
        object_locations_pair = self.__random.choice(self.__object_types_and_locations)
        location = self.__random.choice(object_locations_pair[1])
        dummy_properties = self.__sample_dummy_properties(min_dummy_props, max_dummy_props)
        return KitchenObject(object_locations_pair[0], dummy_properties, location)


class KitchenLocation(object):
    def __init__(self, preposition, location_type, location_name, x_min, x_max, y_min, y_max, z, random_number_gen):
        self.preposition = preposition
        self.location_type = location_type
        self.location_name = location_name
        self.location_type_is_unique = False
        self.__x_min = float(x_min)
        self.__x_max = float(x_max)
        self.__y_min = float(y_min)
        self.__y_max = float(y_max)
        self.__z = float(z)
        self.__random = random_number_gen

    def __repr__(self):
        return str(vars(self))

    @property
    def position(self):
        x = self.__random.uniform(self.__x_min, self.__x_max)
        y = self.__random.uniform(self.__y_min, self.__y_max)
        pos = (x, y, self.__z)
        return pos


class KitchenObject(object):
    def __init__(self, object_type, dummy_properties, location):
        self.object_type = object_type
        self.dummy_properties = dummy_properties
        self.location = location

    def __repr__(self):
        return str(vars(self))


class LISPConverter(object):
    make_designator_str = "(cram-designators::make-designator "

    @staticmethod
    def convert_to_lisp(kitchen_objects):
        objects = ["`(" + LISPConverter.get_bullet_pose(o) + " ," +
                          LISPConverter.convert_object_designator(o, True) + " ," +
                          LISPConverter.convert_object_designator(o, False) + ")"
                   for o in kitchen_objects]
        return reduce(lambda rest, o: rest + "\n," + o, objects, "`(") + ")"

    @staticmethod
    def convert_object_designator(kitchen_object, include_location_designator):
        return LISPConverter.make_designator_str + ":object `(" +\
               "(:type :" + kitchen_object.object_type + ")" +\
               reduce(lambda rest, kv: rest+"(:"+kv[0]+" \""+kv[1]+"\")", kitchen_object.dummy_properties.items(),"") +\
               ("(:at ," + LISPConverter.convert_location_designator(kitchen_object.location) + ")))"
                if include_location_designator else "))")

    @staticmethod
    def convert_location_designator(object_location):
        return LISPConverter.make_designator_str + ":location '(" +\
               "(:" + object_location.preposition + " \"" + object_location.location_type + "\")" +\
               ("" if object_location.location_type_is_unique else "(:name \""+object_location.location_name+"\")") +\
               "))"

    @staticmethod
    def get_bullet_pose(kitchen_object):
        position = kitchen_object.location.position
        return "(({0:.4f} {1:.4f} {2:.4f})(0 0 0 1))".format(position[0], position[1], position[2])


if __name__ == "__main__":
    main()
