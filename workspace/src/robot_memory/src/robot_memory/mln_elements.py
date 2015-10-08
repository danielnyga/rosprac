from copy import deepcopy


class GroundAtom:
    def __init__(self, predicate, constants, negated=False, ):
        self.__predicate = predicate
        self.__constants = [list(t) for t in constants]
        self.__negated = negated

    def __invert__(self):
        return GroundAtom(self.__predicate, self.__constants, not self.__negated)

    def __str__(self):
        args = str(reduce(lambda s1, s2: str(s1) + "," + str(s2), [c[1] for c in self.__constants]))
        return ("!" if self.__negated else "") + self.__predicate.get_name() + "(" + args + ")"

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return self.__predicate.__hash__() + self.__negated.__hash__() + \
            tuple([tuple(pair) for pair in self.__constants]).__hash__()

    def __eq__(self, other):
        return (self.__predicate == other.__predicate) and (self.__negated == other.__negated) and \
               (self.__constants == other.__constants)

    def __ne__(self, other):
        return not self.__eq__(other)

    def is_negated(self):
        return self.__negated

    def get_predicate(self):
        return self.__predicate

    def set_argument_value(self, argument_type, value):
        for pair in self.__constants:
            if pair[0] == argument_type:
                pair[1] = value


class Predicate:
    def __init__(self, predicate_name, *predicate_types):
        self.__predicate_name = predicate_name
        self.__types = predicate_types

    def __call__(self, *args, **kwargs):
        if kwargs:
            raise Exception("keyword arguments are not supported!")
        if len(args) != len(self.__types):
            raise Exception("Invalid number of arguments: " + str(len(self.__types)) + "required!")
        return GroundAtom(self, zip(self.__types, args))

    def __str__(self):
        return self.__predicate_name + "(" + str(reduce(lambda t1, t2: str(t1) + "," + str(t2), self.__types)) + ")"

    def __hash__(self):
        return self.__predicate_name.__hash__() + tuple(self.__types).__hash__()

    def __eq__(self, other):
        return self.__predicate_name == other.__predicate_name and self.__types == other.__types

    def __ne__(self, other):
        return not self.__eq__(other)

    def get_name(self):
        return self.__predicate_name

    def get_types(self):
        return self.__types


class Type:
    def __init__(self, type_name, is_soft_functional=False):
        self.__type_name = type_name
        self.__is_soft_functional = is_soft_functional
        if not is_soft_functional:
            self.__soft_functional_type = Type(type_name, True)

    def __neg__(self):
        return self.__soft_functional_type

    def __str__(self):
        return self.__type_name + ("?" if self.__is_soft_functional else "")

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return self.__type_name == other.__type_name

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return self.__type_name.__hash__()

    def get_name(self):
        return self.__type_name


class DatabaseCollection:
    def __init__(self):
        self.__databases = []

    def __iter__(self):
        return iter(self.__databases)

    def __str__(self):
        return reduce(lambda db1, db2: str(db1) + "\n---\n" + str(db2), self.__databases)

    def __repr__(self):
        return self.__str__()

    def append(self, database):
        self.__databases.append(database)


class Database:
    def __init__(self):
        self.__ground_atoms = []

    def __iter__(self):
        return iter(self.__ground_atoms)

    def __str__(self):
        return reduce(lambda g1, g2: str(g1) + "\n" + str(g2), self.__ground_atoms)

    def __repr__(self):
        return self.__str__()

    def append(self, ground_atom):
        self.__ground_atoms.append(ground_atom)


class Formula:
    def __init__(self, weight, ground_atoms):
        self.__weight = weight
        self.__ground_atoms = deepcopy(ground_atoms)

    def __str__(self):
        return str(self.__weight) + " " + str(reduce(lambda g1, g2: str(g1) + " ^ " + str(g2), self.__ground_atoms))

    def __repr__(self):
        return self.__str__()

    def __iter__(self):
        return self.__ground_atoms.__iter__()

    def __hash__(self):
        return self.__weight.__hash__() + tuple(self.__ground_atoms).__hash__()

    def __eq__(self, other):
        return (self.__weight == other.__weight) + (self.__ground_atoms == other.__ground_atoms)

    def __ne__(self, other):
        return not self.__eq__(other)

    def add_ground_atom_to_conjunction(self, ground_atom):
        self.__ground_atoms.append(ground_atom)


class MLN:
    def __init__(self):
        self.__header = []
        self.__formulas = []

    def __str__(self):
        header = map(str, self.__header)
        header.sort()
        formulas = map(str, self.__formulas)
        formulas.sort()
        return reduce(lambda l1, l2: str(l1) + "\n" + str(l2), header + formulas)

    def __repr__(self):
        return self.__str__()

    def append_predicate(self, predicate):
        self.__header.append(predicate)

    def append_formula(self, formula):
        self.__formulas.append(formula)
