class GroundAtom:
    def __init__(self, predicate_name, constants, negated=False):
        self.__predicate_name = predicate_name
        self.__constants = constants
        self.__negated = negated

    def __invert__(self):
        return GroundAtom(self.__predicate_name, self.__constants, not self.__negated)

    def __str__(self):
        args = str(reduce(lambda s1, s2: str(s1) + "," + str(s2), self.__constants))
        return ("!" if self.__negated else "") + self.__predicate_name + "(" + args + ")"

    def __repr__(self):
        return self.__str__()


class Predicate:
    def __init__(self, predicate_name, number_of_arguments):
        self.__predicate_name = predicate_name
        self.__number_of_arguments = number_of_arguments

    def __call__(self, *args, **kwargs):
        if kwargs:
            raise Exception("keyword arguments are not supported!")
        if len(args) != self.__number_of_arguments:
            raise Exception("Invalid number of arguments: " + str(self.__number_of_arguments) + "required!")
        return GroundAtom(self.__predicate_name, args)

    def name(self):
        return self.__predicate_name


class Type:
    def __init__(self, type_name, is_soft_functional=False):
        self.__type_name = type_name
        self.__is_soft_functional = is_soft_functional
        if not is_soft_functional:
            self.__soft_functional_type = Type(type_name, True)

    def __neg__(self):
        return  self.__soft_functional_type

    def __str__(self):
        return self.__type_name + ("?" if self.__is_soft_functional else "")

    def __repr__(self):
        return self.__str__()


class DatabaseCollection:
    def __init__(self):
        self.__databases = []

    def append(self, database):
        self.__databases.append(database)

    def __iter__(self):
        return iter(self.__databases)

    def __str__(self):
        return reduce(lambda db1, db2: str(db1) + "\n---\n" + str(db2), self.__databases)

    def __repr__(self):
        return self.__str__()


class Database:
    def __init__(self):
        self.__ground_atoms = []

    def append(self, ground_atom):
        self.__ground_atoms.append(ground_atom)

    def __iter__(self):
        return iter(self.__ground_atoms)

    def __str__(self):
        return reduce(lambda g1, g2: str(g1) + "\n" + str(g2), self.__ground_atoms)

    def __repr__(self):
        return self.__str__()


class MLN:
    def __init__(self):
        self.__header = []
        self.__formulas = []

    def append_predicate(self, predicate):
        self.__header.append(predicate)

    def append_formula(self, formula):
        self.__formulas.append(formula)

    def __str__(self):
        return reduce(lambda l1, l2: str(l1) + "\n" + str(l2), self.__header + self.__formulas)

    def __repr__(self):
        return self.__str__()
