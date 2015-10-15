from copy import deepcopy


class GroundAtom(object):
    def __init__(self, predicate, constants, negated=False):
        self.__predicate = predicate
        self._constants = [list(typeValuePair) for typeValuePair in constants]
        self.__negated = negated

    def __invert__(self):
        return GroundAtom(self.__predicate, self._constants, not self.__negated)

    def __str__(self):
        args = str(reduce(lambda s1, s2: str(s1) + "," + str(s2), [c[1] for c in self._constants]))
        return ("!" if self.__negated else "") + self.__predicate.name + "(" + args + ")"

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return self.__predicate.__hash__() + self.__negated.__hash__() + \
            tuple([tuple(pair) for pair in self._constants]).__hash__()

    def __eq__(self, other):
        return (self.__predicate == other.__predicate) and (self.__negated == other.__negated) and \
               (self._constants == other._constants)

    def __ne__(self, other):
        return not self.__eq__(other)

    @property
    def negated(self):
        return self.__negated

    @property
    def predicate(self):
        return self.__predicate

    def set_argument_value(self, argument_type, value):
        for pair in self._constants:
            if pair[0] == argument_type:
                pair[1] = value

    def get_argument_value(self, argument_type):
        for pair in self._constants:
            if pair[0] == argument_type:
                return pair[1]


class Predicate(object):
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

    @property
    def name(self):
        return self.__predicate_name

    @property
    def types(self):
        return self.__types


class Type(object):
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

    @property
    def name(self):
        return self.__type_name


class DatabaseCollection(object):
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

    def save(self, name):
        f = open(name + ".db", 'w')
        f.write(self.__str__())
        f.close()


class Database(object):
    def __init__(self):
        self.__ground_atoms = set()

    def __iter__(self):
        return iter(self.__ground_atoms)

    def __str__(self):
        sorted_atoms = list(self.__ground_atoms)
        sorted_atoms.sort(key=lambda a: a.predicate.name)
        return str(reduce(lambda g1, g2: str(g1) + "\n" + str(g2), sorted_atoms))

    def __repr__(self):
        return self.__str__()

    def append(self, ground_atom):
        self.__ground_atoms.add(ground_atom)


class FormulaGroundAtom(GroundAtom):
    def __init__(self, gnd_atom):
        GroundAtom.__init__(self, gnd_atom.predicate, [const for const in gnd_atom._constants], gnd_atom.negated)
        self.__apply_expand_operator = False

    def __eq__(self, other):
        return GroundAtom.__eq__(self, other) and self.__apply_expand_operator == other.__apply_expand_operator

    def __ne__(self, other):
        return not self.__eq__()

    def __hash__(self):
        return GroundAtom.__hash__(self) + self.__apply_expand_operator.__hash__()

    def __str__(self):
        return ("*" if self.__apply_expand_operator else "") + GroundAtom.__str__(self)

    def __repr__(self):
        return str(self)

    @property
    def apply_expand_operator(self):
        return self.__apply_expand_operator

    @apply_expand_operator.setter
    def apply_expand_operator(self, value):
        self.__apply_expand_operator = value


class Formula(object):
    def __init__(self, weight, *ground_atoms):
        self.__weight = weight
        self.__ground_atoms = [FormulaGroundAtom(g) for g in ground_atoms]
        self.__ground_atoms.sort(key=lambda a: a.predicate.name)

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

    @property
    def weight(self):
        return self.__weight

    def add_ground_atom_to_conjunction(self, ground_atom):
        self.__ground_atoms.append(ground_atom)
        self.__ground_atoms.sort(key=lambda a: a.predicate.name)


class MLN(object):
    def __init__(self, name):
        self.__header = []
        self.__formulas = []
        self.__name = name

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

    def append_formulas(self, formulas):
        self.__formulas+=formulas

    @property
    def name(self):
        return self.__name

    def save(self, file_name):
        if not file_name:
            file_name = str(self.__name)+".mln"
        f = open(file_name + ".mln", 'w')
        f.write(self.__str__())
        f.close()