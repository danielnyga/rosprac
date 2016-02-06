class NegatableElement(object):
    def __init__(self, negated):
        self.__negated = negated

    @property
    def negated(self):
        return self.__negated

    @negated.setter
    def negated(self, value):
        self.negated = value


class GroundAtom(NegatableElement):
    def __init__(self, predicate, constants, negated=False):
        NegatableElement.__init__(self, negated)
        self.__predicate = predicate
        self._constants = [list(typeValuePair) for typeValuePair in constants]

    def __invert__(self):
        return GroundAtom(self.__predicate, self._constants, not self.negated)

    def __str__(self):
        args = str(reduce(lambda s1, s2: str(s1) + "," + str(s2), [c[1] for c in self._constants]))
        return ("!" if self.negated else "") + self.__predicate.name + "(" + args + ")"

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return self.__predicate.__hash__() + self.negated.__hash__() + \
            tuple([tuple(pair) for pair in self._constants]).__hash__()

    def __eq__(self, other):
        return hasattr(other, "predicate") and hasattr(other, "negated") and hasattr(other, "type_value_pairs") and \
               (self.predicate == other.predicate) and \
               (self.negated == other.negated) and \
               (self.type_value_pairs == other.type_value_pairs)

    def __ne__(self, other):
        return not self.__eq__(other)

    @property
    def predicate(self):
        return self.__predicate

    @property
    def type_value_pairs(self):
        return self._constants

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
        for arg in args:
            if str(arg) == "":
                raise Exception(
                    "Empty strings are not supported as predicate arguments: "
                    + self.__predicate_name
                    + str(args)
                )
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
        return str(reduce(lambda db1, db2: str(db1) + "\n---\n" + str(db2), self.__databases))

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
        other_expand_operator = hasattr(other, "__apply_expand_operator") and other.__apply_expand_operator
        return GroundAtom.__eq__(self, other) and self.__apply_expand_operator == other_expand_operator

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
    def __init__(self, weight, logical_formula):
        self.__weight = weight
        self.__logical_formula = logical_formula

    def __str__(self):
        return str(self.__weight) + " " + str(self.__logical_formula)

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return self.__weight.__hash__() + self.__logical_formula.__hash__()

    def __eq__(self, other):
        return (self.__weight == other.__weight) + (self.__logical_formula == other.__logical_formula)

    def __ne__(self, other):
        return not self.__eq__(other)

    @property
    def weight(self):
        return self.__weight

    @weight.setter
    def weight(self, value):
        self.__weight = value

    @property
    def logical_formula(self):
        return self.__logical_formula


class LogicalConnective(NegatableElement):
    def __init__(self, elements, negated=False):
        NegatableElement.__init__(self, negated)
        self._elements = list(elements)
        self._elements.sort(key=lambda a: str(a))

    def __repr__(self):
        return self.__str__()

    def __iter__(self):
        return self._elements.__iter__()

    def __hash__(self):
        return tuple(self._elements).__hash__()

    def __eq__(self, other):
        return (self.__class__.__name__ == other.__class__.__name__) + (self._elements == other._elements)

    def __ne__(self, other):
        return not self.__eq__(other)

    def add_element(self, element):
        self._elements.append(element)
        self._elements.sort(key=lambda a: str(a))

    def remove_element(self, element):
        self._elements.remove(element)


class Conjunction(LogicalConnective):
    def __init__(self, elements, negated=False):
        LogicalConnective.__init__(self, elements, negated)

    def __str__(self):
        elements_in_conjunction = str(reduce(lambda g1, g2: str(g1) + " ^ " + str(g2), self._elements))
        negation_start = "!(" if self.negated else ""
        negation_end = ")" if self.negated else ""
        return negation_start + elements_in_conjunction + negation_end


class ExistentialQuantifier(NegatableElement):
    def __init__(self, qualified_variables, logical_formula, negated=False):
        NegatableElement.__init__(self, negated)
        self.__qualified_variables = qualified_variables
        self.__logical_formula = logical_formula

    def __str__(self):
        qualified_variables = str(reduce(lambda x, y: str(x) + ", " + str(y), self.__qualified_variables))
        logical_formulas = " (" + str(self.__logical_formula) + ")"
        negation_start = "!(" if self.negated else ""
        negation_end = ")" if self.negated else ""
        return negation_start + "EXIST " + qualified_variables + logical_formulas + negation_end

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return tuple(self.__qualified_variables).__hash__() + \
               self.__logical_formula.__hash__() + self.negated.__hash__()

    def __eq__(self, other):
        return self.__qualified_variables == other.__qualified_variables and \
               self.negated == other.negated and \
               self.__logical_formula == other.__logical_formula

    def __ne__(self, other):
        return not self.__eq__(other)


class EqualityComparison(NegatableElement):
    def __init__(self, element1, element2, negated=False):
        NegatableElement.__init__(self, negated)
        self.__element1 = element1
        self.__element2 = element2

    def __str__(self):
        if self.negated:
            return str(self.__element1) + "=/=" + str(self.__element2)
        else:
            return str(self.__element1) + "=" + str(self.__element2)

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return self.__element1.__hash__() + self.__element2.__hash__()

    def __eq__(self, other):
        return self.__element1 == other.__element1 and self.__element2 == other.__element2

    def __ne__(self, other):
        return not self.__eq__(other)


class ClosedWorldGroundAtoms(object):
    def __init__(self, predicate, fixed_variables, exceptions, variable_prefix):
        self.__predicate = predicate
        self.__fixed_type_variable_pairs = fixed_variables
        self.__exceptions = exceptions
        self.__variable_prefix = variable_prefix
        self.__realization = None
        self.calculate_realization()

    def calculate_realization(self):
        variable_suffix = 0
        quantified_variables = []
        variables = []
        type_to_quantified_variables = {}
        for variable_type in self.__predicate.types:
            variable_is_fixed = reduce(lambda rest, f: rest or variable_type == f[0],
                                       self.fixed_type_variable_pairs, False)
            if variable_is_fixed:
                variables.append(reduce(lambda rest, f: f[1] if variable_type == f[0] else rest,
                                        self.fixed_type_variable_pairs, None))
            else:
                variable = str(self.__variable_prefix) + str(variable_suffix)
                variable_suffix += 1
                type_to_quantified_variables[variable_type] = variable
                variables.append(variable)
                quantified_variables.append(variable)
        negated_predicate = self.__predicate(*variables)
        conjunction_elements = [negated_predicate]
        for exceptions_for_one_atom in self.__exceptions:
            sub_conjunction_elements = []
            for exception in exceptions_for_one_atom:
                variable_type = exception[0]
                allowed_value = exception[1]
                variable = type_to_quantified_variables[variable_type]
                sub_conjunction_elements.append(EqualityComparison(variable, allowed_value))
            conjunction_elements.append(Conjunction(sub_conjunction_elements, True))
        new_logic_formula = Conjunction(conjunction_elements)
        self.__realization = ExistentialQuantifier(quantified_variables, new_logic_formula, True)

    def __str__(self):
        return str(self.__realization)

    def __repr__(self):
        return self.__str__()

    def __hash__(self):
        return hash(self.__realization)

    def __eq__(self, other):
        return self.__realization == other.__realization

    def __ne__(self, other):
        return not self.__eq__(other)

    @property
    def predicate(self):
        return self.__predicate

    @property
    def fixed_type_variable_pairs(self):
        return self.__fixed_type_variable_pairs

    @fixed_type_variable_pairs.setter
    def fixed_type_variable_pairs(self, value):
        self.__fixed_type_variable_pairs = value
        self.calculate_realization()


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
        return str(reduce(lambda l1, l2: str(l1) + "\n" + str(l2), header + formulas))

    def __repr__(self):
        return self.__str__()

    def append_predicate(self, predicate):
        self.__header.append(predicate)

    def append_formulas(self, formulas):
        self.__formulas+=formulas

    @property
    def name(self):
        return self.__name

    @property
    def formulas(self):
        return self.__formulas

    @formulas.setter
    def formulas(self, value):
        self.__formulas = value

    def save(self, file_name):
        if not file_name:
            file_name = str(self.__name)+".mln"
        f = open(file_name + ".mln", 'w')
        f.write(self.__str__())
        f.close()