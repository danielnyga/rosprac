from mln_elements import *
from math import log


class WeightCalculator(object):

    @staticmethod
    def learn(mln, database_collection):
        absolute_frequencies_of_true_formulas = {f: 0 for f in mln.formulas}
        for db in database_collection:
            ground_formulas = WeightCalculator.__get_ground_formulas(db, mln)
            for ground_formula in ground_formulas:
                if ground_formula.satisfied(db):
                    absolute_frequencies_of_true_formulas[ground_formula.original_formula] += 1
        for formula, frequency in absolute_frequencies_of_true_formulas.items():
            formula.weight = 0 if frequency == 0 else WeightCalculator.__get_weight_for_frequency_one() + log(frequency)
        return mln

    @staticmethod
    def __get_weight_for_frequency_one():
        return 20

    @staticmethod
    def __get_ground_formulas(db, mln):
        object_type_to_objects = {}
        for gnd_atom in db:
            if hasattr(gnd_atom, "type_value_pairs"):
                for object_type, object_value in gnd_atom.type_value_pairs:
                    if object_type not in object_type_to_objects:
                        object_type_to_objects[object_type] = set()
                    object_type_to_objects[object_type].add(object_value)
        to_return = []
        formulas_to_be_grounded = [WeightCalculator.GroundFormula(formula) for formula in mln.formulas]
        while formulas_to_be_grounded:
            formula = formulas_to_be_grounded.pop()
            if not WeightCalculator.__replace_one_variable(formula, object_type_to_objects, formulas_to_be_grounded):
                to_return.append(formula)
        return to_return

    @staticmethod
    def __replace_one_variable(formula, object_type_to_objects, formulas_to_be_grounded):
        for formula_atom in formula.formula.logical_formula:
            type_value_pairs = []
            if hasattr(formula_atom, "type_value_pairs"):
                type_value_pairs = formula_atom.type_value_pairs
            elif hasattr(formula_atom, "fixed_type_variable_pairs"):
                type_value_pairs = formula_atom.fixed_type_variable_pairs
            for object_type, object_value in type_value_pairs:
                if str(object_value).startswith("?"):
                    if object_type not in object_type_to_objects:
                        return True
                    else:
                        for value in object_type_to_objects[object_type]:
                            new_formula = WeightCalculator.GroundFormula(formula, object_value, value)
                            formulas_to_be_grounded.append(new_formula)
                        return True
        return False

    class GroundFormula(object):
        def __init__(self, formula, variable_to_be_replaced=None, replacing_object=None):
            if hasattr(formula, "formula") and hasattr(formula, "original_formula"):
                self.__formula = deepcopy(formula.formula)
                self.__original_formula = formula.original_formula
            else:
                self.__formula = deepcopy(formula)
                self.__original_formula = formula
            if variable_to_be_replaced is not None and replacing_object is not None:
                for formula_atom in self.__formula.logical_formula:
                    type_value_pairs = []
                    replace_value = lambda old_type, old_value, new_value: None
                    if hasattr(formula_atom, "type_value_pairs"):
                        type_value_pairs = formula_atom.type_value_pairs
                        replace_value = lambda obj_type, old, new: formula_atom.set_argument_value(object_type, new)
                    elif hasattr(formula_atom, "fixed_type_variable_pairs"):
                        type_value_pairs = formula_atom.fixed_type_variable_pairs

                        def replace_fixed_value(obj_type, old, new):
                            formula_atom.fixed_type_variable_pairs = \
                                [(obj_type, new) if t == obj_type and v == old else (t, v)
                                 for t, v in formula_atom.fixed_type_variable_pairs]
                        replace_value = replace_fixed_value
                    for object_type, object_value in type_value_pairs:
                        if object_value == variable_to_be_replaced:
                            replace_value(object_type, object_value, replacing_object)

        def satisfied(self, database):
            for gnd_atom in self.__formula.logical_formula:
                if isinstance(gnd_atom, ClosedWorldGroundAtoms):
                    affected_atoms = filter(lambda atom: hasattr(atom, "predicate"), database)
                    affected_atoms = filter(lambda atom: atom.predicate == gnd_atom.predicate, affected_atoms)
                    affected_atoms = filter(lambda atom: hasattr(atom, "get_argument_value"), affected_atoms)
                    for fixed_type, fixed_variable in gnd_atom.fixed_type_variable_pairs:
                        fixed_parameter_correct = lambda atom: atom.get_argument_value(fixed_type) == fixed_variable
                        affected_atoms = filter(fixed_parameter_correct, affected_atoms)
                    for affected_atom in affected_atoms:
                        if affected_atom not in self.__formula.logical_formula:
                            return False
                elif gnd_atom not in database and not gnd_atom.negated:
                    return False
            return True

        @property
        def formula(self):
            return self.__formula

        @property
        def original_formula(self):
            return self.__original_formula
