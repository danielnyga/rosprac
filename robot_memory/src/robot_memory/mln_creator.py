from itertools import groupby

from robot_memory import database_creator
from robot_memory import task_tree_preprocessor
from robot_memory.robot_memory_constants import Types, Predicates
from robot_memory.mln_elements import *
import math
import time
import os


class MlnType(object):
    DESIGNATOR = "Designator"


def create_and_save_mlns(root_nodes, learner=None, debug=False, logger=None):
    root_nodes = task_tree_preprocessor.preprocess_task_tree(root_nodes)
    dbs = database_creator.create_database_collection(root_nodes)
    mlns = [
        _create_designator_mln(dbs)
    ]
    FILENAME_PREFIX = "learnt_mlns_" + time.strftime("%Y-%m-%d_%H-%M-%S") + "/"
    if not os.path.isdir(FILENAME_PREFIX):
        os.mkdir(FILENAME_PREFIX)
    if debug:
        if logger is not None:
            for node in root_nodes:
                logger.loginfo(node.tree_as_string())
        for mln in mlns:
            mln.save(FILENAME_PREFIX + mln.name + "_before_learning")
        dbs.save(FILENAME_PREFIX + "train")
    for mln in mlns:
        _learn_weights_and_save_mln(mln, dbs, learner, FILENAME_PREFIX)


def _create_designator_mln(databases):
    mln = _create_mln_skeleton(MlnType.DESIGNATOR)
    necessary_predicates = [Predicates.TASK_NAME]
    optional_predicates = [Predicates.TASK_SUCCESS, Predicates.GOAL_PATTERN, Predicates.DESIGNATOR_OR_VALUE,
                           Predicates.GOAL_PROPERTY_VALUE, Predicates.GOAL_PROPERTY_DESIGNATOR,
                           Predicates.DESIGNATOR_PROPERTY, Predicates.DESIGNATOR_TYPE, Predicates.DESIGNATOR_HASH,
                           Predicates.SUB_DESIGNATOR, Predicates.TASK_NAME]
    formulas = _get_formula_templates_from_databases(databases, optional_predicates, necessary_predicates)
    formulas = _split_objects_of_different_type(formulas, Types.DESIGNATOR)
    formulas = _apply_replacements(formulas,
                                   [(Types.DESIGNATOR, "?d"), (Types.DESIGNATOR_OR_VALUE, "?dv"), (Types.TASK, "?t")])
    for formula in formulas:
        for conjunction in formula.logical_formula:
            if not [gnd_atom for gnd_atom in conjunction if gnd_atom.predicate == Predicates.TASK_SUCCESS]:
                conjunction.add_element(~Predicates.TASK_SUCCESS("?t0"))
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.GOAL_PATTERN, [0], "?p")
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.DESIGNATOR_OR_VALUE, [0], "?p")
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.GOAL_PROPERTY_VALUE, [0], "?p")
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.GOAL_PROPERTY_DESIGNATOR, [0], "?p")
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.DESIGNATOR_PROPERTY, [0], "?p")
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.SUB_DESIGNATOR, [0], "?p")
    mln.append_formulas(formulas)
    return mln


def _learn_weights_and_save_mln(mln, databases, learner, filename_prefix):
    if len(mln.formulas) == 0:
        resulting_mln = mln
    elif learner is None:
        resulting_mln = _assign_weights(mln)
    else:
        mln.formulas = list(set(mln.formulas))
        resulting_mln = learner.learn(mln, databases)
    f = open(filename_prefix + mln.name + ".mln", 'w')
    f.write(str(resulting_mln))
    f.close()


def _assign_weights(mln):
    new_formulas = []
    formula_frequency = {}
    for formula in mln.formulas:
        formula_frequency[formula] = 1 if formula not in formula_frequency else formula_frequency[formula]+1
    for formula, frequency in formula_frequency.items():
        formula.weight = 400 + math.log(frequency)
        new_formulas.append(formula)
    mln.formulas = new_formulas
    return mln


def _create_mln_skeleton(mln_name):
    mln = MLN(mln_name)
    for predicate in Predicates.get_all_predicates():
        mln.append_predicate(predicate)
    return mln


def _get_formula_templates_from_databases(databases, possible_preds, necessary_preds):
    predicate_contained_in_formula = lambda gnd_atom: gnd_atom.predicate in possible_preds
    formula_atoms = [filter(predicate_contained_in_formula, [FormulaGroundAtom(ga) for ga in db]) for db in databases]
    predicate_is_in_list = lambda gnd_atoms, pred: pred in [g.predicate for g in gnd_atoms]
    atoms_contain_necessary_elements = lambda atoms: \
        reduce(lambda a, b: a and b, [predicate_is_in_list(atoms, pred) for pred in necessary_preds])
    formula_atoms = filter(atoms_contain_necessary_elements, formula_atoms)
    formulas = \
        [Formula(0.0, Conjunction([Conjunction(ground_atom_list, False, True)])) for ground_atom_list in formula_atoms]
    return formulas


def _apply_replacements(formulas, formula_replacements):
    for formula in formulas:
        replacements = {}

        def apply_replacements_recursively(element, suffix):
            if hasattr(element, "__iter__"):
                for child in element:
                    suffix = apply_replacements_recursively(child, suffix)
            else:
                for replacement in formula_replacements:
                    constant_type = replacement[0]
                    if constant_type in element.predicate.types:
                        for index, current_value in enumerate(element.get_argument_values(constant_type)):
                            if current_value in replacements:
                                element.set_argument_value(constant_type, index, replacements[current_value])
                            else:
                                new_value = replacement[1] + str(suffix)
                                replacements[current_value] = new_value
                                suffix += 1
                                element.set_argument_value(constant_type, index, new_value)
            return suffix

        apply_replacements_recursively(formula.logical_formula, 0)
    return formulas


def _split_objects_of_different_type(formulas, type_to_split):
    to_return = []
    for formula in formulas:
        for conjunction in formula.logical_formula:
            atoms_with_type = filter(lambda atom: type_to_split in atom.predicate.types, conjunction)
            atoms_without_type = filter(lambda atom: type_to_split not in atom.predicate.types, conjunction)
            if not atoms_with_type:
                to_return.append(formula)
                continue
            atoms_with_type.sort(key=lambda atom: atom.get_argument_values(type_to_split)[0])
            for _, group in groupby(atoms_with_type, key=lambda atom: atom.get_argument_values(type_to_split)[0]):
                to_return.append(Formula(formula.weight, Conjunction([Conjunction(
                    [FormulaGroundAtom(gnd_atom) for gnd_atom in atoms_without_type + list(group)], False, True)])))
    return to_return


def _add_negation_for_all_but_existing(formulas, predicate, fixed_indices, prefix):
    for formula in formulas: #TODO: Make it work on nested elements...
        for conjunction in list(formula.logical_formula):
            if not isinstance(conjunction, Conjunction):
                continue
            has_desired_predicate = lambda gnd_atom: hasattr(gnd_atom, "predicate") and gnd_atom.predicate == predicate
            existing_atoms = filter(has_desired_predicate, conjunction)
            fixed_variables_to_list_of_exceptions = {}
            for ground_atom in existing_atoms:
                fixed_variables = tuple([(index, ground_atom.get_argument_value(index)) for index in fixed_indices])
                if not fixed_variables in fixed_variables_to_list_of_exceptions:
                    fixed_variables_to_list_of_exceptions[fixed_variables]=set()
                exceptions = [(index, ground_atom.get_argument_value(index)) for index
                               in range(0, len(ground_atom.predicate.types)) if index not in fixed_indices]
                fixed_variables_to_list_of_exceptions[fixed_variables].add(tuple(exceptions))
            for fixed_variables, exceptions in fixed_variables_to_list_of_exceptions.items():
                formula.logical_formula.add_element(
                    ClosedWorldGroundAtoms(predicate, fixed_variables, exceptions, prefix))
    return formulas

