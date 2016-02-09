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
                           Predicates.DESIGNATOR, Predicates.GOAL_PROPERTY_VALUE, Predicates.DESIGNATOR_PROPERTY,
                           Predicates.SUB_DESIGNATOR, Predicates.TASK_NAME]
    formulas = _get_formula_templates_from_databases(databases, optional_predicates, necessary_predicates)
    formulas = _apply_replacements(formulas, [(Types.DESIGNATOR_OR_VALUE, "?d"), (Types.TASK, "?t")])
    for formula in formulas:
        if not [gnd_atom for gnd_atom in formula.logical_formula if gnd_atom.predicate == Predicates.TASK_SUCCESS]:
            formula.logical_formula.add_element(~Predicates.TASK_SUCCESS("?t0"))
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.GOAL_PATTERN, [(Types.TASK, "?t0")], "?p")
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.DESIGNATOR_OR_VALUE,[(Types.TASK, "?t0")], "?p")
    formulas = _add_negation_for_all_but_existing(
        formulas, Predicates.DESIGNATOR,[(Types.DESIGNATOR_OR_VALUE, "?d0")], "?p")
    formulas = _add_negation_for_all_but_existing(
        formulas, Predicates.GOAL_PROPERTY_VALUE,[(Types.DESIGNATOR_OR_VALUE, "?d0")], "?p")
    formulas = _add_negation_for_all_but_existing(
        formulas, Predicates.DESIGNATOR_PROPERTY,[(Types.DESIGNATOR_OR_VALUE, "?d0")], "?p")
    formulas = _add_negation_for_all_but_existing(
        formulas, Predicates.SUB_DESIGNATOR,[(Types.DESIGNATOR_OR_VALUE, "?d0")], "?p")
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
    formulas = [Formula(0.0, Conjunction(ground_atom_list)) for ground_atom_list in formula_atoms]
    return formulas


def _apply_replacements(formulas, formula_replacements):
    for formula in formulas:
        replacements = {}
        suffix = 0
        for ground_atom in formula.logical_formula:
            for replacement in formula_replacements:
                constant_type = replacement[0]
                if constant_type in ground_atom.predicate.types:
                    current_value = ground_atom.get_argument_value(constant_type)
                    if current_value in replacements:
                        ground_atom.set_argument_value(constant_type, replacements[current_value])
                    else:
                        new_value = new_value = replacement[1] + str(suffix)
                        replacements[current_value] = new_value
                        suffix += 1
                        ground_atom.set_argument_value(constant_type, new_value)
    return formulas


def _add_negation_for_all_but_existing(formulas, predicate, fixed_types_with_value, prefix):
    for formula in formulas: #TODO: Make it work on nested elements...
        has_desired_predicate = lambda gnd_atom: hasattr(gnd_atom, "predicate") and gnd_atom.predicate == predicate
        existing_atoms = filter(has_desired_predicate, formula.logical_formula)
        types_to_be_fixed = filter(lambda t: t not in [f[0] for f in fixed_types_with_value], predicate.types)
        exceptions = []
        for existing_atom in existing_atoms:
            gnd_atom_exception = []
            for fixed_type in types_to_be_fixed:
                value = existing_atom.get_argument_value(fixed_type)
                gnd_atom_exception.append((fixed_type, value))
            exceptions.append(gnd_atom_exception)
        formula.logical_formula.add_element(
            ClosedWorldGroundAtoms(predicate, fixed_types_with_value, exceptions, prefix))
    return formulas