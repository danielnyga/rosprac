from itertools import groupby

from robot_memory import database_creator
from robot_memory import task_tree_preprocessor
from robot_memory.robot_memory_constants import Types, Predicates
from robot_memory.mln_elements import *
import math
import os


class MlnType(object):
    DESIGNATOR = "Designator"


def create_and_save_mlns(root_nodes, learner=None, debug=False, logger=None):
    root_nodes = task_tree_preprocessor.preprocess_task_tree(root_nodes)
    dbs = database_creator.create_database_collection(root_nodes)
    mlns = [
        _create_designator_mln(dbs)
    ]
    FILENAME_PREFIX = "learnt_mlns/"
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
    necessary_designator_predicates = [Predicates.TASK_NAME]
    top_level_designator_elements = necessary_designator_predicates + [Predicates.TASK_SUCCESS, Predicates.GOAL_PATTERN,
                                                                       Predicates.GOAL_PROPERTY,
                                                                       Predicates.GOAL_DESIGNATOR]
    optional_designator_predicates = top_level_designator_elements + [Predicates.DESIGNATOR_HASH,
                                                                      Predicates.DESIGNATOR_TYPE]
    formulas = _get_formula_templates_from_databases(databases, optional_designator_predicates,
                                                                necessary_designator_predicates)
    formulas = _split_objects_of_different_type(formulas, Types.DESIGNATOR)
    formulas = _apply_replacements(formulas, [(Types.DESIGNATOR, "?d"), (Types.TASK, "?t"),
                                              (Types.DESIGNATOR_PROPERTY, "?dp")])
    top_level_designator_formulas, other_designator_formulas = _split_formulas_in_true_false_lists(formulas,
                                                lambda f: _contains_ground_atom_with_predicate(f, Predicates.TASK_NAME))
    other_designator_formulas = _remove_conjunction_elements(other_designator_formulas, top_level_designator_elements)
    for formula in top_level_designator_formulas:
        for conjunction in formula.logical_formula:
            if not [gnd_atom for gnd_atom in conjunction if hasattr(gnd_atom, "predicate") and
                            gnd_atom.predicate == Predicates.TASK_SUCCESS]:
                conjunction.add_element(~Predicates.TASK_SUCCESS("?t0"))
    top_level_designator_formulas = _add_negation_for_all_but_existing(top_level_designator_formulas,
                                                                       Predicates.DESIGNATOR_HASH, 0, "?p",
                                                                       lambda f: not _domain_declaration_in_formula(f))
    top_level_designator_formulas = _add_negation_for_all_but_existing(top_level_designator_formulas,
                                                                       Predicates.DESIGNATOR_TYPE, 0, "?p",
                                                                       lambda f: not _domain_declaration_in_formula(f))
    top_level_designator_formulas = _add_negation_for_all_but_existing(top_level_designator_formulas,
                                                                       Predicates.GOAL_PATTERN, 0, "?p")
    top_level_designator_formulas = _add_negation_for_all_but_existing(top_level_designator_formulas,
                                                                       Predicates.GOAL_PROPERTY, 0, "?p")
    top_level_designator_formulas = _add_negation_for_all_but_existing(top_level_designator_formulas,
                                                                       Predicates.GOAL_DESIGNATOR, 0, "?p")
    top_level_designator_formulas = _remove_domain_declarations(top_level_designator_formulas)
    necessary_property_predicates = [Predicates.DESIGNATOR_PROPERTY, Predicates.DESIGNATOR_HASH,Predicates.PROPERTY_KEY]
    optional_property_predicates = necessary_property_predicates + [Predicates.PROPERTY_STRING_VALUE,
                                                                    Predicates.PROPERTY_DESIGNATOR_VALUE]
    property_formulas = _get_formula_templates_from_databases(databases, optional_property_predicates,
                                                              necessary_property_predicates)
    property_formulas = _split_objects_of_different_type(property_formulas, Types.DESIGNATOR)
    property_formulas = _split_objects_of_different_type(property_formulas, Types.DESIGNATOR_PROPERTY)
    property_formulas = _apply_replacements(property_formulas,[(Types.DESIGNATOR, "?d"),
                                                               (Types.DESIGNATOR_PROPERTY, "?dp")])
    property_formulas = _add_negation_for_all_but_existing(property_formulas, Predicates.PROPERTY_STRING_VALUE, 0,
                                                           "?dvs", lambda f: _contains_ground_atom_with_predicate(
                                                                             f, Predicates.PROPERTY_STRING_VALUE))
    property_formulas = _add_negation_for_all_but_existing(property_formulas, Predicates.PROPERTY_DESIGNATOR_VALUE, 0,
                                                           "?dvd", lambda f: _contains_ground_atom_with_predicate(
                                                                             f, Predicates.PROPERTY_DESIGNATOR_VALUE))
    formulas = other_designator_formulas + top_level_designator_formulas + property_formulas
    mln.append_formulas(formulas)
    return mln


def _learn_weights_and_save_mln(mln, databases, learner, filename_prefix):
    filename = filename_prefix + mln.name + ".mln"
    old_mln = ""
    if os.path.isfile(filename):
        with open(filename, "r") as f:
            old_mln = f.read()
    if len(mln.formulas) == 0:
        resulting_mln = old_mln if old_mln is not None else mln
    elif learner is None:
        _assign_weights(mln)
        resulting_mln = _combine_mlns(old_mln, mln)
    else:
        mln.formulas = list(set(mln.formulas))
        resulting_mln = learner.learn(old_mln, mln, databases)
    with open(filename, "w+") as f:
        f.write(str(resulting_mln))


def _get_weight_offset():
    return 650


def _assign_weights(mln):
    new_formulas = []
    formula_frequency = {}
    for formula in mln.formulas:
        formula_frequency[formula] = 1 if formula not in formula_frequency else formula_frequency[formula]+1
    for formula, frequency in formula_frequency.items():
        if _contains_ground_atom_with_predicate(formula, Predicates.TASK_NAME):
            formula.weight = _get_weight_offset() + math.log(frequency)
        else:
            formula.weight = _get_weight_offset()
        new_formulas.append(formula)
    mln.formulas = new_formulas


def _combine_mlns(old_mln, new_mln):
    #TODO: Do not increment weight of non top level formulas...
    old_mln = str(old_mln)
    new_mln = str(new_mln)
    formulas = {}
    other_lines = set()
    for line in (old_mln+"\n"+new_mln).split("\n"):
        stripped_line = line.strip()
        if stripped_line == "":
            continue
        parts = stripped_line.split()
        if not parts:
            continue
        potential_weight = parts[0]
        try:
            rest_of_the_line = line[len(potential_weight):].strip()
            weight = float(potential_weight)
            if rest_of_the_line not in formulas:
                formulas[rest_of_the_line] = []
            formulas[rest_of_the_line].append(weight)
        except:
            other_lines.add(stripped_line)
    to_return = list(other_lines)
    for formula, weights in formulas.items():
        frequencies = sum([math.exp(weight - _get_weight_offset()) for weight in weights])
        to_return.append(str(_get_weight_offset() + math.log(frequencies)) + " " + formula)
    return reduce(lambda l1, l2: l1+"\n"+l2, to_return)


def _create_mln_skeleton(mln_name):
    mln = MLN(mln_name)
    for predicate in Predicates.get_all_predicates():
        mln.append_predicate(predicate)
    return mln


def _get_formula_templates_from_databases(databases, possible_preds, necessary_preds):
    predicate_contained_in_formula = lambda gnd_atom: not hasattr(gnd_atom, "predicate") or \
                                                      gnd_atom.predicate in possible_preds
    formula_atoms = [filter(predicate_contained_in_formula,
                            [FormulaGroundAtom(ga) if isinstance(ga, FormulaGroundAtom) else ga for ga in db])
                     for db in databases]
    predicate_is_in_list = lambda gnd_atoms, pred: pred in [g.predicate for g in gnd_atoms if hasattr(g, "predicate")]
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
                    if element.get_argument_values(constant_type):
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
            atoms_with_type = filter(lambda atom: atom.get_argument_values(type_to_split), conjunction)
            atoms_without_type = filter(lambda atom: not atom.get_argument_values(type_to_split), conjunction)
            if not atoms_with_type:
                to_return.append(formula)
                continue
            atoms_with_type.sort(key=lambda atom: atom.get_argument_values(type_to_split)[0])
            for _, group in groupby(atoms_with_type, key=lambda atom: atom.get_argument_values(type_to_split)[0]):
                to_return.append(Formula(formula.weight, Conjunction([Conjunction(
                    [FormulaGroundAtom(gnd_atom) if isinstance(gnd_atom, GroundAtom) else gnd_atom
                     for gnd_atom in atoms_without_type + list(group)], False, True)])))
    return to_return


def _add_negation_for_all_but_existing(formulas, predicate, fixed_index, prefix, skip=lambda formula: False):
    for formula in formulas: #TODO: Make it work on nested elements...
        if skip(formula):
            continue
        for conjunction in list(formula.logical_formula):
            if not isinstance(conjunction, Conjunction):
                continue
            fixed_type = predicate.types[fixed_index]
            fixed_variables_to_list_of_exceptions = {}
            for ground_atom in conjunction:
                if hasattr(ground_atom, "predicate") and ground_atom.predicate == predicate:
                    fixed_variables = tuple([(fixed_index, ground_atom.get_argument_value(fixed_index))])
                    if not fixed_variables in fixed_variables_to_list_of_exceptions:
                        fixed_variables_to_list_of_exceptions[fixed_variables]=set()
                    exceptions = [(index, ground_atom.get_argument_value(index)) for index
                                  in range(0, len(ground_atom.predicate.types)) if index != fixed_index]
                    fixed_variables_to_list_of_exceptions[fixed_variables].add(tuple(exceptions))
                elif hasattr(ground_atom, "get_argument_values"):
                    for argument_value in ground_atom.get_argument_values(fixed_type):
                        fixed_variables = tuple([(fixed_index, argument_value)])
                        if fixed_variables not in fixed_variables_to_list_of_exceptions:
                            fixed_variables_to_list_of_exceptions[fixed_variables] = set()
            for fixed_variables, exceptions in fixed_variables_to_list_of_exceptions.items():
                formula.logical_formula.add_element(
                    ClosedWorldGroundAtoms(predicate, fixed_variables, exceptions, prefix))
    return formulas


def _remove_domain_declarations(formulas):
    def remove_domain_declarations_recursively(sub_formula):
        if not hasattr(sub_formula, "__iter__"):
            return
        for child in sub_formula:
            if isinstance(child, DomainDeclaration):
                sub_formula.remove_element(child)
            else:
                remove_domain_declarations_recursively(child)

    for formula in formulas:
        for sub_formula in formula.logical_formula:
            remove_domain_declarations_recursively(sub_formula)

    return formulas


def _split_formulas_in_true_false_lists(formulas, condition):
    true_list = []
    false_list = []
    for formula in formulas:
        if condition(formula):
            true_list.append(formula)
        else:
            false_list.append(formula)
    return true_list, false_list


def _remove_conjunction_elements(formulas, predicates_to_remove):
    to_return = []
    for formula in formulas:
        for atom in list(formulas.logical_formula):
            if atom.predicate in predicates_to_remove:
                formula.logical_formula.remove_element(atom)
        if len(formula.logical_formula) > 0:
            to_return.append(formula)
    return to_return


def _contains_ground_atom_with_predicate(formula, predicate):
    if hasattr(formula, "logical_formula"):
        return _contains_ground_atom_with_predicate(formula.logical_formula, predicate)
    if hasattr(formula, "predicate"):
        return formula.predicate == predicate
    if not hasattr(formula, "__iter__"):
        return False
    return any([_contains_ground_atom_with_predicate(child, predicate) for child in formula])


def _domain_declaration_in_formula(formula):
    if hasattr(formula, "logical_formula"):
        formula = formula.logical_formula
    if not hasattr(formula, "__iter__"):
        return False
    for child in formula:
        if isinstance(child, DomainDeclaration):
            return True
        elif _domain_declaration_in_formula(child):
            return True
    return False

#TODO: Move all trhee *recursively* functions into one...
