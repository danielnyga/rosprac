from itertools import groupby, combinations

from robot_memory import database_creator
from robot_memory import task_tree_preprocessor
from robot_memory.robot_memory_constants import Types, Predicates
from robot_memory.mln_elements import *
import math
import os
import shutil


class MlnType(object):
    TASK_AND_DESIGNATOR = "task_and_designator"
    DESIGNATOR = "designator"


class Configuration(object):
    def __init__(self, extend_old_mlns, include_perform=False, debug=False, logger=None, learner=None):
        self.extend_old_mlns = extend_old_mlns
        self.include_perform = include_perform
        self.debug = debug
        self.logger = logger
        self.learner = learner

def create_and_save_mlns(root_nodes, configuration):
    root_nodes = task_tree_preprocessor.preprocess_task_tree(root_nodes, configuration.include_perform)
    dbs = database_creator.create_database_collection(root_nodes)
    mlns = [
        _create_task_and_designator_mln(dbs),
        _create_designator_mln(dbs)
    ]
    FILENAME_PREFIX = "../learnt_mlns/"
    if not configuration.extend_old_mlns and os.path.isdir(FILENAME_PREFIX):
        shutil.rmtree(FILENAME_PREFIX)
    if not os.path.isdir(FILENAME_PREFIX):
        os.mkdir(FILENAME_PREFIX)
    if configuration.debug:
        if configuration.logger is not None:
            for node in root_nodes:
                configuration.logger.loginfo(node.tree_as_string())
        for mln in mlns:
            mln.save(FILENAME_PREFIX + mln.name + "_before_learning")
        dbs.save(FILENAME_PREFIX + "train")
    for mln in mlns:
        _learn_weights_and_save_mln(mln, dbs, configuration.learner, FILENAME_PREFIX)


def _create_task_and_designator_mln(databases):
    mln = _create_mln_skeleton(MlnType.TASK_AND_DESIGNATOR)
    necessary_predicates = [Predicates.TASK_NAME]
    optional_predicates = [Predicates.TASK_FAILURE, Predicates.DESIGNATOR_PROPERTY,
                           Predicates.DESIGNATOR_PROPERTY_KEY, Predicates.DESIGNATOR_PROPERTY_VALUE,
                           Predicates.GOAL_PATTERN, Predicates.TASK_NAME, Predicates.GOAL_PARAMETER,
                           Predicates.GOAL_PARAMETER_KEY]
    formulas = _get_formula_templates_from_databases(databases, optional_predicates, necessary_predicates)
    formulas = _split_different_designators(formulas)
    formulas = _apply_replacements(formulas, [(Types.DESIGNATOR_PROPERTY, "?dp"),  (Types.TASK, "?t"),
                                              (Types.DESIGNATOR, "?d")])
    mln.append_formulas(formulas)
    return mln


def _create_designator_mln(databases):
    mln = _create_mln_skeleton(MlnType.DESIGNATOR)
    necessary_predicates = [Predicates.DESIGNATOR_PROPERTY, Predicates.TASK_FAILURE]
    optional_predicates = [Predicates.DESIGNATOR_PROPERTY, Predicates.TASK_FAILURE,
                           Predicates.DESIGNATOR_PROPERTY_KEY, Predicates.DESIGNATOR_PROPERTY_VALUE]
    formulas = _get_formula_templates_from_databases(databases, optional_predicates, necessary_predicates)
    formulas = _split_different_designators(formulas)
    formulas = _apply_replacements(formulas, [(Types.DESIGNATOR_PROPERTY, "?dp"), (Types.DESIGNATOR, "?d")])
    for formula in formulas:
        for conjunction in formula.logical_formula:
            for atom in list(conjunction):
                if atom.predicate == Predicates.TASK_FAILURE:
                    conjunction.remove_element(atom)
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
    return 100


def _assign_weights(mln):
    new_formulas = []
    formula_frequency = {}
    for formula in mln.formulas:
        formula_frequency[formula] = 1 if formula not in formula_frequency else formula_frequency[formula]+1
    for formula, frequency in formula_frequency.items():
        formula.weight = _get_weight_offset() + math.log(frequency)
        new_formulas.append(formula)
    mln.formulas = new_formulas


def _combine_mlns(old_mln, new_mln):
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

        def order(conjunction):
            replacement_types = {replacement[0] for replacement in formula_replacements}
            if not all([hasattr(e, "predicate") and hasattr(e, "type_value_pairs") for e in conjunction]):
                return conjunction
            first = []
            last = []
            for _, group in groupby(sorted(conjunction, key=lambda e1: e1.predicate), lambda e2: e2.predicate):
                group = list(group)
                tvps = [(str([tvp for tvp in e.type_value_pairs if tvp[0] not in replacement_types]), e) for e in group]
                different_arguments = len(list(groupby(tvps, lambda tvp: tvp[0]))) > 1
                if different_arguments:
                    first += [e for _, e in sorted(tvps)]
                else:
                    last += list(group)
            return first + last

        def apply_replacements_recursively(element, suffix):
            if hasattr(element, "__iter__"):
                for child in order(element):
                    suffix = apply_replacements_recursively(child, suffix)
                if hasattr(element, "sort"):
                    element.sort()
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


def _split_different_designators(formulas):
    to_return = []
    for formula in formulas:
        for conjunction in formula.logical_formula:
            designator_property_list = [(atom.get_argument_values(Types.DESIGNATOR), atom,
                                         atom.get_argument_values(Types.DESIGNATOR_PROPERTY)) for atom in conjunction]
            property_designator_map = {p[0]: d[0] for d, _, p in designator_property_list if d and p}
            designator_atom_map = {d[0]: [] for d, _, _ in designator_property_list if d}
            other_atoms = []
            for designators, atom, properties in designator_property_list:
                if designators:
                    designator_atom_map[designators[0]].append(atom)
                elif properties:
                    designator_atom_map[property_designator_map[properties[0]]].append(atom)
                else:
                    other_atoms.append(atom)
            for _, atoms_for_one_designator in designator_atom_map.items():
                get_property_arguments = lambda atom: atom.get_argument_values(Types.DESIGNATOR_PROPERTY)
                designator_atoms = filter(lambda atom: not get_property_arguments(atom), atoms_for_one_designator)
                property_atoms = filter(get_property_arguments, atoms_for_one_designator)
                property_atoms.sort(key = lambda atom: get_property_arguments(atom)[0])
                property_groups = [list(grp) for _, grp in groupby(property_atoms, lambda a: get_property_arguments(a))]
                if len(property_groups) == 1:
                    conjunction_elements = [] #TODO: Support that somehow...i.e. use a negated existential quantifier
                else:
                    def escape(combination):
                        new_combination = [FormulaGroundAtom(a) for a in combination]
                        for a in new_combination:
                            a.set_argument_value(Types.DESIGNATOR_PROPERTY, 0, "Foo")
                        return new_combination
                    comb = filter(lambda c: escape(c[0]) != escape(c[1]), combinations(property_groups, 2))
                    conjunction_elements = [other_atoms + designator_atoms + list(c[0]) + list(c[1]) for c in comb]
                for atoms in conjunction_elements:
                    atoms = [FormulaGroundAtom(atom) for atom in atoms]
                    new_formula = Formula(formula.weight, Conjunction([Conjunction(atoms)]))
                    to_return.append(new_formula)
    return to_return