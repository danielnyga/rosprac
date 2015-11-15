import database_creator
import robot_state_preprocessor
from robot_memory_constants import Types, Predicates
from mln_elements import *
from functools import partial
from itertools import groupby
import time
import os


class MlnType(object):
    STATE_MACHINE = "StateMachine"
    TASK = "Task"
    OBJECT = "Object"


def create_and_save_mlns(robot_state_messages, learner, debug=False):
    preprocessed_messages = robot_state_preprocessor.extract_additional_relations(robot_state_messages)
    dbs = database_creator.create_database_collection(preprocessed_messages)
    mlns = [
        _create_state_machine_mln(dbs),
        _create_task_mln(dbs),
        _create_object_mln(dbs),
    ]
    FILENAME_PREFIX = "learnt_mlns_" + time.strftime("%Y-%m-%d_%H-%M-%S") + "/"
    if not os.path.isdir(FILENAME_PREFIX):
        os.mkdir(FILENAME_PREFIX)
    if debug:
        for mln in mlns:
            mln.save(FILENAME_PREFIX + mln.name + "_before_learning")
        dbs.save(FILENAME_PREFIX + "train")
    for mln in mlns:
        _learn_weights_and_save_mln(mln, dbs, learner, FILENAME_PREFIX)


def _create_state_machine_mln(databases):
    mln = _create_mln_skeleton(MlnType.STATE_MACHINE)
    state_machine_predicates = [Predicates.CURRENT_TASK_FINISHED, Predicates.CURRENT_TASK, Predicates.ERROR,
                                Predicates.CURRENT_PARENT_TASK, Predicates.NEXT_TASK, Predicates.NEXT_TASK_FINISHED,
                                Predicates.CURRENT_PARAMETER, Predicates.PARENT_PARAMETER, Predicates.NEXT_PARAMETER]
    formulas = _get_formula_templates_from_databases(databases, state_machine_predicates, [Predicates.CURRENT_TASK])
    formulas = _apply_replacements(formulas, [(Types.TIME_STEP, "?t")])
    for formula in formulas:
        if not [gnd_atom for gnd_atom in formula.logical_formula if gnd_atom.predicate == Predicates.NEXT_TASK]:
            formula.logical_formula.add_element(~Predicates.NEXT_TASK_FINISHED("?t"))
    formulas = _add_not_error_atom_if_necessary(formulas)
    formulas = _remove_duplicate_error_formulas(formulas)
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.NEXT_PARAMETER, [(Types.TIME_STEP, "?t0")], "?p")
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.NEXT_TASK, [(Types.TIME_STEP, "?ts")], "?nt")
    mln.append_formulas(formulas)
    return mln


def _create_task_mln(databases):
    mln = _create_mln_skeleton(MlnType.TASK)
    formulas = []
    # task_predicates = [Predicates.CURRENT_TASK, Predicates.CURRENT_PARENT_TASK, Predicates.CHILD_TASK, Predicates.ERROR,
    #                    Predicates.CURRENT_PARAMETER, Predicates.PARENT_PARAMETER, Predicates.DURATION,
    #                    Predicates.CURRENT_TASK_FINISHED]
    task_predicates = [Predicates.CURRENT_TASK, Predicates.CURRENT_PARENT_TASK, Predicates.ERROR,
                        Predicates.CURRENT_PARAMETER, Predicates.PARENT_PARAMETER, Predicates.DURATION,
                        Predicates.CURRENT_TASK_FINISHED]
    formulas += _get_formula_templates_from_databases(databases, task_predicates,
                                                      [Predicates.CURRENT_TASK, Predicates.CURRENT_TASK_FINISHED])
    formulas = _apply_replacements(formulas, [(Types.TIME_STEP, "?t"), (Types.OBJECT, "?o")])
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.ERROR, [(Types.TIME_STEP, "?t0")], "?e")
    # formulas = _add_negation_for_all_but_existing(formulas, Predicates.CHILD_TASK, [(Types.TIME_STEP, "?t0")], "?ct")

    def no_negation_of(formula, predicate):
        return not reduce(lambda r, g: r or g.predicate == predicate and g.negated, formula.logical_formula, False)
    formulas = filter(partial(no_negation_of, predicate=Predicates.CURRENT_TASK_FINISHED), formulas)
    mln.append_formulas(formulas)
    return mln


def _create_object_mln(databases):
    mln = _create_mln_skeleton(MlnType.OBJECT)
    perception_predicates = [Predicates.CURRENT_TASK, Predicates.CURRENT_PARENT_TASK, Predicates.PERCEIVED_OBJECT,
                             Predicates.OBJECT_LOCATION, Predicates.OBJECT_TYPE, Predicates.OBJECT_PROPERTY,
                             Predicates.USED_OBJECT]
    necessary_predicates = [Predicates.CURRENT_TASK, Predicates.OBJECT_TYPE]
    formulas = _get_formula_templates_from_databases(databases,perception_predicates, necessary_predicates)
    formulas = _split_objects_of_different_type(formulas, Types.OBJECT)
    formulas = _apply_replacements(formulas, [(Types.TIME_STEP, "?t"), (Types.OBJECT, "?o")])
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.OBJECT_PROPERTY, [(Types.OBJECT, "?o1")], "?op")
    formulas = _add_negation_for_all_but_existing(formulas, Predicates.OBJECT_LOCATION,
                                                  [(Types.TIME_STEP, "?t0"), (Types.OBJECT, "?o1")], "?op")
    formulas = _add_not_perceived_or_not_acted_on_if_necessary(formulas, "?t0")
    mln.append_formulas(formulas)
    return mln


def _learn_weights_and_save_mln(mln, databases, learner, filename_prefix):
    resulting_mln = learner.learn(mln, databases)
    f = open(filename_prefix + mln.name + ".mln", 'w')
    f.write(str(resulting_mln))
    f.close()


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
    return list(set(formulas))


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
    return list(set(formulas))


def _add_not_error_atom_if_necessary(formulas):
    is_error_ground_atom = lambda atom: atom.predicate == Predicates.ERROR and not atom.negated
    formulas_with_error = filter(lambda f: not not filter(is_error_ground_atom, f.logical_formula), formulas)
    for error_formula in formulas_with_error:
        errors = filter(is_error_ground_atom, error_formula.logical_formula)
        error = None
        if len(errors) > 1:
            raise Exception("Only one error is currently supported!")
        else:
            error = errors[0]
        predicates_to_remove = {Predicates.ERROR, Predicates.NEXT_TASK,
                                Predicates.NEXT_TASK_FINISHED, Predicates.NEXT_PARAMETER}
        remove_error_next = lambda f: filter(lambda g: g.predicate not in predicates_to_remove, f.logical_formula)
        formulas_are_similar = lambda pre_process, f: \
            pre_process(f) == pre_process(error_formula) and error not in f.logical_formula
        similar_formulas = filter(partial(formulas_are_similar, remove_error_next), formulas)
        for similar_formula in similar_formulas:
            similar_formula.logical_formula.add_element(~error)
    return formulas


def _remove_duplicate_error_formulas(formulas):
    to_return = formulas
    target_state_predicates = {Predicates.NEXT_TASK, Predicates.NEXT_TASK_FINISHED, Predicates.NEXT_PARAMETER}
    formulas = filter(lambda f: Predicates.ERROR in [gnd_atom.predicate for gnd_atom in f.logical_formula], formulas)
    target_predicates_in_formula = \
        lambda f: str(filter(lambda g: g.predicate in target_state_predicates, f.logical_formula))
    formulas.sort(key=target_predicates_in_formula)
    groups = groupby(formulas, key=target_predicates_in_formula)
    groups = [[(formula, set(formula.logical_formula)) for formula in group] for _, group in groups]
    for group in groups:
        for formula_set_pair in group:
            formula = formula_set_pair[0]
            predicate_is_not_error_or_target = lambda p: p!=Predicates.ERROR and p not in target_state_predicates
            formula_without_error = \
                set(filter(lambda gnd: predicate_is_not_error_or_target(gnd.predicate), formula.logical_formula))
            formula_is_unique_without_error = True
            for other_group in groups:
                if other_group == group:
                    continue
                for formula_and_set_from_other_group in other_group:
                    other_formula_set = formula_and_set_from_other_group[1]
                    if other_formula_set.issuperset(formula_without_error):
                        formula_is_unique_without_error = False
            if formula_is_unique_without_error:
                for error_atom in \
                        filter(lambda gnd_atom: gnd_atom.predicate==Predicates.ERROR, formula.logical_formula):
                    formula.logical_formula.remove_element(error_atom)
    return list(set(to_return))


def _add_not_perceived_or_not_acted_on_if_necessary(formulas, timestep):
    for formula in formulas:
        perceived = set()
        used = set()
        objects = set()
        for gnd_atom in formula.logical_formula:
            if hasattr(gnd_atom, "predicate") and hasattr(gnd_atom, "get_argument_value"):
                object = gnd_atom.get_argument_value(Types.OBJECT)
                if not object is None:
                    objects.add(object)
                    if gnd_atom.predicate == Predicates.PERCEIVED_OBJECT:
                        perceived.add(object)
                    if gnd_atom.predicate == Predicates.USED_OBJECT:
                        used.add(object)
        for object in objects:
            if object not in perceived:
                formula.logical_formula.add_element(~Predicates.PERCEIVED_OBJECT(timestep, object))
            if object not in used:
                formula.logical_formula.add_element(~Predicates.USED_OBJECT(timestep, object))
    return formulas


def _split_multiple_groundings_of_same_predicate(formulas):
    to_return = []
    for formula in formulas:
        predicate_count = dict()
        for gnd_atom in formula.logical_formula:
            predicate = gnd_atom.predicate
            predicate_count[predicate] = 1 if predicate not in predicate_count else predicate_count[predicate]+1
        duplicate_predicates = [predicate for predicate, count in predicate_count.items() if count > 1]
        if not duplicate_predicates:
            to_return.append(formula)
            continue
        if len(duplicate_predicates) > 1:
            raise Exception("Only one duplicate predicate is supported")
        duplicate_predicate = duplicate_predicates[0]
        duplicate_atoms = filter(lambda atom: atom.predicate == duplicate_predicate, formula.logical_formula)
        other_atoms = filter(lambda atom: atom.predicate != duplicate_predicate, formula.logical_formula)
        create_conjunction = lambda gnd_atoms: Conjunction([FormulaGroundAtom(gnd_atom) for gnd_atom in gnd_atoms])
        to_return += [Formula(formula.weight, create_conjunction(other_atoms + [dupl])) for dupl in duplicate_atoms]
    return to_return


def _split_objects_of_different_type(formulas, type_to_split):
    to_return = []
    create_conjunction = lambda gnd_atoms: Conjunction([FormulaGroundAtom(gnd_atom) for gnd_atom in gnd_atoms])
    for formula in formulas:
        atoms_with_type = filter(lambda atom: type_to_split in atom.predicate.types, formula.logical_formula)
        atoms_without_type = filter(lambda atom: type_to_split not in atom.predicate.types, formula.logical_formula)
        if not atoms_with_type:
            to_return.append(formula)
            continue
        atoms_with_type.sort(key=lambda atom: atom.get_argument_value(type_to_split))
        for _, group in groupby(atoms_with_type, key=lambda atom: atom.get_argument_value(type_to_split)):
            to_return.append(Formula(formula.weight, create_conjunction(atoms_without_type + list(group))))
    return to_return


def _add_expand_operator(formulas, predicates_with_star_operator):
    for formula in formulas:
        for ground_atom in formula.logical_formula:
            if ground_atom.predicate in predicates_with_star_operator:
                ground_atom.apply_expand_operator = True
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


def _add_atom_to_every_formula(formulas, atom):
    for formula in formulas:
        formula.logical_formula.add_element(atom)
    return formulas