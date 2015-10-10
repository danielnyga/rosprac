import database_creator
import robot_state_preprocessor
from robot_memory_constants import Types, Predicates
from mln_elements import MLN, Formula
from functools import partial
import pracmln_adapter
from itertools import groupby


class MlnType:
    state_machine = "StateMachine"
    task = "Task"
    object = "Object"
    input_output = "InputOutput"


def create_and_save_mlns(robot_state_messages, debug=False):
    preprocessed_messages = robot_state_preprocessor.extract_additional_relations(robot_state_messages)
    dbs = database_creator.create_database_collection(preprocessed_messages)
    mlns = [
        __create_state_machine_mln(dbs),
        __create_task_mln(dbs),
        __create_input_output_mln(dbs),
        __create_object_mln(dbs)
    ]
    if debug:
        for mln in mlns:
            mln.save(mln.name + "_before_learning")
        dbs.save("train")
    for mln in mlns:
        __learn_weights_and_save_mln(mln, dbs)


def __create_state_machine_mln(databases):
    mln = __create_mln_skeleton(MlnType.state_machine)
    state_machine_predicates = [Predicates.CURRENT_TASK_FINISHED, Predicates.CURRENT_TASK, Predicates.ERROR,
                                Predicates.CURRENT_PARENT_TASK, Predicates.NEXT_TASK, Predicates.NEXT_TASK_FINISHED]
    formulas = __get_formula_templates_from_databases(databases, state_machine_predicates, [Predicates.NEXT_TASK])
    formulas = __apply_replacements(formulas, [(Types.TIME_STEP, "t")])
    __add_not_error_atom_if_necessary(formulas)
    mln.append_formulas(formulas)
    return mln


def __create_task_mln(databases):
    mln = __create_mln_skeleton(MlnType.task)
    goal_predicates = [Predicates.GOAL, Predicates.CURRENT_TASK]
    formulas = __get_formula_templates_from_databases(databases, goal_predicates, goal_predicates)
    formulas += [Formula(0.0, ~Predicates.GOAL("t", "g"))]
    child_predicates = [Predicates.CURRENT_TASK, Predicates.CHILD_TASK]
    formulas += __get_formula_templates_from_databases(databases, child_predicates, child_predicates)
    formulas += [Formula(0.0, ~Predicates.CHILD_TASK("t1", "t2"))]
    error_predicates = [Predicates.CURRENT_TASK, Predicates.ERROR]
    formulas += __get_formula_templates_from_databases(databases, error_predicates, error_predicates)
    formulas += [Formula(0.0, ~Predicates.ERROR("t", "e"))]
    duration_predicates = [Predicates.CURRENT_TASK, Predicates.DURATION]
    formulas += __get_formula_templates_from_databases(databases, duration_predicates, duration_predicates)
    formulas = __split_multiple_groundings_of_same_predicate(formulas)
    formulas = __apply_replacements(formulas, [(Types.TIME_STEP, "t")])
    mln.append_formulas(formulas)
    return mln


def __create_input_output_mln(databases):
    mln = __create_mln_skeleton(MlnType.input_output)
    perception_predicates = [Predicates.CURRENT_TASK, Predicates.PERCEIVED_OBJECT,
                             Predicates.OBJECT_LOCATION, Predicates.OBJECT_TYPE]
    formulas = __get_formula_templates_from_databases(databases,perception_predicates, perception_predicates)
    formulas += [Formula(0.0, ~Predicates.PERCEIVED_OBJECT("t", "o"))]
    action_predicates = [Predicates.CURRENT_TASK, Predicates.USED_OBJECT, Predicates.OBJECT_TYPE]
    formulas += __get_formula_templates_from_databases(databases, action_predicates, action_predicates)
    formulas += [Formula(0.0, ~Predicates.USED_OBJECT("t", "o"))]
    formulas = __split_objects_of_different_type(formulas, Types.OBJECT)
    formulas = __apply_replacements(formulas, [(Types.TIME_STEP, "t"), (Types.OBJECT, "o")])
    mln.append_formulas(formulas)
    return mln


def __create_object_mln(databases):
    mln = __create_mln_skeleton(MlnType.object)
    property_predicates = [Predicates.PERCEIVED_OBJECT, Predicates.OBJECT_TYPE, Predicates.OBJECT_PROPERTY]
    formulas = __get_formula_templates_from_databases(databases, property_predicates, property_predicates)
    formulas += [Formula(0.0, ~Predicates.OBJECT_PROPERTY("o", "p"))]
    formulas = __split_objects_of_different_type(formulas, Types.OBJECT)
    formulas = __split_multiple_groundings_of_same_predicate(formulas)
    formulas = __apply_replacements(formulas, [(Types.TIME_STEP, "t"), (Types.OBJECT, "o")])
    mln.append_formulas(formulas)
    return mln


def __learn_weights_and_save_mln(mln, databases):
    resulting_mln = pracmln_adapter.learn(str(mln), str(databases))
    f = open(mln.name + ".mln", 'w')
    f.write(resulting_mln)
    f.close()


def __create_mln_skeleton(mln_name):
    mln = MLN(mln_name)
    for predicate in Predicates.get_all_predicates():
        mln.append_predicate(predicate)
    return mln


def __get_formula_templates_from_databases(databases, possible_preds, necessary_preds):
    predicate_contained_in_formula = lambda gnd_atom: gnd_atom.predicate in possible_preds
    formula_atoms = [filter(predicate_contained_in_formula, list(db)) for db in databases]
    predicate_is_in_list = lambda gnd_atoms, pred: pred in [g.predicate for g in gnd_atoms]
    atoms_contain_necessary_elements = lambda atoms: \
        reduce(lambda a, b: a and b, [predicate_is_in_list(atoms, pred) for pred in necessary_preds])
    formula_atoms = filter(atoms_contain_necessary_elements, formula_atoms)
    formulas = [Formula(0.0, *ground_atom_list) for ground_atom_list in formula_atoms]
    return list(set(formulas))


def __apply_replacements(formulas, formula_replacements):
    for formula in formulas:
        for ground_atom in formula:
            for replacement in formula_replacements:
                constant_type = replacement[0]
                new_value = replacement[1]
                if constant_type in ground_atom.predicate.types:
                    ground_atom.set_argument_value(constant_type, new_value)
    return list(set(formulas))


def __add_not_error_atom_if_necessary(formulas):
    is_error_ground_atom = lambda atom: atom.predicate == Predicates.ERROR and not atom.negated
    formulas_with_error = filter(lambda f: not not filter(is_error_ground_atom, f), formulas)
    for error_formula in formulas_with_error:
        errors = filter(is_error_ground_atom, error_formula)
        error = None
        if len(errors) > 1:
            raise Exception("Only one error is currently supported!")  # TODO: Support more errors?!
        else:
            error = errors[0]
        predicates_to_remove = {Predicates.ERROR, Predicates.NEXT_TASK, Predicates.NEXT_TASK_FINISHED}
        remove_error = lambda f: filter(lambda g: g.predicate != Predicates.ERROR, f)
        remove_error_next = lambda f: filter(lambda g: g.predicate not in predicates_to_remove, f)
        formulas_are_similar = lambda pre_process, f: pre_process(f) == pre_process(error_formula) and error not in f
        similar_formulas = filter(partial(formulas_are_similar, remove_error_next), formulas)
        similar_formulas = filter(lambda f: not formulas_are_similar(remove_error, f), similar_formulas)
        for similar_formula in similar_formulas:
            similar_formula.add_ground_atom_to_conjunction(~error)


def __split_multiple_groundings_of_same_predicate(formulas):
    to_return = []
    for formula in formulas:
        predicate_count = dict()
        for gnd_atom in formula:
            predicate = gnd_atom.predicate
            predicate_count[predicate] = 0 if predicate not in predicate_count else predicate_count[predicate]+1
        duplicate_predicates = [predicate for predicate, count in predicate_count.items() if count > 1]
        if not duplicate_predicates:
            to_return.append(formula)
            continue
        if len(duplicate_predicates) > 1:
            raise Exception("Only one duplicate predicate is supported")
        duplicate_predicate = duplicate_predicates[0]
        duplicate_atoms = filter(lambda atom: atom.predicate == duplicate_predicate, formula)
        other_atoms = filter(lambda atom: atom.predicate != duplicate_predicate, formula)
        to_return += [Formula(formula.weight, *(other_atoms + [duplicate_atom])) for duplicate_atom in duplicate_atoms]
    return to_return


def __split_objects_of_different_type(formulas, type_to_split):
    to_return = []
    for formula in formulas:
        atoms_with_type = filter(lambda atom: type_to_split in atom.predicate.types, formula)
        atoms_without_type = filter(lambda atom: type_to_split not in atom.predicate.types, formula)
        if not atoms_with_type:
            to_return.append(formula)
            continue
        atoms_with_type.sort(key=lambda atom: atom.get_argument_value(type_to_split))
        for _, group in groupby(atoms_with_type, key=lambda atom: atom.get_argument_value(type_to_split)):
            to_return.append(Formula(formula.weight, *(atoms_without_type + list(group))))
    return to_return
