import database_creator
import robot_state_preprocessor
from robot_memory_constants import Types, Predicates
from mln_elements import MLN, Formula
from functools import partial
import pracmln_adapter


class MlnType:
    state_machine = "StateMachine"
    task = "Task"
    object = "Object"
    input_output = "InputOutput"


def create_and_save_mlns(robot_state_messages):
    preprocessed_messages = robot_state_preprocessor.extract_additional_relations(robot_state_messages)
    dbs = database_creator.create_database_collection(preprocessed_messages)
    mlns = dict()
    mlns[MlnType.state_machine] = __create_state_machine_mln(dbs)
#    mlns[MlnType.task] = __create_task_mln(dbs)
#    mlns[MlnType.object] = __create_object_mln(dbs)
#    mlns[MlnType.input_output] = __create_input_output_mln(dbs)
    mlns = dict([(name, __learn_weights(mln, dbs)) for name, mln in mlns.items()])
    for name, mln in mlns.items():
        f = open(name + ".mln", 'w')
        f.write(str(mln))
        f.close()


def __create_state_machine_mln(databases):
    mln = __create_mln_skeleton()
    state_machine_predicates = [Predicates.CURRENT_TASK_FINISHED, Predicates.CURRENT_TASK, Predicates.ERROR,
                                Predicates.CURRENT_PARENT_TASK, Predicates.NEXT_TASK, Predicates.NEXT_TASK_FINISHED]
    formulas = __get_formula_templates_from_databases(databases, state_machine_predicates, [Predicates.NEXT_TASK])
    is_error_ground_atom = lambda atom: atom.get_predicate() == Predicates.ERROR and not atom.is_negated()
    formulas_with_error = filter(lambda f: not not filter(is_error_ground_atom, f), formulas)
    for error_formula in formulas_with_error:
        errors = filter(is_error_ground_atom, error_formula)
        error = None
        if len(errors) > 1:
            raise(Exception("Only one error is currently supported!")) #TODO: Support more errors?!
        else:
            error = errors[0]
        remove_error = lambda f: filter(lambda g: not g.get_predicate() == Predicates.ERROR, f)
        similar_formulas = filter(lambda f: remove_error(f) == remove_error(error_formula) and error not in f, formulas)
        for similar_formula in similar_formulas:
            similar_formula.add_ground_atom_to_conjunction(~error)
    for f in formulas:
        mln.append_formula(f)
    return str(mln)


def __create_task_mln(databases):
    return __create_mln_skeleton() # TODO


def __create_object_mln(databases):
    return __create_mln_skeleton() # TODO


def __create_input_output_mln(databases):
    return __create_mln_skeleton() # TODO


def __learn_weights(mln, databases):
    return pracmln_adapter.learn(str(mln), str(databases))


def __create_mln_skeleton():
    mln = MLN()
    for predicate in Predicates.get_all_predicates():
        mln.append_predicate(predicate)
    return mln


def __get_formula_templates_from_databases(databases, possible_preds, necessary_preds):
    predicate_contained_in_formula = lambda gnd_atom: gnd_atom.get_predicate() in possible_preds
    formula_atoms = [filter(predicate_contained_in_formula, list(db)) for db in databases]
    list_contains_element = lambda cond, lst: not not filter(cond, lst)
    predicate_is_necessary = lambda gnd_atom: gnd_atom.get_predicate() in necessary_preds
    formula_atoms = filter(partial(list_contains_element, predicate_is_necessary), formula_atoms)
    formulas = [Formula(0.0, ground_atom_list) for ground_atom_list in formula_atoms]
    for formula in formulas:
        for ground_atom in formula:
            if Types.TIME_STEP in ground_atom.get_predicate().get_types():
                ground_atom.set_argument_value(Types.TIME_STEP, "t")
    return set(formulas)