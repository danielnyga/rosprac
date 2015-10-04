import database_creator
import robot_state_preprocessor
from robot_memory_constants import Types, Predicates
from mln_elements import MLN


class MlnType:
    state_machine = "StateMachine"
    task = "Task"
    object = "Object"
    input_output = "InputOutput"


def create_and_save_mlns(robot_state_messages):
    preprocessed_messages = robot_state_preprocessor.extract_additional_relations(robot_state_messages)
    dbs = database_creator.create_database_collection(preprocessed_messages)
    mlns = dict()
    mlns[MlnType.state_machine] = __create_state_machine_mln(preprocessed_messages)
    mlns[MlnType.task] = __create_task_mln(preprocessed_messages)
    mlns[MlnType.object] = __create_object_mln(preprocessed_messages)
    mlns[MlnType.input_output] = __create_input_output_mln(preprocessed_messages)
    mlns = dict([(name, __learn_weights(mln, dbs)) for name, mln in mlns.items()])
    for name, mln in mlns.items():
        f = open(name + ".mln", 'w')
        f.write(str(mln))
        f.close()


def __create_state_machine_mln(preprocessed_messages):
    return __create_mln_skeleton() # TODO


def __create_task_mln(preprocessed_messages):
    return __create_mln_skeleton() # TODO


def __create_object_mln(preprocessed_messages):
    return __create_mln_skeleton() # TODO


def __create_input_output_mln(preprocessed_messages):
    return __create_mln_skeleton() # TODO


def __learn_weights(mln, databases):
    return str(mln) + "\n\n" + str(databases) # TODO


def __create_mln_skeleton():
    mln = MLN()
    mln.append_predicate(Predicates.CURRENT_TASK(Types.TIME_STEP, -Types.TASK_TYPE))
    mln.append_predicate(Predicates.CURRENT_TASK_FINISHED(Types.TIME_STEP))
    mln.append_predicate(Predicates.CURRENT_PARENT_TASK(Types.TIME_STEP, -Types.TASK_TYPE))
    mln.append_predicate(Predicates.NEXT_TASK(Types.TIME_STEP, -Types.TASK_TYPE))
    mln.append_predicate(Predicates.NEXT_TASK_FINISHED(Types.TIME_STEP))
    mln.append_predicate(Predicates.CHILD_TASK(Types.TIME_STEP, Types.TASK_TYPE))
    mln.append_predicate(Predicates.ERROR(Types.TIME_STEP, -Types.ERROR))
    mln.append_predicate(Predicates.GOAL(Types.TIME_STEP, -Types.GOAL))
    mln.append_predicate(Predicates.PERCEIVED_OBJECT(Types.TIME_STEP, Types.OBJECT))
    mln.append_predicate(Predicates.USED_OBJECT(Types.TIME_STEP, Types.OBJECT))
    mln.append_predicate(Predicates.OBJECT_PROPERTY(Types.OBJECT, Types.OBJECT_PROPERTY))
    mln.append_predicate(Predicates.OBJECT_TYPE(Types.OBJECT, -Types.OBJECT_TYPE))
    mln.append_predicate(Predicates.OBJECT_LOCATION(Types.TIME_STEP, Types.OBJECT, -Types.OBJECT_LOCATION))
    mln.append_predicate(Predicates.DURATION(Types.TIME_STEP, -Types.DURATION))
    return mln