from mln_elements import DatabaseCollection, Database
from robot_memory_constants import Predicates
import re


def create_database_collection(preprocessed_states):
    to_return = DatabaseCollection()
    preprocessed_states = __remove_duplicate_leaf_nodes(preprocessed_states)
    for preprocessed_state in preprocessed_states:
        db = Database()
        to_return.append(db)
        current_time = preprocessed_state.current_time.secs
        db.append(Predicates.CURRENT_TASK(current_time, __escape(preprocessed_state.task_name)))
        if preprocessed_state.finished:
            db.append(Predicates.CURRENT_TASK_FINISHED(current_time))
        else:
            db.append(~Predicates.CURRENT_TASK_FINISHED(current_time))
        if preprocessed_state.parent_state:
            db.append(Predicates.CURRENT_PARENT_TASK(current_time, __escape(preprocessed_state.parent_state.task_name)))
            for parameter in preprocessed_state.parent_state.parameters:
                db.append(Predicates.PARENT_PARAMETER(current_time, __escape(parameter.name),__escape(parameter.value)))
        if preprocessed_state.next_state is not None:
            db.append(Predicates.NEXT_TASK(current_time, __escape(preprocessed_state.next_state.task_name)))
            for parameter in preprocessed_state.next_state.parameters:
                db.append(Predicates.NEXT_PARAMETER(current_time, __escape(parameter.name), __escape(parameter.value)))
        if preprocessed_state.next_state is not None:
            if preprocessed_state.next_state.finished:
                db.append(Predicates.NEXT_TASK_FINISHED(current_time))
            else:
                db.append(~Predicates.NEXT_TASK_FINISHED(current_time))
        for child in preprocessed_state.child_states:
            db.append(Predicates.CHILD_TASK(current_time, __escape(child.task_name)))
        if preprocessed_state.error != "":
            db.append(Predicates.ERROR(current_time, __escape(preprocessed_state.error)))
        db.append(Predicates.DURATION(current_time, __escape(preprocessed_state.abstract_duration)))
        for parameter in preprocessed_state.parameters:
            db.append(Predicates.CURRENT_PARAMETER(current_time, __escape(parameter.name), __escape(parameter.value)))
        objects = []
        for object in preprocessed_state.perceived_objects:
            db.append(Predicates.PERCEIVED_OBJECT(current_time, __escape(object.object_id)))
            objects.append(object)
        for object in preprocessed_state.used_objects:
            db.append(Predicates.USED_OBJECT(current_time, __escape(object.object_id)))
            objects.append(object)
        for object in objects:  # TODO: filter duplicate objects
            if object.object_type != "":
                db.append(Predicates.OBJECT_TYPE(__escape(object.object_id), __escape(object.object_type)))
            if object.object_location != "":
                db.append(Predicates.OBJECT_LOCATION(
                    current_time, __escape(object.object_id), __escape(object.object_location)))
            for prop in object.properties:
                db.append(Predicates.OBJECT_PROPERTY(__escape(object.object_id), __escape(prop.name),
                                                     __escape(prop.value)))
    return to_return


def __remove_duplicate_leaf_nodes(preprocessed_states):
    to_return = []
    for preprocessed_state in preprocessed_states:
        if not preprocessed_state.child_states:
            if preprocessed_state.finished:
                continue
            else:
                preprocessed_state.next_state = preprocessed_state.partner.next_state
                preprocessed_state.finished = True
        to_return.append(preprocessed_state)
    return  to_return


def __escape(string):
    return re.sub("\W", "_", str(string)).capitalize()
