import robot_state_preprocessor
from mln_elements import DatabaseCollection, Database
from robot_memory_constants import Predicates


def create_database_collection(preprocessed_states):
    to_return = DatabaseCollection()
    preprocessed_states = __remove_duplicate_leaf_nodes(preprocessed_states)
    for preprocessed_state in preprocessed_states:
        db = Database()
        to_return.append(db)
        current_time = preprocessed_state.current_time.secs
        db.append(Predicates.CURRENT_TASK(current_time, preprocessed_state.task_name))
        if preprocessed_state.finished:
            db.append(Predicates.CURRENT_TASK_FINISHED(current_time))
        else:
            db.append(~Predicates.CURRENT_TASK_FINISHED(current_time))
        if preprocessed_state.parent_state:
            db.append(Predicates.CURRENT_PARENT_TASK(current_time, preprocessed_state.parent_state.task_name))
        if preprocessed_state.next_state is not None:
            db.append(Predicates.NEXT_TASK(current_time, preprocessed_state.next_state.task_name))
        if preprocessed_state.next_state is not None:
            if preprocessed_state.next_state.finished:
                db.append(Predicates.NEXT_TASK_FINISHED(current_time))
            else:
                db.append(~Predicates.NEXT_TASK_FINISHED(current_time))
        for child in preprocessed_state.child_states:
            db.append(Predicates.CHILD_TASK(current_time, child.task_name))
        if len(preprocessed_state.errors) > 1:
            raise Exception("More than one error is currently not supported!")  # TODO: Change MLN!
        elif len(preprocessed_state.errors) == 1:
            db.append(Predicates.ERROR(current_time, preprocessed_state.errors[0]))
        if preprocessed_state.goal != "":
            db.append(Predicates.GOAL(current_time, preprocessed_state.goal))
        db.append(Predicates.DURATION(current_time, preprocessed_state.abstract_duration))
        objects = []
        for object in preprocessed_state.perceived_objects:
            db.append(Predicates.PERCEIVED_OBJECT(current_time, object.object_id))
            objects.append(object)
        for object in preprocessed_state.objects_acted_on:
            db.append(Predicates.USED_OBJECT(current_time, object.object_id))
            objects.append(object)
        for object in objects:  # TODO: filter duplicate objects
            db.append(Predicates.OBJECT_TYPE(object.object_id, object.object_type))
            db.append(Predicates.OBJECT_LOCATION(current_time, object.object_id, object.object_location))
            for prop in object.properties:  # TODO: Change MLN to support property keys
                db.append(Predicates.OBJECT_PROPERTY(object.object_id, prop.property_value))
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