from robot_memory.mln_elements import DatabaseCollection, Database
from robot_memory.robot_memory_constants import Predicates
import re
import uuid


def create_database_collection(root_tasks):
    to_return = DatabaseCollection()

    for task in _get_all_tasks_as_one_list(root_tasks):
        db = Database()
        to_return.append(db)
        _append_to_db(db, Predicates.TASK_NAME, task.task_id, task.name)
        if task.success:
            _append_to_db(db, Predicates.TASK_SUCCESS, task.task_id)
        for designator in _get_all_designators_as_one_list(task):
            _append_to_db(db, Predicates.DESIGNATOR_HASH, designator.designator_id, designator.sha1_hash)
            _append_to_db(db, Predicates.DESIGNATOR_TYPE, designator.designator_id, designator.designator_type)
            for key, value in designator.properties:
                _append_to_db(db, Predicates.DESIGNATOR_PROPERTY, designator.designator_id, key, value)
            for key, sub_designator in designator.designators:
                _append_to_db(db, Predicates.SUB_DESIGNATOR,designator.designator_id, key, sub_designator.sha1_hash)
        for key, designator in task.designators:
            designator_id = uuid.uuid4()
            _append_to_db(db, Predicates.DESIGNATOR_OR_VALUE, task.task_id, key, designator_id)
            _append_to_db(db, Predicates.GOAL_PROPERTY_DESIGNATOR, designator_id, designator.sha1_hash)
        if hasattr(task, "goal_pattern"):
            _append_to_db(db, Predicates.GOAL_PATTERN, task.task_id, task.goal_pattern)
        if hasattr(task, "goal_properties"):
            for key, value in task.goal_properties:
                property_id = uuid.uuid4()
                _append_to_db(db, Predicates.DESIGNATOR_OR_VALUE, task.task_id, key, property_id)
                _append_to_db(db, Predicates.GOAL_PROPERTY_VALUE, property_id, value)
    return to_return


def _append_to_db(db, predicate, *arguments):
    escaped_arguments = [_escape(arg) for arg in arguments]
    db.append(predicate(*escaped_arguments))


def _get_all_tasks_as_one_list(root_tasks):
    def get_sub_tasks_as_list(task):
        return [task] + reduce(lambda l1, l2: l1+l2, [get_sub_tasks_as_list(t) for t in task.child_tasks], [])

    return reduce(lambda l1, l2: l1+l2, [get_sub_tasks_as_list(root_task) for root_task in root_tasks], [])


def _get_all_designators_as_one_list(task):
    def get_designators_as_list(d):
        return [d] + reduce(lambda l1, l2: l1+l2, [get_designators_as_list(child) for _, child in d.designators], [])

    return reduce(lambda l1,l2: l2+l2, [get_designators_as_list(d) for _,d in task.designators], [])


def _escape(string):
    return re.sub("\W", "_", str(string)).capitalize()
    #TODO: Use base64 encoding...
