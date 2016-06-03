from robot_memory.mln_elements import DatabaseCollection, Database, DomainDeclaration
from robot_memory.robot_memory_constants import Predicates, Types
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
        for key, designator, value in _get_full_scoped_key_designator_and_value(task):
            property_id = str(uuid.uuid4())
            _append_to_db(db, Predicates.DESIGNATOR_PROPERTY, property_id, task.task_id)
            _append_to_db(db, Predicates.DESIGNATOR_TYPE, property_id, designator.designator_type)
            _append_to_db(db, Predicates.DESIGNATOR_PROPERTY_KEY, property_id, key)
            _append_to_db(db, Predicates.DESIGNATOR_PROPERTY_VALUE, property_id, value)
        if hasattr(task, "goal_pattern"):
            _append_to_db(db, Predicates.GOAL_PATTERN, task.task_id, task.goal_pattern)
        if hasattr(task, "goal_properties"):
            for key, value in task.goal_properties:
                property_id = str(uuid.uuid4())
                _append_to_db(db, Predicates.DESIGNATOR_PROPERTY, property_id, task.task_id)
                _append_to_db(db, Predicates.DESIGNATOR_TYPE, property_id, "none")
                _append_to_db(db, Predicates.DESIGNATOR_PROPERTY_KEY, property_id, key)
                _append_to_db(db, Predicates.DESIGNATOR_PROPERTY_VALUE, property_id, value)
    return to_return


def _append_to_db(db, predicate, *arguments):
    escaped_arguments = [_escape(arg) for arg in arguments]
    db.append(predicate(*escaped_arguments))


def _get_all_tasks_as_one_list(root_tasks):
    def get_sub_tasks_as_list(task):
        return [task] + reduce(lambda l1, l2: l1+l2, [get_sub_tasks_as_list(t) for t in task.child_tasks], [])

    return reduce(lambda l1, l2: l1+l2, [get_sub_tasks_as_list(root_task) for root_task in root_tasks], [])


def _get_full_scoped_key_designator_and_value(task_or_designator, previous_keys_as_string=""):
    key = "" if previous_keys_as_string == "" else previous_keys_as_string + "::"
    sub_designators = [_get_full_scoped_key_designator_and_value(d, key + k) for k, d in task_or_designator.designators]
    get_designator = lambda d: d if previous_keys_as_string == "" else task_or_designator
    flattened_sub_designators = [(k, get_designator(d), v) for kv_pairs in sub_designators for k, d, v in kv_pairs]
    key_value_pairs = task_or_designator.properties if hasattr(task_or_designator, "properties") else []
    properties = [(key+k, task_or_designator, v) for k, v in key_value_pairs]
    return flattened_sub_designators + properties


def _escape(string):
    if "\n" in str(string):
        string = string.replace("\n", "_")
        print("Warning: newline replaced by underscore in " + string)
    if "\"" in str(string):
        string = string.replace("\"", "_")
        print("Warning: quotation mark replaced by underscore in " + string)
    return "\"" + string + "\""
