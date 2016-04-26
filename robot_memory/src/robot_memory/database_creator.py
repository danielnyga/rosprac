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
        all_sub_designators = _get_all_designators_as_one_list(task)
        for designator in all_sub_designators:
            _append_to_db(db, Predicates.DESIGNATOR_HASH, designator.designator_id, designator.sha1_hash)
            _append_to_db(db, Predicates.DESIGNATOR_TYPE, designator.designator_id, designator.designator_type)
            for key, value in designator.properties:
                property_id = str(uuid.uuid4())
                _append_to_db(db, Predicates.DESIGNATOR_PROPERTY, property_id, designator.designator_id)
                _append_to_db(db, Predicates.PROPERTY_KEY, property_id, key)
                _append_to_db(db, Predicates.PROPERTY_VALUE, property_id, value)
            for key, sub_designator in designator.designators:
                property_id = str(uuid.uuid4())
                _append_to_db(db, Predicates.DESIGNATOR_PROPERTY, property_id, designator.designator_id)
                _append_to_db(db, Predicates.PROPERTY_KEY, property_id, key)
                _append_to_db(db, Predicates.PROPERTY_VALUE, property_id, sub_designator.sha1_hash)
        if not all_sub_designators:
            dummy_designator_id = str(uuid.uuid4())
            db.append(DomainDeclaration(Types.DESIGNATOR, dummy_designator_id))
        for key, designator in task.designators:
            _append_to_db(db, Predicates.GOAL_DESIGNATOR, task.task_id, key, designator.sha1_hash)
        if hasattr(task, "goal_pattern"):
            _append_to_db(db, Predicates.GOAL_PATTERN, task.task_id, task.goal_pattern)
        if hasattr(task, "goal_properties"):
            for key, value in task.goal_properties:
                _append_to_db(db, Predicates.GOAL_PROPERTY, task.task_id, key, value)
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
    if "\n" in str(string):
        string = string.replace("\n", "_")
        print("Warning: newline replaced by underscore in " + string)
    if "\"" in str(string):
        string = string.replace("\"", "_")
        print("Warning: quotation mark replaced by underscore in " + string)
    return "\"" + string + "\""
