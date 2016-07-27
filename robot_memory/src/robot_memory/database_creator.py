from robot_memory.mln_elements import DatabaseCollection, Database, DomainDeclaration
from robot_memory.robot_memory_constants import Predicates
import itertools
import uuid


def create_database_collection(root_tasks):
    to_return = DatabaseCollection()

    for task in _get_all_tasks_as_one_list(root_tasks):
        def create_db_for_designator(goal_key, designator):
            db = Database()
            to_return.append(db)
            _append_to_db(db, Predicates.TASK_NAME, task.task_id, task.name)
            _append_to_db(db, Predicates.TASK_FAILURE, task.task_id, " " if task.failure is None else task.failure)
            if hasattr(task, "goal_pattern"):
                _append_to_db(db, Predicates.GOAL_PATTERN, task.task_id, task.goal_pattern)
            else:
                _append_to_db(db, Predicates.GOAL_PATTERN, task.task_id, " ")
            _append_to_db(db, Predicates.GOAL_PARAMETER, designator.designator_id, task.task_id)
            _append_to_db(db, Predicates.GOAL_PARAMETER_KEY, designator.designator_id, goal_key)
            return db

        for goal_key, designator in task.designators:
            for combination in itertools.combinations(_get_full_scoped_key_and_value(designator), 2):
                db = create_db_for_designator(goal_key, designator)
                for key, value in combination:
                    property_id = str(uuid.uuid4())
                    _append_to_db(db, Predicates.DESIGNATOR_PROPERTY, property_id, designator.designator_id)
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


def _get_full_scoped_key_and_value(designator, previous_keys_as_string=""):
    key = ("" if previous_keys_as_string == "" else previous_keys_as_string + "::") + designator.designator_type + "."
    sub_designators = [_get_full_scoped_key_and_value(d, key + k) for k, d in designator.designators]
    flattened_sub_designators = [kv_pair for kv_pairs in sub_designators for kv_pair in kv_pairs]
    key_value_pairs = designator.properties if hasattr(designator, "properties") else []
    properties = [(key+k, v) for k, v in key_value_pairs]
    return flattened_sub_designators + properties


def _escape(string):
    if "\n" in str(string):
        string = string.replace("\n", "_")
        print("Warning: newline replaced by underscore in " + string)
    if "\"" in str(string):
        string = string.replace("\"", "_")
        print("Warning: quotation mark replaced by underscore in " + string)
    return "\"" + string + "\""
