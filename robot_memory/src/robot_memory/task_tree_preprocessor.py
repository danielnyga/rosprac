import re


def preprocess_task_tree(root_nodes):
    pruned_tree = _remove_nodes(root_nodes, [
        lambda n: n.name.lower() == "with-failure-handling",
        lambda n: n.name.lower() == "with-designators",
        lambda n: n.name.lower() == "perform-action-designator",
        lambda n: n.name.lower() == "anonymous-top-level",
        lambda n: n.name.lower() == "motion-planning",
        lambda n: n.name.lower() == "find-objects",
        lambda n: n.name.lower() == "at-location",
        lambda n: n.name.lower() == "monitor-action",
    ])
    return _round_numbers(root_nodes)


def _remove_nodes(nodes, predicates_to_remove):
    to_return = []

    def remove_nodes_recursively(node_to_examine):
        for child in list(node_to_examine.child_tasks):
            remove_nodes_recursively(child)
        parent = node_to_examine.parent_task
        if reduce(lambda a, b: a or b, [p(node_to_examine) for p in predicates_to_remove], False):
            for child_task in node_to_examine.child_tasks:
                child_task.parent_task = parent
                if parent is not None:
                    parent.add_child_task(child_task)
                else:
                    to_return.append(child_task)
            if parent is not None:
                parent.remove_child_task(node_to_examine)
        elif parent is None:
            to_return.append(node_to_examine)

    for node in nodes:
        remove_nodes_recursively(node)
    return to_return


def _round_numbers(nodes):
    def round_number_if_necessary(number):
        if not re.match("[0-9]\.[0-9]", number):
            return number
        number_as_float = float(number)
        return "%.3f" % number_as_float

    def round_numbers_recursively(node):
        if hasattr(node, "goal_properties"):
            for key_value_pair in node.goal_properties:
                node.replace_goal_property_value(key_value_pair, round_number_if_necessary(key_value_pair[1]))
        if hasattr(node, "designators"):
            for key, designator in node.designators:
                round_designator_recursively(designator)
        for child in list(node.child_tasks):
            round_numbers_recursively(child)

    def round_designator_recursively(designator):
        for key_value_pair in designator.properties:
            designator.replace_property_value(key_value_pair, round_number_if_necessary(key_value_pair[1]))
        for key, child_designator in designator.designators:
            round_designator_recursively(child_designator)

    for n in nodes:
        round_numbers_recursively(n)
    return nodes
