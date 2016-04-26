from robot_memory.mln_elements import Type, Predicate


class Types(object):
    TASK = Type("task")
    TASK_NAME = Type("taskName")
    GOAL_PATTERN = Type("goalPattern")
    GOAL_KEY = Type("goalKey")
    GOAL_PROPERTY_VALUE = Type("goalPropertyValue")
    DESIGNATOR = Type("designator")
    DESIGNATOR_TYPE = Type("designatorType")
    DESIGNATOR_HASH = Type("designatorHash")
    DESIGNATOR_PROPERTY_KEY = Type("designatorPropertyKey")
    DESIGNATOR_PROPERTY_VALUE = Type("designatorPropertyValue")
    SUB_DESIGNATOR_KEY = Type("subDesignatorKey")


class Predicates(object):
    TASK_NAME = Predicate("name", Types.TASK, +Types.TASK_NAME)
    TASK_SUCCESS = Predicate("success", Types.TASK)
    GOAL_PATTERN = Predicate("goalPattern", Types.TASK, -Types.GOAL_PATTERN)
    GOAL_PROPERTY = Predicate("goalProperty", Types.TASK, Types.GOAL_KEY, Types.GOAL_PROPERTY_VALUE)
    GOAL_DESIGNATOR = Predicate("goalDesignator", Types.TASK, Types.GOAL_KEY, Types.DESIGNATOR_HASH)
    DESIGNATOR_TYPE = Predicate("designatorType", Types.DESIGNATOR, -Types.DESIGNATOR_TYPE)
    DESIGNATOR_HASH = Predicate("designatorHash", Types.DESIGNATOR, -Types.DESIGNATOR_HASH)
    DESIGNATOR_PROPERTY = Predicate(
        "designatorProperty", Types.DESIGNATOR, Types.DESIGNATOR_PROPERTY_KEY, Types.DESIGNATOR_PROPERTY_VALUE)
    SUB_DESIGNATOR = Predicate(
        "subDesignator", Types.DESIGNATOR, Types.SUB_DESIGNATOR_KEY, Types.DESIGNATOR_HASH)

    @staticmethod
    def get_all_predicates():
        return filter(lambda p: isinstance(p, Predicate), Predicates.__dict__.values())