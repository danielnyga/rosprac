from robot_memory.mln_elements import Type, Predicate


class Types(object):
    DESIGNATOR_OR_VALUE = Type("designatorOrValue")
    DESIGNATOR_TYPE = Type("designatorType")
    TASK = Type("task")
    TASK_NAME = Type("taskName")
    GOAL_PATTERN = Type("goalPattern")
    GOAL_PROPERTY_VALUE = Type("goalPropertyValue")
    DESIGNATOR_PROPERTY_VALUE = Type("value")
    PROPERTY_KEY = Type("propertyKey")
    SUB_DESIGNATOR_KEY = Type("subDesignatorKey")
    GOAL_KEY = Type("goalKey")


class Predicates(object):
    TASK_NAME = Predicate("name", Types.TASK, +Types.TASK_NAME)
    TASK_SUCCESS = Predicate("success", Types.TASK)
    GOAL_PATTERN = Predicate("goalPattern", Types.TASK, -Types.GOAL_PATTERN)
    DESIGNATOR_OR_VALUE = Predicate("designatorOrValue", Types.TASK, Types.GOAL_KEY, -Types.DESIGNATOR_OR_VALUE)
    DESIGNATOR = Predicate("designator", Types.DESIGNATOR_OR_VALUE, -Types.DESIGNATOR_TYPE)
    GOAL_PROPERTY_VALUE = Predicate("goalPropertyValue", Types.DESIGNATOR_OR_VALUE, -Types.GOAL_PROPERTY_VALUE)
    DESIGNATOR_PROPERTY = \
        Predicate("designatorProperty", Types.DESIGNATOR_OR_VALUE, Types.PROPERTY_KEY, -Types.DESIGNATOR_PROPERTY_VALUE)
    SUB_DESIGNATOR = \
        Predicate("subDesignator", Types.DESIGNATOR_OR_VALUE, Types.SUB_DESIGNATOR_KEY, -Types.DESIGNATOR_OR_VALUE)

    @staticmethod
    def get_all_predicates():
        return filter(lambda p: isinstance(p, Predicate), Predicates.__dict__.values())