from robot_memory.mln_elements import Type, Predicate


class Types(object):
    TASK = Type("task")
    TASK_NAME = Type("taskName")
    GOAL_PATTERN = Type("goalPattern")
    GOAL_PARAMETER_KEY = Type("goalParameter")
    DESIGNATOR = Type("designator")
    DESIGNATOR_PROPERTY = Type("designatorProperty")
    DESIGNATOR_PROPERTY_KEY = Type("designatorPropertyKey")
    DESIGNATOR_PROPERTY_VALUE = Type("designatorPropertyValue")


class Predicates(object):
    TASK_NAME = Predicate("name", Types.TASK, +Types.TASK_NAME)
    TASK_SUCCESS = Predicate("success", Types.TASK)
    GOAL_PATTERN = Predicate("goalPattern", Types.TASK, +Types.GOAL_PATTERN)
    GOAL_PARAMETER = Predicate("goalParameter", Types.DESIGNATOR, +Types.TASK)
    GOAL_PARAMETER_KEY = Predicate("goalParameterKey", Types.DESIGNATOR, +Types.GOAL_PARAMETER_KEY)
    DESIGNATOR_PROPERTY = Predicate("designatorProperty", Types.DESIGNATOR_PROPERTY, +Types.DESIGNATOR)
    DESIGNATOR_PROPERTY_KEY = Predicate("propertyKey", Types.DESIGNATOR_PROPERTY, +Types.DESIGNATOR_PROPERTY_KEY)
    DESIGNATOR_PROPERTY_VALUE = Predicate("propertyValue", Types.DESIGNATOR_PROPERTY, +Types.DESIGNATOR_PROPERTY_VALUE)

    @staticmethod
    def get_all_predicates():
        return filter(lambda p: isinstance(p, Predicate), Predicates.__dict__.values())