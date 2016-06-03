from robot_memory.mln_elements import Type, Predicate


class Types(object):
    TASK = Type("task")
    TASK_NAME = Type("taskName")
    GOAL_PATTERN = Type("goalPattern")
    DESIGNATOR_TYPE = Type("designatorType")
    DESIGNATOR_PROPERTY = Type("designatorProperty")
    DESIGNATOR_PROPERTY_KEY = Type("designatorPropertyKey")
    DESIGNATOR_PROPERTY_VALUE = Type("designatorPropertyValue")


class Predicates(object):
    TASK_NAME = Predicate("name", Types.TASK, +Types.TASK_NAME)
    TASK_SUCCESS = Predicate("success", Types.TASK)
    GOAL_PATTERN = Predicate("goalPattern", Types.TASK, -Types.GOAL_PATTERN)
    DESIGNATOR_PROPERTY = Predicate("designatorProperty", Types.DESIGNATOR_PROPERTY, +Types.TASK) # TODO: Insert first property here ?!
                                                                                                  # TODO: Add additional formulas combining property + task...
                                                                                                  # TODO: Add count to measure how many propertys are needed...
    DESIGNATOR_PROPERTY_KEY = Predicate("propertyKey", Types.DESIGNATOR_PROPERTY, +Types.DESIGNATOR_PROPERTY_KEY)
    DESIGNATOR_PROPERTY_VALUE = Predicate("propertyValue", Types.DESIGNATOR_PROPERTY, +Types.DESIGNATOR_PROPERTY_VALUE)
    DESIGNATOR_TYPE = Predicate("designatorType", Types.DESIGNATOR_PROPERTY, +Types.DESIGNATOR_TYPE)

    @staticmethod
    def get_all_predicates():
        return filter(lambda p: isinstance(p, Predicate), Predicates.__dict__.values())