from mln_elements import Type, Predicate

class Types:
    TIME_STEP = Type("timeStep")
    TASK_TYPE = Type("taskType")
    ERROR = Type("error")
    GOAL = Type("goal")
    OBJECT = Type("object")
    OBJECT_PROPERTY = Type("objectProperty")
    OBJECT_TYPE = Type("objectType")
    OBJECT_LOCATION = Type("location")
    DURATION = Type("duration")


class Predicates:
    CURRENT_TASK = Predicate("currentTask", Types.TIME_STEP, -Types.TASK_TYPE)
    CURRENT_TASK_FINISHED = Predicate("currentTaskFinished", Types.TIME_STEP)
    CURRENT_PARENT_TASK = Predicate("currentParentTask", Types.TIME_STEP, -Types.TASK_TYPE)
    NEXT_TASK = Predicate("nextTask", Types.TIME_STEP, -Types.TASK_TYPE)
    NEXT_TASK_FINISHED = Predicate("nextTaskFinished", Types.TIME_STEP)
    CHILD_TASK = Predicate("childTask", Types.TIME_STEP, Types.TASK_TYPE)
    ERROR = Predicate("error", Types.TIME_STEP, -Types.ERROR)
    GOAL = Predicate("goal", Types.TIME_STEP, -Types.GOAL)
    PERCEIVED_OBJECT = Predicate("perceivedObject", Types.TIME_STEP, Types.OBJECT)
    USED_OBJECT = Predicate("usedObject", Types.TIME_STEP, Types.OBJECT)
    OBJECT_PROPERTY = Predicate("objectProperty", Types.OBJECT, Types.OBJECT_PROPERTY)
    OBJECT_TYPE = Predicate("objectType", Types.OBJECT, -Types.OBJECT_TYPE)
    OBJECT_LOCATION = Predicate("objectLocation", Types.TIME_STEP, Types.OBJECT, -Types.OBJECT_LOCATION)
    DURATION = Predicate("duration", Types.TIME_STEP, -Types.DURATION)

    @staticmethod
    def get_all_predicates():
        return filter(lambda p: isinstance(p, Predicate), Predicates.__dict__.values())