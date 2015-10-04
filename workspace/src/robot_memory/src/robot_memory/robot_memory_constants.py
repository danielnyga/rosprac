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
    CURRENT_TASK = Predicate("currentTask", 2)
    CURRENT_TASK_FINISHED = Predicate("currentTaskFinished", 1)
    CURRENT_PARENT_TASK = Predicate("currentParentTask", 2)
    NEXT_TASK = Predicate("nextTask", 2)
    NEXT_TASK_FINISHED = Predicate("nextTaskFisnihed", 1)
    CHILD_TASK = Predicate("childTask", 2)
    ERROR = Predicate("error", 2)
    GOAL = Predicate("goal", 2)
    PERCEIVED_OBJECT = Predicate("perceivedObject", 2)
    USED_OBJECT = Predicate("usedObject", 2)
    OBJECT_PROPERTY = Predicate("objectProperty", 2)
    OBJECT_TYPE = Predicate("objectType", 2)
    OBJECT_LOCATION = Predicate("objectLocation", 3)
    DURATION = Predicate("duration", 2)
