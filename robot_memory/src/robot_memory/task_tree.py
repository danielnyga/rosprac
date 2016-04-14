import hashlib

class TaskTreeNode(object):
    def __init__(self, task_id, name, success, duration):
        self.__id = task_id
        self.__name = name
        self.__success = success
        self.__duration = duration
        self.__child_tasks = []
        self.__designators = []
        self.__parent_task = None

    @property
    def task_id(self):
        return self.__id

    @task_id.setter
    def task_id(self, value):
        self.__id = value

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, value):
        self.__name = value

    @property
    def success(self):
        return self.__success

    @success.setter
    def success(self, value):
        self.__success = value

    @property
    def duration(self):
        return self.__duration

    @duration.setter
    def duration(self, value):
        self.__duration = value

    @property
    def child_tasks(self):
        return self.__child_tasks

    def add_child_task(self, child_task):
        self.child_tasks.append(child_task)

    def remove_child_task(self, child_task):
        self.child_tasks.remove(child_task)

    @property
    def parent_task(self):
        return self.__parent_task

    @parent_task.setter
    def parent_task(self, parent):
        self.__parent_task = parent

    @property
    def designators(self):
        return self.__designators

    def add_designator(self, key, designator):
        self.__designators.append((key, designator))

    def __str__(self):
        return "node id=%s name=%s success=%s duration=%s" % (self.task_id, self.name, self.success, self.duration)

    def tree_as_string(self, indentation_level=0):
        return _reduce_string(lambda s1, s2: s1 +"\n"+ s2, [_indentation_string() * indentation_level + self.__str__()]+
            [_indentation_string() * (indentation_level + 1) + key + ": " +
             d.tree_as_string(indentation_level+1) for key, d in self.designators] +
            [s.tree_as_string(indentation_level+1) for s in self.child_tasks])


class TaskTreeGoalNode(TaskTreeNode):
    def __init__(self, task_id, name, success, duration):
        TaskTreeNode.__init__(self, task_id, name, success, duration)
        self.__goal_pattern = ""
        self.__goal_properties = []

    @property
    def goal_pattern(self):
        return self.__goal_pattern

    @goal_pattern.setter
    def goal_pattern(self, value):
        self.__goal_pattern = value

    @property
    def goal_properties(self):
        return self.__goal_properties

    def add_goal_property(self, key, value):
        self.__goal_properties.append([key, value])

    def replace_goal_property_value(self, old_pair, new_value):
        old_pair[1]=new_value

    def __str__(self):
        return TaskTreeNode.__str__(self) + " pattern=" + self.__goal_pattern + " " + \
            _reduce_string(lambda s1,s2: s1 + ", " + s2, [k + ": " + v for k , v in self.goal_properties])


class DesignatorType(object):
    ACTION = "action"
    OBJECT = "object"
    LOCATION = "location"
    HUMAN = "human"
    SPEECH = "speech"


class Designator:
    def __init__(self, designator_id, designator_type):
        self.__id = designator_id
        self.__type = designator_type
        self.__properties = []
        self.__designators = []

    @property
    def designator_id(self):
        return self.__id

    @property
    def designator_type(self):
        return self.__type

    @property
    def properties(self):
        return self.__properties

    def add_property(self, key, value):
        self.__properties.append([key, value])

    def replace_property_value(self, old_pair, new_value):
        old_pair[1] = new_value

    @property
    def designators(self):
        return self.__designators

    def add_designator(self, key, designator):
        keys = [d[0] for d in self.designators]
        self.__designators.append((key, designator))

    def __str__(self):
        return "designator id=%s type=%s " % (self.designator_id, self.designator_type) \
               + _reduce_string(lambda s1,s2: s1 + ", " + s2, [ k + ": " + v for k , v in self.properties])

    def tree_as_string(self, indentation_level):
        return _reduce_string(lambda s1, s2: s1+"\n"+s2, [self.__str__()]+[_indentation_string()*(indentation_level+1) +
            k + ": " + d.tree_as_string(indentation_level+1) for k, d in self.designators])

    @property
    def sha1_hash(self):
        string_representation = self.designator_type + \
                                str(self.__properties) + str([(k, v.sha1_hash) for k,v in self.__designators])
        return hashlib.sha1(string_representation).hexdigest()


def _reduce_string(function, strings):
    return "" if not strings else reduce(function, strings)


def _indentation_string():
    return " "
