from genpy.rostime import Duration
from robot_memory.msg import RobotState


class AbstractDuration:
    short = "Short"
    long = "Long"
    medium = "Medium"


class ExtendedRobotState:
    def __init__(self, ros_robot_state):
        # inherited properties
        self.task_name = ros_robot_state.task_name
        self.current_time = ros_robot_state.current_time
        self.errors = ros_robot_state.errors
        self.finished = ros_robot_state.finished
        self.goal = ros_robot_state.goal
        self.objects_acted_on = ros_robot_state.objects_acted_on
        self.perceived_objects = ros_robot_state.perceived_objects
        self.sequence_number = ros_robot_state.sequence_number
        self.task_id = ros_robot_state.task_id
        # own properties
        self.parent_state = None
        self.child_states = []
        self.next_state = None
        self.duration = None
        self.partner = None
        self.abstract_duration = None

    def __str__(self):
        return str(self.__dict__)


def extract_additional_relations(robot_state_messages):
    to_return = [ExtendedRobotState(message) for message in robot_state_messages]
    call_stack = [None]
    last_message = None
    for message in to_return:
        if last_message is not None:
            last_message.next_state = message
        last_message = message
        if not message.finished:
            message.parent_state = call_stack[-1]
            call_stack.append(message)
            if message.parent_state is not None:
                message.parent_state.child_states.append(message)
        else:
            if len(call_stack) < 2 or call_stack[-1].task_id != message.task_id:
                raise Exception("Did not find the partner state!")
            message.partner = call_stack.pop()
            message.partner.partner = message
            message.parent_state = message.partner.parent_state
            message.child_states = message.partner.child_states
            message.duration = message.current_time - message.partner.current_time
            message.partner.duration = message.duration
            message.abstract_duration = calculate_abstract_duration(message)
            message.partner.abstract_duration = message.abstract_duration
    return to_return


def calculate_abstract_duration(message):
    # TODO: Use clustering instead...
    if message.duration <= Duration(1):
        return AbstractDuration.short
    elif message.duration <= Duration(10):
        return AbstractDuration.medium
    else:
        return  AbstractDuration.long