import time
import threading
import traceback
import rospy
from robot_memory.task_tree import TaskTreeNode, TaskTreeGoalNode, Designator, DesignatorType
from robot_memory.pracmln_adapter import PracmlnAdapter
from robot_memory import mln_creator
from std_srvs.srv import Trigger, TriggerResponse
from task_tree_msgs.msg import RobotState, Designator as RosDesignator
from task_tree_msgs.srv import LearningTrigger, LearningTriggerResponse
from sys import argv


def start_ros_service():
    service = RobotMemoryService()
    service.run()


class RobotMemoryService(object):
    def __init__(self):
        self.__mutex = threading.Lock()

    def run(self):
        rospy.init_node("robot_memory")
        rospy.Service("robot_memory/start_collecting_training_data", Trigger, self.__start_collecting_training_data)
        rospy.Service("robot_memory/learn", LearningTrigger, self.__start_learning)
        rospy.Subscriber("robot_memory/state", RobotState, self.__robot_state_received)
        rospy.loginfo("Entering main loop...")
        rospy.spin()

    def __robot_state_received(self, robot_state):
        with self.__mutex:
            self.__collected_messages.append(robot_state)

    def __start_collecting_training_data(self, request):
        with self.__mutex:
            rospy.loginfo("Start collecting messages...")
            self.__collected_messages = []
        return TriggerResponse(success=True)

    def __start_learning(self, request):
        with self.__mutex:
            rospy.loginfo("Start learning...")
            number_of_received_messages = len(self.__collected_messages)
        rospy.loginfo("waiting for messages to arrive...")
        while number_of_received_messages < request.number_of_required_messages:
            rospy.loginfo("Received %i of %i messages",
                          number_of_received_messages, request.number_of_required_messages)
            time.sleep(1)
            with self.__mutex:
                number_of_received_messages = len(self.__collected_messages)
        with self.__mutex:
            messages = list(self.__collected_messages)
        rospy.loginfo("Learning...")
        try:
            debug = "debug" in argv
            root_nodes = _create_task_trees(messages)
#            learner = PracmlnAdapter()
            learner = None
            mln_creator.create_and_save_mlns(root_nodes, learner, debug, rospy)
        except Exception:
            rospy.logfatal(traceback.format_exc())
            return LearningTriggerResponse(success=False)
        rospy.loginfo("Learning finished!")
        return LearningTriggerResponse(success=True)


def _create_task_trees(messages):
    to_return = []
    messages.sort(key=lambda m: m.sequence_number)
    call_stack = [None]
    last_message = None
    for message in messages:
        if not message.finished:
            node_class = TaskTreeGoalNode if message.goal_context != "" or message.goal_properties else TaskTreeNode
            tree_node = node_class(message.task_id, message.task_name, message.success, message.current_time)
            tree_node.parent_task = call_stack[-1]
            call_stack.append(tree_node)
            if tree_node.parent_task is not None:
                tree_node.parent_task.add_child_task(tree_node)
            else:
                to_return.append(tree_node)
        else:
            if len(call_stack) < 2 or call_stack[-1].task_id != message.task_id:
                raise Exception("Did not find the partner state!")
            tree_node = call_stack.pop()
            tree_node.name = message.task_name
            tree_node.success = message.success
            tree_node.duration = message.current_time - tree_node.duration
            designators = {}
            for designator in message.aggregated_child_designators:
                if designator.id in designators:
                    print("Warning: Duplicate designator ID:" + designator.id)
                    continue
                designators[designator.id] = designator
            _add_designators(tree_node, message.designator_key_to_designator_id, designators)
            if hasattr(tree_node, "goal_pattern"):
                tree_node.goal_pattern = message.goal_context
            if hasattr(tree_node, "add_goal_property"):
                for prop in message.goal_properties:
                    tree_node.add_goal_property(prop.name, prop.value)
    return to_return


def _add_designators(designator_parent, child_keys_and_ids, designator_map):
    for child_key_and_id in child_keys_and_ids:
        if child_key_and_id.value not in designator_map:
            raise Exception("Unknown designator id: " + child_key_and_id.value)
        designator_message = designator_map[child_key_and_id.value]
        designator_type_mapping = {RosDesignator.TYPE_ACTION: DesignatorType.ACTION,
                                   RosDesignator.TYPE_HUMAN: DesignatorType.HUMAN,
                                   RosDesignator.TYPE_LOCATION: DesignatorType.LOCATION,
                                   RosDesignator.TYPE_OBJECT: DesignatorType.OBJECT,
                                   RosDesignator.TYPE_SPEECH: DesignatorType.SPEECH}
        if designator_message.type not in designator_type_mapping:
            raise Exception("Unknown designator type: " + str(designator_message.type))
        designator = Designator(designator_message.id, designator_type_mapping[designator_message.type])
        for property_key_and_value in designator_message.properties:
            designator.add_property(property_key_and_value.name, property_key_and_value.value)
        _add_designators(designator, designator_message.designator_key_to_designator_id, designator_map)
        designator_parent.add_designator(child_key_and_id.name, designator)




