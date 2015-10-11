#!/usr/bin/env python2

import subprocess
import os
import rospy
import time
from robot_memory.srv import LearningTrigger
from std_srvs.srv import Trigger
from robot_memory.msg import RobotState, Object, ObjectProperty


def main():
    test = RobotStateTest()
    test.execute()


class RobotStateTest:
    def execute(self):
        service_process = subprocess.Popen(["rosrun", "robot_memory", "service", "debug"], env=os.environ)
        self.__send_test_messages()
        service_process.terminate()
        service_process.wait()

    def __send_test_messages(self):
        rospy.init_node("robot_memory_test")
        publisher = rospy.Publisher('robot_memory/state', RobotState, queue_size=9999)
        time.sleep(2) # TODO: avoiding deadlocks using sleeps is bad style, but this is just a test...
        start_collecting = rospy.ServiceProxy("robot_memory/start_collecting_training_data", Trigger)
        response = start_collecting()
        if not response.success:
            raise Exception("Start Trigger unsuccessful")
        sent_messages = 0
        for message in self.__get_test_messages():
            publisher.publish(message)
            sent_messages += 1
        learn = rospy.ServiceProxy("robot_memory/learn", LearningTrigger)
        response = learn(sent_messages)
        if not response.success:
            raise Exception("Learn Trigger unsuccessful")

    def __get_test_messages(self):
        pancakemaker0 = Object(object_type="Pancakemaker",
                               object_location="Cupboard",
                               object_id="PM",
                               properties=[
                                   ObjectProperty(property_name="Color",
                                                  property_value="Green")])
        spatula0 = Object(object_type="Spatula",
                          object_location="Cupboard",
                          object_id="SP",
                          properties=[
                              ObjectProperty(property_name="Form",
                                             property_value="Long")])
        spatula1 = Object(object_type="Spatula",
                          object_location="Sink",
                          object_id="SP",
                          properties=[
                              ObjectProperty(property_name="Color",
                                             property_value="Yellow")])
        plate0 = Object(object_type="Plate",
                        object_location="Sink",
                        object_id="PL",
                        properties=[
                            ObjectProperty(property_name="Form",
                                           property_value="Round")])
        plate1 = Object(object_type="Plate",
                        object_location="Sink",
                        object_id="PL")
        plate2 = Object(object_type="Plate",
                        object_location="Corridor",
                        object_id="PL")
        plate3 = Object(object_type="Plate",
                        object_location="DiningRoom",
                        object_id="PL")
        to_return = [
            RobotState(sequence_number=1,
                       task_id="Displace1",
                       task_name="Displace",
                       goal="ObjectAtLocation",
                       finished=False,
                       current_time=rospy.Time(0)),
            RobotState(sequence_number=2,
                       task_id="Perceive1",
                       task_name="Perceive",
                       errors=["ObjectNotFound"],
                       perceived_objects=[spatula0, pancakemaker0],
                       finished=False,
                       current_time=rospy.Time(1)),
            RobotState(sequence_number=3,
                       task_id="Perceive1",
                       task_name="Perceive",
                       errors=["ObjectNotFound"],
                       perceived_objects=[spatula0, pancakemaker0],
                       finished=True,
                       current_time=rospy.Time(2)),
            RobotState(sequence_number=4,
                       task_id="Perceive2",
                       task_name="Perceive",
                       perceived_objects=[spatula1, plate0],
                       finished=False,
                       current_time=rospy.Time(3)),
            RobotState(sequence_number=5,
                       task_id="Perceive2",
                       task_name="Perceive",
                       perceived_objects=[spatula1, plate0],
                       finished=True,
                       current_time=rospy.Time(4)),
            RobotState(sequence_number=6,
                       task_id="Pick1",
                       task_name="Pick",
                       goal="ObjectPicked",
                       objects_acted_on=[plate1],
                       finished=False,
                       current_time=rospy.Time(5)),
            RobotState(sequence_number=7,
                       task_id="Pick1",
                       task_name="Pick",
                       goal="ObjectPicked",
                       objects_acted_on=[plate1],
                       finished=True,
                       current_time=rospy.Time(7)),
            RobotState(sequence_number=8,
                       task_id="Move1",
                       task_name="Move",
                       objects_acted_on=[plate2],
                       finished=False,
                       current_time=rospy.Time(8)),
            RobotState(sequence_number=9,
                       task_id="Move1",
                       task_name="Move",
                       objects_acted_on=[plate2],
                       finished=True,
                       current_time=rospy.Time(10)),
            RobotState(sequence_number=10,
                       task_id="Move2",
                       task_name="Move",
                       objects_acted_on=[plate3],
                       finished=False,
                       current_time=rospy.Time(11)),
            RobotState(sequence_number=11,
                       task_id="Move2",
                       task_name="Move",
                       objects_acted_on=[plate3],
                       finished=True,
                       current_time=rospy.Time(22)),
            RobotState(sequence_number=12,
                       task_id="Place1",
                       task_name="Place",
                       goal = "ObjectPlaced",
                       objects_acted_on=[plate3],
                       finished=False,
                       current_time=rospy.Time(23)),
            RobotState(sequence_number=13,
                       task_id="Place1",
                       task_name="Place",
                       goal = "ObjectPlaced",
                       objects_acted_on=[plate3],
                       finished=True,
                       current_time=rospy.Time(25)),
            RobotState(sequence_number=14,
                       task_id="Displace1",
                       task_name="Displace",
                       goal="ObjectAtLocation",
                       finished=True,
                       current_time=rospy.Time(26))
        ]
        return to_return

if __name__ == "__main__":
    main()
