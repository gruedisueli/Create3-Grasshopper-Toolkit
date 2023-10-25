'''
This script allows control of multiple robots at once, executing actions in sequence
'''

#!/usr/bin/env python3

import rclpy
from threading import Thread
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import DriveArc
from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNoteVector
from irobot_create_msgs.msg import AudioNote
from builtin_interfaces.msg import Duration

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Parameter

#define your robot namespaces here.
#robot names are configured in the web app on the robot itself
ROBOT_A = 'Maeve'
ROBOT_B = 'Dolores'
ROBOT_C = 'Teddy'
ROBOT_D = 'Bernard'

class RobotMessage:
    def __init__(self, robotName:str):
        self._robotName = robotName

class MessageDriveStraight(RobotMessage):
    def __init__(self, robotName:str, distanceMeters:float, maxSpeedMetersPerSecond:float):
        super().__init__(robotName)
        self._distanceMeters = distanceMeters
        self._maxSpeedMetersPerSecond = maxSpeedMetersPerSecond

class MessageRotate(RobotMessage):
    def __init__(self, robotName:str, angleRadians:float, maxRotationSpeedRadiansPerSecond:float):
        super().__init__(robotName)
        self._angleRadians = angleRadians
        self._maxRotationSpeedRadiansPerSecond = maxRotationSpeedRadiansPerSecond

class MessageDriveArc(RobotMessage):
    def __init__(self, robotName:str, arcAngleRadians:float, arcRadiusMeters:float, maxSpeedMetersPerSecond:float, isForwards:bool):
        super().__init__(robotName)
        self._arcAngleRadians = arcAngleRadians
        self._arcRadiusMeters = arcRadiusMeters
        self._maxSpeedMetersPerSecond = maxSpeedMetersPerSecond
        self._isForwards = isForwards

class MessageWait(RobotMessage):
    def __init__(self, robotName:str):
        super().__init__(robotName)

class MessagePlaySong(RobotMessage):
    def __init__(self, robotName:str, frequencyHzList, durationNanoSecList):
        super().__init__(robotName)
        self._frequencyHzList = frequencyHzList
        self._durationNanoSecList = durationNanoSecList


class SequenceStep:
    def __init__(self, messageList):
        self._messageList = messageList

class RobotActionClient(Node):
    def __init__(self, robotNamespace:str):
        super().__init__(robotNamespace + '_robot_action_client')
        self._robot = robotNamespace
        self._driveDistanceClient = ActionClient(self, DriveDistance, robotNamespace + '/drive_distance')
        self._rotateAngleClient = ActionClient(self, RotateAngle, robotNamespace + '/rotate_angle')
        self._driveArcClient = ActionClient(self, DriveArc, robotNamespace + '/drive_arc')
        self._sendSongClient = ActionClient(self, AudioNoteSequence, robotNamespace + '/audio_note_sequence')
        self._isReady = True
        self._client = self.create_client(SetParameters, robotNamespace + '/motion_control/set_parameters')
        print (self._robot + ': initialized')

    def sendMsg(self, robotMsg:RobotMessage):
        if isinstance(robotMsg, MessageDriveStraight):
            self.sendDrive(robotMsg)
        elif isinstance(robotMsg, MessageRotate):
            self.sendRotate(robotMsg)
        elif isinstance(robotMsg, MessageDriveArc):
            self.sendArc(robotMsg)
        elif isinstance(robotMsg, MessagePlaySong):
            self.sendSong(robotMsg._frequencyHzList, robotMsg._durationNanoSecList)
        #don't need to do anything with a "wait" message. The robot can remain "ready"

    def sendSong(self, frequencyHzList, durationNanoSecList):
        if len(frequencyHzList) != len(durationNanoSecList):
            print("ERROR: song frequency list length does not match duration list length")
            return
        print (self._robot + ": sending song")
        self._isReady = False
        goalMsg = AudioNoteSequence.Goal()
        vector = AudioNoteVector()
        notes = []
        for i in range(0, len(frequencyHzList)):
            note = AudioNote()
            note.frequency = frequencyHzList[i]
            duration = Duration()
            duration.sec = 0
            duration.nanosec = durationNanoSecList[i]
            note.max_runtime = duration
            notes.append(note)
        vector.notes = notes
        goalMsg.note_sequence = vector
        print (self._robot + ': waiting for audio not sequence server to become available...')
        self._sendSongClient.wait_for_server()
        self._send_goal_future = self._sendSongClient.send_goal_async(goalMsg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def sendDrive(self, driveStraightMessage:MessageDriveStraight):
        print (self._robot + ": sending drive info")
        self._isReady = False
        goalMsg = DriveDistance.Goal()
        goalMsg.distance = driveStraightMessage._distanceMeters
        goalMsg._max_translation_speed = driveStraightMessage._maxSpeedMetersPerSecond
        print (self._robot + ': waiting for drive distance server to become available...')
        self._driveDistanceClient.wait_for_server()
        self._send_goal_future = self._driveDistanceClient.send_goal_async(goalMsg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def sendRotate(self, rotateMessage:MessageRotate):
        print (self._robot + ": sending rotation info")
        self._isReady = False
        goalMsg = RotateAngle.Goal()
        goalMsg.max_rotation_speed = rotateMessage._maxRotationSpeedRadiansPerSecond
        goalMsg.angle = rotateMessage._angleRadians
        print (self._robot + ': waiting for rotate angle server to become available...')
        self._rotateAngleClient.wait_for_server()
        self._send_goal_future = self._rotateAngleClient.send_goal_async(goalMsg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def sendArc(self, arcMessage:MessageDriveArc):
        print (self._robot + ": sending arc info")
        self._isReady = False
        goalMsg = DriveArc.Goal()
        goalMsg.angle = arcMessage._arcAngleRadians
        goalMsg.radius = arcMessage._arcRadiusMeters
        goalMsg.max_translation_speed = arcMessage._maxSpeedMetersPerSecond
        goalMsg.translate_direction = 1 if arcMessage._isForwards else -1
        print (self._robot + ': waiting for drive arc server to become available...')
        self._driveArcClient.wait_for_server()
        self._send_goal_future = self._driveArcClient.send_goal_async(goalMsg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        print(self._robot + ': checking if goal was accepted or rejected...')
        goalHandle = future.result()
        if not goalHandle.accepted:
            self.get_logger().info('goal rejected')
            return
        self.get_logger().info(self._robot + ': goal accepted')
        self._get_result_future = goalHandle.get_result_async() #log the result
        self._get_result_future.add_done_callback(self.get_result_callback) #new callback for when we have result logged

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(self._robot + ': Result: {0}'.format(result))
        self._isReady = True

    def set_params(self, mode:str):
        request = SetParameters.Request()
        param = Parameter()
        param.name = "safety_override"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = mode
        request.parameters.append(param)

        print('waiting for service to override safeties')

        self._client.wait_for_service()
        print('request sent')
        self._future = self._client.call_async(request)

    def shutdown(self):
        self.set_params('none')
        print('reset safety overrides')

def runscene(moves):
    
    #first we scan the entire program and find all the robots involved
    #we also check that messages are of known types to reduce the chances of errors downstream
    haveA = False
    haveB = False
    haveC = False
    haveD = False
    for step in moves:
        for msg in step._messageList:
            isKnownType = isinstance(msg, MessageDriveStraight) or isinstance(msg, MessageDriveArc) or isinstance(msg, MessageRotate) or isinstance(msg, MessageWait) or isinstance(msg, MessagePlaySong)
            if not isKnownType:
                print("ERROR: Unknown message type. Check message type names in the set of input moves")
                return
            if msg._robotName == ROBOT_A:
                haveA = True
            elif msg._robotName == ROBOT_B:
                haveB = True
            elif msg._robotName == ROBOT_C:
                haveC = True
            elif msg._robotName == ROBOT_D:
                haveD = True
            else:
                print("ERROR: Unknown robot name. Check robot name in the set of input moves.")
                return
            
    rclpy.init()
    
    activeRobots = [RobotActionClient(ROBOT_A) if haveA else None,
                    RobotActionClient(ROBOT_B) if haveB else None,
                    RobotActionClient(ROBOT_C) if haveC else None,
                    RobotActionClient(ROBOT_D) if haveD else None]

    executor = MultiThreadedExecutor() #syncrhonized operations in ros2 don't work without multithreading

    for robot in activeRobots:
        if robot == None:
            continue
        robot.set_params('full') #safety overrides so we can drive backwards without them stopping
        executor.add_node(robot)

    executorThread = Thread(target=executor.spin, daemon=True)
    executorThread.start()

    step = 0
    maxStep = len(moves)
    print("MOVES",moves)
    print("MAX STEP:",maxStep)

    #sort of "wake up" the robots. we observed that the first command can response can be delayed causing synchronization issues
    for robot in activeRobots:
        if robot == None:
            continue
        robot.sendSong([50], [1000000000])

    startStep = True
    try:
        while (rclpy.ok):
            
            allReady = True
            for robot in activeRobots:
                if robot == None:
                    continue
                if not robot._isReady:
                    allReady = False
                    break

            if not allReady:
                continue

            #send "song" at start of each step consisting of a single short note.
            #there are bugs associated with sending multiple of the same command in a row.
            #this hack gets around this issue
            #don't worry, the note is so low that you shouldn't hear it / the roomba speaker probably can't play it.
            if(startStep):
                for robot in activeRobots:
                    if robot == None:
                        continue
                    robot.sendSong([1], [1000000000])
                startStep = False
            else:
                for message in moves[step]._messageList:
                    n = message._robotName
                    if n == ROBOT_A:
                        activeRobots[0].sendMsg(message)
                    elif n == ROBOT_B:
                        activeRobots[1].sendMsg(message)
                    elif n == ROBOT_C:
                        activeRobots[2].sendMsg(message)
                    elif n == ROBOT_D:
                        activeRobots[3].sendMsg(message)
                step = step + 1
                startStep = True
                if step >= maxStep:
                    break

    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
        pass

    print('Shutting down robot nodes')
    for robot in activeRobots:
        if robot == None:
            continue
        robot.shutdown
    print('Robot nodes shut down')

    rclpy.shutdown()
    print('rclpy shutdown')
    executorThread.join()
    print('thread joined')



