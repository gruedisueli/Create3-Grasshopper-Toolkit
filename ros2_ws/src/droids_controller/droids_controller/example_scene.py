'''
This is a sample script for a robot scene
Make a copy of it and rename to whatever you want
Add this name to the "entry points" list in setup.py
Then rebuild from the CLI: navigate to the parent directory to the "src" directory, then run "colcon build --symlink-install"
Then you can call your script by running "ros2 run droids_controller {your script name}", for example: "ros2 run droids_controller example_scene"
'''

#!/usr/bin/env python3

from .sync_drive import SequenceStep
from .sync_drive import MessageDriveStraight
from .sync_drive import MessageDriveArc
from .sync_drive import MessageRotate
from .sync_drive import MessageWait
from .sync_drive import MessagePlaySong
from .sync_drive import runscene

#moves is a list of SequenceSteps. 
#each sequence step includes multiple messages (one per robot in the scene)
#message types are defined in "sync_drive" and imported here
#refer to "sync_drive" for the meaning of the arguments provided to each message
#make sure float arguments do not have too many decimal places--this can cause the robots to stop responding

#"moves" can be conveniently generated from drawn curves in Rhino3d using our Grasshopper plugin

#below are some test scripts you can uncomment out one-at-a-time and see how the robots respond
#you may need to replace the robot names if you are working with different ones
# moves = [
#     SequenceStep([
#         MessageRotate("Maeve", -0.4711, 0.2000)]),
#     SequenceStep([
#         MessageDriveStraight("Maeve", 0.5000, 0.0750)]),
#     SequenceStep([
#         MessageRotate("Maeve", -1.5708, 0.2000)]),
#     SequenceStep([
#         MessageDriveStraight("Maeve", 0.5000, 0.0750)]),
#     SequenceStep([
#         MessageRotate("Maeve", -1.5708, 0.2000)]),
#     SequenceStep([
#         MessageDriveStraight("Maeve", 0.5000, 0.0750)]),
#     SequenceStep([
#         MessageRotate("Maeve", -1.5708, 0.2000)]),
#     SequenceStep([
#         MessageDriveStraight("Maeve", 0.4445, 0.0750)]),
# ]

# moves = [
#     SequenceStep([
#         MessageRotate("Maeve", -1.1249, 0.2000)]),
#     SequenceStep([
#         MessageDriveArc("Maeve", -2.3620, 0.8385, 0.0750, True)]),
# ]

# moves = [
#     SequenceStep([
#         MessageRotate("Maeve", -1.1249, 0.2000),
#         MessageRotate("Teddy", 3.1416, 0.2000)]),
#     SequenceStep([
#         MessageDriveArc("Maeve", -2.3620, 0.8385, 0.0750, True),
#         MessageRotate("Teddy", -3.1416, 0.2000)]),
# ]

# moves = [
#     SequenceStep([
#         MessageRotate("Maeve", -1.1249, 0.2000),
#         MessageRotate("Teddy", 3.1416, 0.2000)]),
#     SequenceStep([
#         MessageDriveArc("Maeve", -2.3620, 0.8385, 0.0750, True),
#         MessageWait("Teddy")]),
# ]

moves = [
    SequenceStep([
        MessageRotate("Maeve", -1.1249, 0.2000)]),
    SequenceStep([
        MessageDriveArc("Maeve", -2.3620, 0.2626, 0.0750, True)]),
    SequenceStep([
        MessageDriveArc("Maeve", 1.3449, 0.1536, 0.0750, True)]),
    SequenceStep([
        MessageRotate("Maeve", 1.6708, 0.2000)]),
    SequenceStep([
        MessageDriveStraight("Maeve", 0.1206, 0.0750)]),
    SequenceStep([
        MessageRotate("Maeve", -1.6708, 0.2000)]),
    SequenceStep([
        MessageDriveArc("Maeve", -1.3449, 0.1536, 0.0750, False)]),
    SequenceStep([
        MessageDriveArc("Maeve", 2.3620, 0.2626, 0.0750, False)]),
    SequenceStep([
        MessagePlaySong("Maeve", [440,220,440], [1000000000,500000000,1000000000])]),
]


def main(args = None):
    #setting the booleans to true allows the script to override whatever speed settings you are copying into the program above
    #if for some reason you want to explicitly control speeds, set these to False.
    runscene(moves)

    

if __name__ == '__main__':
    main()

