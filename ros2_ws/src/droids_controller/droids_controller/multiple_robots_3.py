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

moves = [
    SequenceStep([
        MessageDriveStraight("Dolores", 2.7376, 0.0750),
        MessageDriveStraight("Maeve", -1.0229, 0.0750),
        MessageDriveStraight("Teddy", 2.7376, 0.0750)]),
    SequenceStep([
        MessageRotate("Dolores", 1.5726, 0.2000),
        MessageRotate("Maeve", 1.5717, 0.2000),
        MessageRotate("Teddy", 1.5726, 0.2000)]),
    SequenceStep([
        MessageDriveArc("Dolores", 1.5690, 3.1170, 0.0750, True),
        MessageDriveArc("Maeve", -3.1425, 1.6741, 0.0750, True),
        MessageDriveArc("Teddy", 1.5713, 3.1170, 0.0750, True)]),
    SequenceStep([
        MessageWait("Dolores"),
        MessageDriveArc("Maeve", 3.1416, 1.5626, 0.0750, True),
        MessageWait("Teddy")]),
]

def main(args = None):
    #setting the booleans to true allows the script to override whatever speed settings you are copying into the program above
    #if for some reason you want to explicitly control speeds, set these to False.
    runscene(moves)

    

if __name__ == '__main__':
    main()

