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
        MessageDriveStraight("Dolores", 0.6844, 0.0750),
        MessageDriveStraight("Maeve", 0.6844, 0.0750)]),
    SequenceStep([
        MessageRotate("Dolores", 1.5726, 0.2000),
        MessageRotate("Maeve", 1.5726, 0.2000)]),
    SequenceStep([
        MessageDriveArc("Dolores", 1.5690, 0.7793, 0.0750, True),
        MessageDriveArc("Maeve", 1.5713, 0.7793, 0.0750, True)]),
    SequenceStep([
        MessageDriveStraight("Dolores", 0.2901, 0.0750),
        MessageDriveStraight("Maeve", 0.2883, 0.0750)]),
    SequenceStep([
        MessageRotate("Dolores", -1.5708, 0.2000),
        MessageRotate("Maeve", -1.5731, 0.2000)]),
    SequenceStep([
        MessageDriveStraight("Dolores", -0.4040, 0.0750),
        MessageDriveStraight("Maeve", -0.4034, 0.0750)]),
]


def main(args = None):
    #setting the booleans to true allows the script to override whatever speed settings you are copying into the program above
    #if for some reason you want to explicitly control speeds, set these to False.
    runscene(moves)

    

if __name__ == '__main__':
    main()

