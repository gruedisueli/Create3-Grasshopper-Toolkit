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
        MessagePlaySong("Dolores", [440,494,523,587,659,699,784,880,880,784,699,659,587,523,494,440], [1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000]),
        MessagePlaySong("Maeve", [880,784,699,659,587,523,494,440,440,494,523,587,659,699,784,880], [1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000,1000000000])]),
]


def main(args = None):
    #setting the booleans to true allows the script to override whatever speed settings you are copying into the program above
    #if for some reason you want to explicitly control speeds, set these to False.
    runscene(moves)

    

if __name__ == '__main__':
    main()

