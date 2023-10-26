# Create3-Grasshopper-Toolkit
A toolkit for generating robot instructions from within Rhino3d's Grasshopper plugin.

## Installation instructions:

### Grasshopper
1. Install Rhino 7
1. Open Rhino
1. Run "grasshopper" command
1. In the Grasshopper window go to "File" --> "Special Folders" --> "Components Folder"
1. In the explorer window that pops up, copy the GHA for this assembly into that folder
    1. If you downloaded this GHA, make sure it is "unblocked" by right clicking on it, going to "properties" and selecting "unblock" in the window that pops up.
1. Restart Rhino and Grasshopper to load the assembly.

### Ros2 Installation on either Raspberry Pi or Virtual Machine:
1. If running on a Raspberry Pi
    1. Purchase a Raspberry Pi 4B with at least 4gb of memory and a micro SD card, preferably with a very high read/write speed (Class 10) and at least 32gb memory.
    1. Install Ubuntu 22.04 LTS 64bit Desktop on the SD card using the directions found [here](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4)
        1. In the raspberry pi tool on your PC, select "other general purpose OS" --> "Ubuntu" --> "Ubuntu Desktop 22.04 64bit" 
        1. Select destination drive (SD card)
        1. Click "write"
        1. When it's done, you can eject the SD card and insert it into your Pi.
1. If setting up a virtual machine:
    1. Download Oracle VM Virtual Box (or your preferred virtual machine software)
    1. Set up a virtual machine with at least 4gb of RAM, 2 CPUs, and 32gb of storage.
    1. Install Ubuntu 22.04 TS 64 bit Desktop on it from an image you download.
    1. Make sure your Virtual Machine's network adapter settings are set to "bridged" to prevent communication issues with the Create3. You can change these settings in Virtual Box.
1. After Ubuntu is set up
1. Start your system (either Pi or virtual machine)
1. If you need to set up user credentials, do so
1. After you're logged into the Desktop, open a terminal window
1. Follow the directions found [here](https://iroboteducation.github.io/create3_docs/setup/ubuntu2204/) to install Ros2 on your system.
    1. Make sure you install the desktop version of Humble
    1. Use the option for cyclonedds middleware, or whatever middleware your Create3 is running.
1. After installation is complete, you should be able to test out ros2 by running the command "ros2 topic list" from the terminal window. It may not show any nodes, if your robots are not yet on your network, but at least it should not give you errors. If there is an error message, installation was not successful
1. Install VS code. For a virtual machine, you can follow directions you find online for installing the software, for Raspberry Pi, you need to perform a special install because the Pi runs on an ARM processor and the typical install won't work:
    1. Follow the installation instructions found [here](https://phoenixnap.com/kb/install-vscode-ubuntu), using "Method 2", EXCEPT IN STEP 4 of "Method 2" do this:
        1. In "Step 4" change the command to say "set\[arm64\]" instead of the default. This will find the correct install package for your processor.
    1. After you install code, verify that it works by typing "code ." in the terminal and making sure the VS code opens
1. Set up the Ros2 syntax highlighting for VS code [here](https://www.youtube.com/watch?v=hf76VY0a5Fk)
1. Install git, using the directions for "install git on Linux" [here](https://www.atlassian.com/git/tutorials/install-git)

### Highly recommended tutorials on Ros2!
1. There is an OUTSTANDING tutorial for Ros2 found [here](https://www.youtube.com/watch?v=idQb2pB-h2Q)
1. Additionally, you can set up syntax highlighting for VS Code with Ros2 [here](https://www.youtube.com/watch?v=hf76VY0a5Fk)
1. The [Ros2 Humble Docs](https://docs.ros.org/en/humble/index.html) are also excellent and provide both high level and detailed information.

### Setup of Create3 robot
1. Please refer to iRobot's extensive documentation found [here](https://iroboteducation.github.io/create3_docs/)
1. Also, the [iRobot Learning Library](https://edu.irobot.com/learning-library) contains hours of useful tutorials.
