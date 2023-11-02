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
1. Install Python3 "sudo apt install python3"
1. set the correct version of setup tools "pip install setuptools==58.2.0" because you might get the error outlined [here](https://answers.ros.org/question/348083/error-ros2-run-package-not-found/). You might have to install pip first.
1. Verify contents of bashrc contains following lines at the bottom, by running the command "gedit ~/.bashrc", adding them if missing:
    ```
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```

### Recommended use of the ros2 workspace in this project:
1. Create a [fork](https://docs.github.com/en/get-started/quickstart/fork-a-repo) of this repository so that you can make your own additions to robot behaviors.
1. Create a [clone](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository) of your forked repository on your local machine, using SSH if you want to be able to push changes back to it.
1. Navigate to the ros2_ws subdirectory of the project in your local clone of the repo.
1. Run "colcon build --symlink-install"
1. Run "gedit ~/.bashrc"
1. add at the bottom of the file: "source ~/{your project directory}/ros2_ws/install/setup.bash", OR if running on a non-admin user: "source home/{username}/{path to repo in your user directory}/ros2_ws/install/setup.bash"
    1. Important, if you are not the admin user, and a separate admin user set up ros2 on your system already, add the following to the .bashrc file as well:
    ```
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```

### Separately, if you are not using this repo and are just creating a new ros2 workspace in a fresh repo, here are some directions:
1. Again, this is not necessary if you are just forking the repo and working off of it. 
1. If you decide to create a new codebase in a completely fresh ros2 workspace, then you'll need to do this.
1. Set up your ros2 workspace following instructions in this [tutorial](https://www.youtube.com/watch?v=idQb2pB-h2Q), which I've outlined below:
    1. create an empty repo, clone it locally
    1. add to .gitignore file:
    ```
        /ros2_ws/build
        /ros2_ws/install
        /ros2_ws/log
        *.pyc
    ```
    1. in the repo root, run "mkdir ros2_ws"
    1. "cd ros2_ws"
    1. "mkdir src"
    1. "colcon build", verify that 3 folders are created in the ros2_ws directory: "install", "build", and "log"
    1. cd "src"
    1. "ros2 pkg create {name of your project without spaces} --build-type ament_python --dependencies rclpy"
    1. After your project is set up, run "gedit ~/.bashrc" and add "source ~/{your project directory}/ros2_ws/install/setup.bash", OR if running on a non-admin user: "source home/{username}/{path to repo in your user directory}/ros2_ws/install/setup.bash"
    1. you can now build the project from the ros2_ws directory by running "colcon build --symlink-install"
    1. finally, you can commit your changes to your repo.

### Highly recommended tutorials on Ros2!
1. There is an OUTSTANDING tutorial for Ros2 found [here](https://www.youtube.com/watch?v=idQb2pB-h2Q)
1. Additionally, you can set up syntax highlighting for VS Code with Ros2 [here](https://www.youtube.com/watch?v=hf76VY0a5Fk)
1. The [Ros2 Humble Docs](https://docs.ros.org/en/humble/index.html) are also excellent and provide both high level and detailed information.

### Setup of Create3 robot
1. Please refer to iRobot's extensive documentation found [here](https://iroboteducation.github.io/create3_docs/)
1. Also, the [iRobot Learning Library](https://edu.irobot.com/learning-library) contains hours of useful tutorials.
