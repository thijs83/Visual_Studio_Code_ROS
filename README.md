# Automation in VS Code with ROS

This repository contains two important python scripts that automate your ROS workflow in the Visual Studio Code IDE. Below is a step by step guide on how to setup the VS environment and work with this environment. 

Note: The assumption is made that you know how ROS works and that it is installed. Also Visual Studio Code should be installed.


## Contents:
* [1. Visual Studio Code Extensions](#1-visual-studio-code-extensions)
* [2. Setup of the VS workspace](#2-setup-of-the-vs-workspace)
* [3. Automation of the VS workspace](#3-automation-of-the-vs-workspace)
* [4. Debugging your code in VS with ROS](#4-debugging-your-code-in-vs-with-ros)
* [5. Hints to increase development speed](#5-hints-to-increase-development-speed)

## 1) Visual Studio Code Extensions

The following extensions are recommended:
- C/C++ (C/C++ IntelliSense, debugging, and code browsing.)
- Python (IntelliSense (Pylance), Linting, Debugging (multi-threaded, remote), Jupyter Notebooks, code formatting, refactoring, unit tests, and more.)
- ROS (Develop Robot Operating System (ROS) with Visual Studio Code.)
- CMake (CMake langage support for Visual Studio Code)
- CMake Tools (Extended CMake support in Visual Studio Code)

## 2) Setup of the VS Workspace

First, a ROS workspace needs to be created, which eventually also will be the VS code workspace.
```bash
 mkdir ~/catkin_ws
 cd ~/catkin_ws
 mkdir src
 mkdir .vscode
```
Now we need to initilize the catkin workspace,
```
catkin build
```
Note: You could also use 'catkin_make' with this repository but it is recommended to use 'catkin build' since this repository only provides a basic setup for 'catkin_make' and doesn't have all the features described below.

Note: Using catkin build or catkin_make is essential to set up the ROS workspace for VS to know that it is a ROS workspace

Now we need the two files and too showcase the example, there are two beginner ROS packages included. These need to be moved to the src folder. If you have other packages then remove the ones included in this repository.
```bash
 git clone https://github.com/thijs83/Visual_Studio_Code_ROS.git
 mv -v Visual_Studio_Code_ROS/beginner_tutorials/ src
 mv -v Visual_Studio_Code_ROS/hello_vs_code/ src
 mv -v Visual_Studio_Code_ROS/.vstools .
 sudo rm -r Visual_Studio_Code_ROS
```

Now we run the first python file to setup all the .json files in the .vscode folder. These are used by VS code to setup the environment and determine the debug settings. This script only has to run one time (the first time).
```bash
 python3 .vstools/initialise_VSDebug.py
```

Next, we need to open Visual Studio Code and open the folder catkin_ws. Now go to File -> Save Workspace As and press save. The final structure of the ROS and VS workspace will look like:
```
~/catkin_ws/
    .catkin_tools
    .vscode/
        c_cpp_properties.json
        extensions.json
        settings.json
        tasks.json
    .vstools/
        initialise_VSDebug.py
        update_VSDebug_launch_pkg.py
        update_VSDebug_launch.py
        update_VSDebug.py
    build/
    devel/
    logs/
    src/
        beginner_tutorials/
        hello_vs_code/
```

Below an image of how it will look like in Visual studio code.

![Alt text](images/image_setup.png?raw=true "Initial setup")



Note: Make sure that the ROS environment is sourced in .bashrc and that ROS1 is mentioned with its distribution in the bar located at the bottom of VS code.

## 3) Automation of the VS workspace

Now everything is ready we can start automating using the second python script. To make it easy and not having to run the python script everytime, a task is made that can be run from the Command Palette. This task automatically runs catkin_make (with debug settings to ON) and then runs the python script to include all ros exutables to the debug section.
```
To do this: press Ctrl+Shift+P -->> Tasks: Run Task -->> ROS: update Build & Debug
```

![Alt-text-1](images/tasks.png?raw=true "Tasks drop down menu") ![Alt-text-2](images/ros_build.png?raw=true "ROS build drop down menu")


Now in the folder .vscode another file will be included named launch.json automatically. Everytime you alter files or delete/add executables, you have to run the above command. (This command runs catkin_make automatically and then updates the launch.json, so all executables are up to date with your code)

We can now go to the Run and Debug section (Ctrl+Shift+D) and all the ros nodes are added to the drop-down menu visualized in the image below. 

![Alt-text-1](images/dropdown_menu.png?raw=true "Debug drop down menu")

The nodes are first described by if it is made as a python or c++ script, followed by the node name and then the package it originates from.
Lets start two nodes that talk to eachother, one publishes a counter and the other subscribes to the same counter. 

Roscore has to be started for connecting the nodes. This is done by following the commands:
```
Press Ctrl+Shift+P -->> ROS: Start
```
Now start c++: cppsub_Node - PKG:beginner_tutorials by selecting this node from the drop down menu and then press the green play button to the left. This starts the subscriber node. Now do the same for the c++: cpptalker_Node - PKG:beginner_tutorials. Now the two nodes are running and talking to eachother. The terminal of the talker is visualized but you can switch to the subscriber node terminal using the bottom-left pannel (Marked with the green circle) and press on the wanted terminal as shown below.


![Alt-text-1](images/two_nodes.png?raw=true "Two nodes running")
RED: The Debug player  
GREEN: The debug terminals, one for each node  
ORANGE: Checkmark when Roscore is running or cross if not  

To stop the nodes, the terminals can be closed or the stop button can be pressed for the node. This is visualized in the image below.

Now you know how to start the nodes and stop them. The next section shows how to debug the code

## 4) Debugging your code in VS with ROS

The next step to debugging is very easy. Let's take the two nodes from previous section and open the scripts next to eachother and in both scripts set a Breakpoint as visualized in the image below.


Now run both scripts as done in previous section and see how the code stops at the break point set in the publisher (left figure below). The debug player should be put on the talker node, as shown in the figure. Now press a few times F5 or the continue button in the debug player to see what happens. After a few times pressing, a yellow bar should appear in the subscriber file and the breakpoint is hit (see the right figure below). Now in the dropdown menu in the debug player, select the subscriber node and the corresponding variables are loaded. Again press F5 to let the code continue. 

![Alt-text-1](images/talk_breakpoint.png?raw=true "Breakpoint hit in talker") ![Alt-text-2](images/sub_breakpoint.png?raw=true "Breakpoint hit in subscriber")


   

| Important! |
| --- |
| - You will see that the subscriber doesnt receive the first or first few messages. This is due to the setup of the architecture of ROS. The publisher is already publishing messages when the connections are not yet initialized, and thus are lost. To have zero loss, set a ros rate sleeper (of around 5 seconds) after setting up the publisher note and always first start the subscriber nodes. This will make sure that all messages are received by the subscribers. |
| - Another thing, you will notice with the tutorial nodes that sometimes the breakpoint of the publisher is hit twice before the breakpoint in the subscriber is hit. This is due to the speed until the next breakpoint, the publisher was faster in hitting the next breakpoint than the subscriber was in receiving the message and hitting it's breakpoint. |

## 5) Hints to increase development speed

### Keybindings
We can assign a keybinding to start ROS: update Build & Debug by doing:
```
Press Ctrl+Shift+P -->> Preferences: Open Keyboard Shortcuts (JSON)
```
The keybindings.json file will open. The keybindings in here overwrite the default keybinding so make sure that the keybinding is not something you reguraly use. Add the following to the file,
```
[
    { 
        "key": "ctrl+F5",         
        "command": "workbench.action.tasks.runTask",
        "args": "ROS: update Build & Debug"
    }
]
```

And it should look like the image below. Save the file to make the keybinding work. Now you only have to use the keybinding to build/update the ros workspace and add new nodes to the launch file for VS code debugger

![Alt-text-1](images/keybinding.png?raw=true "Keybinding for build and debug")


### Multiple windows

Instead of using one VS code window, one could use multiple VS code windows to debug more easily. This is espacially usefull with a multiple monitor setup. This needs to be done in a specific way, we need to clone the workspace. To do this follow,
```
Go to File --> Duplicate Workspace
```
Now you will have to VS code windows, both with the same VS code workspace. Now you can do the same test as presented in the previous chapter, only now you can run the publisher in one window and the subscriber in the other window. Below an image of how it looks like.

![Alt-text-1](images/two_screens.png?raw=true "Use of two VS code windows")


