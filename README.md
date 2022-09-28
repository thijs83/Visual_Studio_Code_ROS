# Automation in VS Code with ROS

This repository contains two important python scripts that automate your ROS workflow in the Visual Studio Code IDE. Below is a step by step guide on how to setup the VS environment and work with this environment. 

Note: The assumption is made that you know how ROS works and that it is installed. Also Visual Studio Code should be installed.


## Contents:
* [1. Visual Studio Code Extensions](#1-visual-studio-code-extensions)
* [2. Setup of the VS workspace](#2-setup-of-the-vs-workspace)
* [3. Automation of the VS workspace](#3-automation-of-the-vs-workspace)

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
 mkdir ./vscode
 catkin_make
```

Note: Using catkin_make is essential to set up the ROS workspace for VS to know that it is a ROS workspace

Now we need the two files and too showcase the example, there are two beginner ROS packages included. These need to be moved to the src folder. If you have other packages then remove the ones included in this repository.
```bash
 git clone https://github.com/thijs83/Visual_Studio_Code_ROS.git
 mv -v ~/catkin_ws/Visual_Studio_Code_ROS/beginner_tutorials/ ~/catkin_ws/src
 mv -v ~/catkin_ws/Visual_Studio_Code_ROS/hello_vs_code/ ~/catkin_ws/src
```

Now we run the first python file to setup all the .json files in the .vscode folder. These are used by VS code to setup the environment and determine the debug settings. This script only has to run one time.
```bash
 python3 initialise_VSDebug.py
```

Next, we need to open Visual Studio Code and open the folder catkin_ws. Now go to File -> Save Workspace As and press save. The final structure of the ROS and VS workspace will look like:
```
~/catkin_ws/
    .vscode/
        c_cpp_properties.json
        extensions.json
        settings.json
        tasks.json
    build/
    devel/
    src/
        beginner_tutorials/
        hello_vs_code/
    .catkin_workspace
    initialise_VSDebug.py
    update_VSDebug.py
    catkin_ws.code-workspace
```

Below an image of how it will look like in Visual studio code.

TO DO: image

## 3) Automation of the VS workspace

Now everything is ready we can start automating using the second python script. To make it easy and not having to run the python script everytime, a task is made that can be run from the Command Palette. This task automatically runs catkin_make (with debug settings to ON) and then runs the python script to include all ros exutables to the debug section.
```
To do this: press Ctrl+Shift+P -->> Tasks: Run Task -->> ROS: update ROS & Debug
```

TO DO: images next to eachother


Now in the folder .vscode another file will be included named launch.json automatically. Everytime you alter files or delete/add executables, you have to run the above command. (This command runs catkin_make automatically and then updates the launch.json, so all executables are up to date with your code)

We can now go to the Run and Debug section (Ctrl+Shift+D) and all the ros nodes are added to the drop-down menu visualized in the image below. 

TO DO: Image

The nodes are first described by if it is made as a python or c++ script, followed by the node name and then the package it originates from.
Lets start two nodes that talk to eachother, one publishes a counter and the other subscribes to the same counter. 

Roscore has to be started for connecting the nodes. This is done by following the commands:
```
Press Ctrl+Shift+P -->> ROS: Start
```
Now start c++: cppsub_Node - PKG:beginner_tutorials by selecting this node from the drop down menu and then press the green play button to the left. This starts the subscriber node. Now do the same for the c++: cpptalker_Node - PKG:beginner_tutorials. 





## 4) Increase speed

TO DO: add shortcut to ROS: update ROS & Debug

