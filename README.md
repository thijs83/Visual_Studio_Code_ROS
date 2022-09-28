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