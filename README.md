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
```
$ mkdir ~/catkin_ws
$ cd ~/catkin_ws
$ mkdir src
$ mkdir ./vscode
$ catkin_make
```

Note: Using catkin_make is essential to set up the ROS workspace, which is used by VS to know the workspace





## 3) Automation of the VS workspace