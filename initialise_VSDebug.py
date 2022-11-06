#!/usr/bin/env python3

import json
from pathlib import Path
import os
import sys

# Retrieve the workspace folder
workspace_folder = str(Path().absolute())

# check if build with catkin_make or catkin build by finding the log folder
folder_names = [f for f in os.listdir(workspace_folder) if os.path.isdir(os.path.join(workspace_folder,f))]
catkin_make = True
if ".catkin_tools" in folder_names:
    catkin_make = False

########################################################################################
## Creation of the tasks.json file
########################################################################################
if catkin_make:
    tasks_data = {
        "version": "2.0.0",
        "tasks": [
            {
                "label": "ROS: catkin_make",
                "type": "catkin_make",
                "args": [
                    "--directory",
                    workspace_folder,
                    "-DCMAKE_BUILD_TYPE=Debug",
                    "-DCMAKE_EXPORT_COMPILE_COMMANDS=1"
                ],
                "problemMatcher": "$catkin-gcc",
                "group": {
                    "kind": "build",
                    "isDefault": True
                }
            },
            {
                "label": "ROS: catkin_make --->> Debug rosrun",
                "command": "python3 "+str(workspace_folder)+"/update_VSDebug.py",
                "type": "shell",
                "group": {
                    "kind": "build",
                    "isDefault": True
                },
                "presentation": {
                    "reveal": "always",
                    "panel": "new",
                    "focus": True
                },"dependsOn": [
                    "ROS: catkin_make"                               
                ]
            },
            {
                "label": "ROS: catkin_make --->>> Debug roslaunch",
                "command": "python3 "+str(workspace_folder)+"/update_VSDebug_launch.py",
                "type": "shell",
                "group": {
                    "kind": "build",
                    "isDefault": True
                },
                "presentation": {
                    "reveal": "always",
                    "panel": "new",
                    "focus": True
                },"dependsOn": [
                    "ROS: catkin_make"                               
                ]
            }
        ]
    }
else:
        tasks_data = {
        "version": "2.0.0",
        "tasks": [
            {
                "label": "ROS: catkin build",
                "type": "catkin",
                "args": [
                    "build",
                    "-DCMAKE_BUILD_TYPE=Debug",
                    "-DCMAKE_EXPORT_COMPILE_COMMANDS=1"
                ],
                "problemMatcher": "$catkin-gcc",
                "group": {
                    "kind": "build",
                    "isDefault": True
                }
            },
            {
                "label": "ROS: catkin clean",
                "type": "catkin",
                "args": [
                    "clean"
                ],
                "problemMatcher": "$catkin-gcc",
                "group": {
                    "kind": "build",
                    "isDefault": True
                }
            },
            {
                "label": "ROS: catkin build --->> Debug rosrun",
                "command": "python3 "+str(workspace_folder)+"/update_VSDebug.py",
                "type": "shell",
                "group": {
                    "kind": "build",
                    "isDefault": True
                },
                "presentation": {
                    "reveal": "always",
                    "panel": "old",
                    "focus": True
                },"dependsOn": [
                    "ROS: catkin build"                               
                ]
            },
            {
                "label": "ROS: catkin build -->> Debug roslaunch",
                "command": "python3 "+str(workspace_folder)+"/update_VSDebug_launch.py",
                "type": "shell",
                "group": {
                    "kind": "build",
                    "isDefault": True
                },
                "presentation": {
                    "reveal": "always",
                    "panel": "old",
                    "focus": True
                },"dependsOn": [
                    "ROS: catkin build"                               
                ]
            }
        ]
    }

# Store the data in the json file
with open('.vscode/tasks.json', 'w') as jsonFile_tasks:
    json.dump(tasks_data, jsonFile_tasks, indent=4)
    jsonFile_tasks.close()

########################################################################################
## Creation of the c_cpp_properties.json file
########################################################################################

# create the full properties file
# So intellisense can find all headers from ROS, installed packages and headers in the workspace
c_cpp_properties_data = {
        "configurations": [
            {
            "name": "Ubuntu",
            "includePath": [
                "/usr/include/**",
                "/opt/ros/noetic/include/**",
                "${workspaceFolder}/**"
            ],
            "intelliSenseMode": "gcc-x64",
            "compilerPath": "/usr/bin/g++",
            "cStandard": "c11",
            "cppStandard": "c++17"
            }
        ],
        "version": 4
    }

# Store the data in the json file
with open('.vscode/c_cpp_properties.json', 'w') as jsonFile_c_cpp_properties:
    json.dump(c_cpp_properties_data, jsonFile_c_cpp_properties, indent=4)
    jsonFile_c_cpp_properties.close()

########################################################################################
## Creation of the extensions.json file
########################################################################################
extensions_data = {
	"recommendations": [
		"eamodio.gitlens",
        "ms-iot.vscode-ros",
        "ms-python.python",
        "ms-vscode.cpptools",
        "twxs.cmake",
        "vscode-icons-team.vscode-icons"
	]
}
# Store the data in the json file
with open('.vscode/extensions.json', 'w') as jsonFile_extensions:
    json.dump(extensions_data, jsonFile_extensions, indent=4)
    jsonFile_extensions.close()

########################################################################################
## Creation of the settings.json file
########################################################################################
settings_data = {
    "python.autoComplete.extraPaths": [
        "/opt/ros/noetic/lib/python3/dist-packages"
    ],
    "cmake.sourceDirectory": "${workspaceFolder}/src",
    "cmake.configureOnOpen": False
}
# Store the data in the json file
with open('.vscode/settings.json', 'w') as jsonFile_settings:
    json.dump(settings_data, jsonFile_settings, indent=4)
    jsonFile_settings.close()
