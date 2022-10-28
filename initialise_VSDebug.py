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
                "label": "ROS: central_catkin_make",
                "type": "catkin_make",
                "args": [
                    "--directory",
                    workspace_folder,
                    "-j4",
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
                "label": "ROS: update Build & Debug",
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
                    "ROS: central_catkin_make"                               
                ]
            }
        ]
    }
else:
        tasks_data = {
        "version": "2.0.0",
        "tasks": [
            {
                "label": "ROS: central_catkin_build",
                "type": "catkin",
                "args": [
                    "build",
                    "-j4",
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
                "label": "ROS: update Build & Debug",
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
                    "ROS: central_catkin_build"                               
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
c_cpp_properties_temp = []
## check for catkin build if packages are already there
package_names = [f for f in os.listdir(workspace_folder+"/build") if os.path.isdir(os.path.join(workspace_folder+"/build",f))]
# Remove name catkin_tools_prebuild
if "catkin_tools_prebuild" in package_names:
    package_names.remove("catkin_tools_prebuild")

# check if catkin_make 
if catkin_make:
    c_cpp_properties_temp = {
                "name": "Linux",
                "intelliSenseMode": "gcc-x64",
                "compilerPath": "/usr/bin/g++",
                "cStandard": "c11",
                "cppStandard": "c++17",
                "compileCommands": "${workspaceFolder}/build/compile_commands.json"
            }
else:
    for j in package_names:
        c_cpp_properties_temp.append({
                    "name": "Linux_"+str(j),
                    "intelliSenseMode": "gcc-x64",
                    "compilerPath": "/usr/bin/g++",
                    "cStandard": "c11",
                    "cppStandard": "c++17",
                    "compileCommands": "${workspaceFolder}/build/"+str(j)+"/compile_commands.json"
        })

# create the full properties file
c_cpp_properties_data = {
        "configurations": [
        ],
        "version": 4
    }
c_cpp_properties_data["configurations"] = c_cpp_properties_temp

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
