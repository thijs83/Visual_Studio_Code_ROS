#!/usr/bin/env python3

import json
from pathlib import Path
import os
import sys
import glob

# Retrieve the workspace folder
workspace_folder = str(Path().absolute())

# Find all packages and check if there exists atleast one
package_names = [f for f in os.listdir(workspace_folder+"/src") if os.path.isdir(os.path.join(workspace_folder+"/src",f))]

if not package_names:
    print("There are no packages and thus nothing is build")
    sys.exit(1)

# Check if initial build is done
try:
    test = os.listdir(workspace_folder+"/devel")

    if not test:
        print("First build the envirnment, before calling this script")
        sys.exit(1)
except:
    print("First build the envirnment, before calling this script")
    sys.exit(1)

########################################################################################
## Creation of the launch.json file
########################################################################################
launch_data_array = []
for i in package_names:
    # Find all executables inside package
    executable_names = [f for f in os.listdir(workspace_folder+"/devel/lib/"+str(i)) if os.path.isfile(os.path.join(workspace_folder+"/devel/lib/"+str(i),f))]

    # Remove name cmake.lock
    if "cmake.lock" in executable_names:
        executable_names.remove("cmake.lock")
    
    # CPP and Python file name in their own array
    python_executable_names = [name for name in executable_names if name.endswith(".py")]
    cpp_executable_names = [name for name in executable_names if not name.endswith(".py")]

    # Add the CPP files to the launch file
    for j in cpp_executable_names:     
        launch_data_array.append({
                        "name": "c++: "+str(j)+"_Node - PKG:"+str(i),
                        "type": "cppdbg",
                        "request": "launch",
                        "program": "${workspaceFolder}/devel/lib/"+str(i)+"/"+str(j),
                        "args": [],
                        "stopAtEntry": False,
                        "cwd": "${workspaceFolder}/../../",
                        "environment": [],
                        "externalConsole": False,
                        "MIMode": "gdb",
                        "setupCommands": [
                            {
                                "description": "Enable pretty-printing for gdb",
                                "text": "-enable-pretty-printing",
                                "ignoreFailures": True
                            }
                        ]
                    })

    # Find the python files and also add them to the launch file
    for j in python_executable_names:
        path_package = "./src/"+str(i)
        path_file = glob.glob(path_package + "/**/"+str(j), recursive = True)
        
        path = path_file[0]

        launch_data_array.append({
                        "name": "py: "+str(j)+"_Node - PKG:"+str(i),
                        "type": "python",
                        "request": "launch",
                        "program": "${workspaceFolder}"+path[1:]
        })

# Give the correct form to the launch file     
launch_data = {}
launch_data["configurations"] = launch_data_array

# Store the data in the json file
with open('.vscode/launch.json', 'w') as jsonFile_launch:
    json.dump(launch_data, jsonFile_launch, indent=4)
    jsonFile_launch.close()

# If new package is added we need to also make new cpp properties file so we just make new one
c_cpp_properties_temp = []
## check for catkin build if packages are already there
package_names = [f for f in os.listdir(workspace_folder+"/build") if os.path.isdir(os.path.join(workspace_folder+"/build",f))]
# Remove name catkin_tools_prebuild
if "catkin_tools_prebuild" in package_names:
    package_names.remove("catkin_tools_prebuild")

# check if build with catkin_make or catkin build by finding the log folder
folder_names = [f for f in os.listdir(workspace_folder) if os.path.isdir(os.path.join(workspace_folder,f))]
catkin_make = True
if ".catkin_tools" in folder_names:
    catkin_make = False

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
