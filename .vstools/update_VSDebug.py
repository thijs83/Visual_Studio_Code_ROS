#!/usr/bin/env python3

import json
from pathlib import Path
import os
import sys
import glob

# Retrieve the workspace folder
workspace_folder = str(Path().absolute())

# Find all packages and check if there exists atleast one
try:
    package_names = [
        f
        for f in os.listdir(workspace_folder + "/devel/lib")
        if os.path.isdir(os.path.join(workspace_folder + "/devel/lib", f))
    ]
    if "pkgconfig" in package_names:
        package_names.remove("pkgconfig")
    if "python3" in package_names:
        package_names.remove("python3")
except Exception:
    print(
        "First build the envirnment or include packages, \
          before calling this script"
    )
    sys.exit(1)


###################################################################
# Creation of the launch.json file
###################################################################
launch_data_array = []
for i in package_names:
    # Find all executables inside package
    executable_names = [
        f
        for f in os.listdir(workspace_folder + "/devel/lib/" + str(i))
        if os.path.isfile(os.path.join(workspace_folder + "/devel/lib/" + str(i), f))
    ]

    # Remove name cmake.lock
    if "cmake.lock" in executable_names:
        executable_names.remove("cmake.lock")

    # CPP and Python file name in their own array
    python_executable_names = [
        name for name in executable_names if name.endswith(".py")
    ]
    cpp_executable_names = [
        name for name in executable_names if not name.endswith(".py")
    ]

    # Add the CPP files to the launch file
    for j in cpp_executable_names:
        launch_data_array.append(
            {
                "name": "c++: " + str(j) + "_Node - PKG:" + str(i),
                "type": "cppdbg",
                "request": "launch",
                "program": "${workspaceFolder}/devel/lib/" + str(i) + "/" + str(j),
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
                        "ignoreFailures": True,
                    }
                ],
            }
        )

    # Find the python files and also add them to the launch file
    for j in python_executable_names:
        path_package = "./src/" + str(i)
        path_file = glob.glob(path_package + "/**/" + str(j), recursive=True)

        path = path_file[0]

        launch_data_array.append(
            {
                "name": "py: " + str(j) + "_Node - PKG:" + str(i),
                "type": "python",
                "request": "launch",
                "program": "${workspaceFolder}" + path[1:],
            }
        )

# Give the correct form to the launch file
launch_data = {}
launch_data["configurations"] = launch_data_array

# Store the data in the json file
with open(".vscode/launch.json", "w") as jsonFile_launch:
    json.dump(launch_data, jsonFile_launch, indent=4)
    jsonFile_launch.close()
