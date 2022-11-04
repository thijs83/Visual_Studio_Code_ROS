#!/usr/bin/env python3

import json
from pathlib import Path
import os
import sys
import glob

# Retrieve the workspace folder
workspace_folder = str(Path().absolute())

# Array to store configurations
launch_data_array = []
for path, subdirs, files in os.walk(workspace_folder+"/src"):
    for name in files:
        if name.endswith(".launch"):
            # check package name to name launch configuration
            folder_names = path.split("/")
            package = folder_names[-2]
            launch_data_array.append({
                "name": "ROS: "+name+" ----Pkg:"+package+"",
                "type": "ros",
                "request": "launch",
                "target": ""+os.path.join(path, name)+""
            })

# Give the correct form to the launch file     
launch_data = {}
launch_data["configurations"] = launch_data_array

# Store the data in the json file
with open('.vscode/launch.json', 'w') as jsonFile_launch:
    json.dump(launch_data, jsonFile_launch, indent=4)
    jsonFile_launch.close()
