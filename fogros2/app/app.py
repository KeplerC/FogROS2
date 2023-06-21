import sys
import json
import os 
import subprocess

def handler(event, context):
    # for now, not needed for SGC
    serialized_param = event["param"]
    # subprocess.call(". /opt/ros/humble/setup.sh && . /fog_ws/install/setup.sh && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && cd / && /gdp-router router&", shell=True)
    subprocess.call(". /opt/ros/humble/setup.sh && . /fog_ws/install/setup.sh && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 run fogros2_examples service&", shell=True)
    subprocess.call(f". /opt/ros/humble/setup.sh && . /fog_ws/install/setup.sh && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 run fogros2 client {serialized_param}", shell=True)

    return {
        "statusCode": 200,
        "headers": {
            "Content-Type": "application/json"
        },
        "body": json.dumps({
            "Region ": "hello world"
        })
    }