import sys
import json
import os 
import subprocess

def handler(event, context):
    subprocess.call(". /opt/ros/humble/setup.sh && . /fog_ws/install/setup.sh && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && cd / && /gdp-router router&", shell=True)
    subprocess.call(". /opt/ros/humble/setup.sh && . /fog_ws/install/setup.sh && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 run fogros2_examples talker", shell=True)

    return {
        "statusCode": 200,
        "headers": {
            "Content-Type": "application/json"
        },
        "body": json.dumps({
            "Region ": "hello world"
        })
    }