import sys
import json
import os 
import subprocess

def handler(event, context):
    subprocess.call(". /opt/ros/humble/setup.sh && . /fog_ws/install/setup.sh && cd / && /gdp-router router&", shell=True)
    subprocess.call(". /opt/ros/humble/setup.sh && . /fog_ws/install/setup.sh && ros2 run demo_nodes_cpp talker", shell=True)

    return {
        "statusCode": 200,
        "headers": {
            "Content-Type": "application/json"
        },
        "body": json.dumps({
            "Region ": "hello world"
        })
    }