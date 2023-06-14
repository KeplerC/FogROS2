import sys
import json
import os 

def handler(event, context):
    
    return {
        "statusCode": 200,
        "headers": {
            "Content-Type": "application/json"
        },
        "body": json.dumps({
            "Region ": "hello world"
        })
    }