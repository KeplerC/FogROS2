
import abc
import json
import os
import subprocess
import random

from .cloud_instance import CloudInstance
from .name_generator import get_unique_name
from .util import (
    MissingEnvironmentVariableException,
    instance_dir,
    make_zip_file,
)

docker_file_template = '''
# Define function directory
ARG FUNCTION_DIR="/function"
ARG BASE_IMAGE="osrf/ros:humble-desktop"
FROM ${BASE_IMAGE} as build-image

# Install aws-lambda-cpp build dependencies
RUN apt-get update &&   apt-get install -y   g++   make   cmake   unzip   python3-pip   libcurl4-openssl-dev wget

# Include global arg in this stage of the build
ARG FUNCTION_DIR
# Create function directory
RUN mkdir -p ${FUNCTION_DIR}

# Install the runtime interface client
RUN /usr/bin/python3 -m pip install  --target ${FUNCTION_DIR}   awslambdaric

RUN wget https://github.com/aws/aws-lambda-runtime-interface-emulator/releases/latest/download/aws-lambda-rie -O ${FUNCTION_DIR}/aws-lambda-rie

# Copy function code
COPY ./src/FogROS2-lambda/fogros2/app/* ${FUNCTION_DIR}/

WORKDIR /fog_ws
COPY . .
RUN colcon build

# Multi-stage build: grab a fresh copy of the base image
FROM ${BASE_IMAGE}

# Include global arg in this stage of the build
ARG FUNCTION_DIR
# Set working directory to function root directory
WORKDIR ${FUNCTION_DIR}

RUN apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp python3-pip

RUN pip3 install boto3 paramiko scp wgconfig jsonpickle

# Copy in the build image dependencies
COPY --from=build-image ${FUNCTION_DIR} ${FUNCTION_DIR}
RUN chmod +x ./entry_script.sh && mv ./entry_script.sh /entry_script.sh
RUN chmod +x ./aws-lambda-rie && mv aws-lambda-rie /usr/local/bin/aws-lambda-rie

COPY --from=build-image /fog_ws/install /fog_ws/install
COPY --from=keplerc/fogros2-sgc:v0.2debug /src /src 
COPY --from=keplerc/fogros2-sgc:v0.2debug /scripts /scripts
COPY --from=keplerc/fogros2-sgc:v0.2debug /gdp-router /gdp-router

WORKDIR ${FUNCTION_DIR}
ENV HOME /tmp
# ENTRYPOINT [ "/usr/bin/python3", "-m", "awslambdaric" ]
ENTRYPOINT [ "/entry_script.sh" ]
CMD [ "app.handler" ]
'''
class AWSLambdas(CloudInstance):
    """AWS Implementation of CloudInstance."""
    def __init__(self, 
        ros_workspace=os.path.dirname(os.getenv("COLCON_PREFIX_PATH")),
        working_dir_base=instance_dir(),
        aws_user_id = "736982044827",
        aws_repo_id = "fogros_lambda"
    ):
        self.ros_workspace = ros_workspace
        self._name = get_unique_name()
        self.aws_user_id = aws_user_id
        self._working_dir_base = working_dir_base
        self._working_dir = os.path.join(self._working_dir_base, self._name)
        os.makedirs(self._working_dir, exist_ok=True)

        self.version = str(random.randint(0, 999))

        self.docker_repo_uri = f"{self.aws_user_id}.dkr.ecr.us-west-1.amazonaws.com/{aws_repo_id}"
        self.lambda_name = f"fogros-lambda-{self.version}"
        self.image_name = f"{self.docker_repo_uri}:{self.version}"
        self.response_file_name = f"/tmp/response-{self.lambda_name}.json"
        self.init()

    def init(self):
        subprocess.call(f"aws ecr get-login-password --region us-west-1 | docker login --username AWS --password-stdin {self.aws_user_id}.dkr.ecr.us-west-1.amazonaws.com", shell = True)

    def create(self):
        self.create_docker_file()
        subprocess.call(f"cd {self.ros_workspace} && docker build -t fogros-lambda-image . ", shell=True)
        subprocess.call(f"docker tag fogros-lambda-image:latest {self.image_name}", shell=True)
        subprocess.call(f"docker push {self.image_name}", shell=True)
        subprocess.call(f"aws lambda create-function --function-name {self.lambda_name}  --package-type Image   --code ImageUri={self.image_name}  --output text --role arn:aws:iam::736982044827:role/RoleLambda", shell=True)
        from time import sleep
        sleep(60)
        subprocess.call(f"aws lambda update-function-configuration --function-name {self.lambda_name} --timeout 60 --memory-size 10240 --output text", shell=True)
        sleep(10)

    def invoke(self, request_file_path):
        import json

        invoke_command = f"aws lambda invoke --payload fileb://{request_file_path} --function-name {self.lambda_name} {self.response_file_name}"
        subprocess.call(invoke_command, shell=True)

        



    def create_docker_file(self): 
        with open(self.ros_workspace + "/Dockerfile", "w+") as f:
            f.write(docker_file_template)
