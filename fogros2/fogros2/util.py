# Copyright 2022 The Regents of the University of California (Regents)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Copyright ©2022. The Regents of the University of California (Regents).
# All Rights Reserved. Permission to use, copy, modify, and distribute this
# software and its documentation for educational, research, and not-for-profit
# purposes, without fee and without a signed licensing agreement, is hereby
# granted, provided that the above copyright notice, this paragraph and the
# following two paragraphs appear in all copies, modifications, and
# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
# otl@berkeley.edu, http://ipira.berkeley.edu/industry-info for commercial
# licensing opportunities. IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE. REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY,
# PROVIDED HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

import errno
import os
import tarfile
from pydoc import locate

_work_dir_cache = None
_instance_dir_cache = None


class MissingEnvironmentVariableException(Exception):
    pass


def _mkdir(path, mode=0o700):
    try:
        os.mkdir(path, mode=mode)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise


def work_dir():
    global _work_dir_cache
    if _work_dir_cache is None:
        home = os.path.expanduser("~")
        path = os.path.join(home, ".fogros2")
        _mkdir(path)
        _work_dir_cache = path
    return _work_dir_cache


def instance_dir():
    global _instance_dir_cache
    if _instance_dir_cache is None:
        path = os.path.join(work_dir(), "instances")
        _mkdir(path)
        _instance_dir_cache = path
    return _instance_dir_cache


# Using Tar not Zip
def make_zip_file(dir_name, target_path):
    root_dir, workspace_name = os.path.split(dir_name)
    print(root_dir, workspace_name)
    base_name = os.path.abspath(target_path)
    os.chdir(root_dir)

    tar_compression = ''
    archive_name = base_name + '.tar' + ''
    archive_dir = os.path.dirname(archive_name)

    # https://stackoverflow.com/questions/16000794/python-tarfile-and-excludes
    EXCLUDE_FILES = ['.git']

    if archive_dir and not os.path.exists(archive_dir):
        os.makedirs(archive_dir)
    tar = tarfile.open(archive_name, 'w|%s' % tar_compression)
    try:
        tar.add(workspace_name, filter=lambda x: None if x.name in EXCLUDE_FILES else x)
    finally:
        tar.close()
    return archive_name


def extract_bash_column(subprocess_output: str, column_name: str, row_number: int = 0):
    """Find the value of any given column value - ex: CLUSTER-IP -> 10.0.015.

    :param subprocess_output: Direct output of subprocess.check_output().decode()
    :param column_name: The column name to search for ex: CLUSTER-IP
    :param row_number: Defaults to the first data row, row_number = 1 is second data row
    :return: String of output value
    """
    lines = subprocess_output.split('\n')
    if column_name not in lines[0]:
        raise LookupError(f"Could not find column {column_name} in {lines[0].strip()}")
    column_index = lines[0].index(column_name)

    output_str = ''
    while column_index != len(lines[row_number+1]) and lines[row_number+1][column_index] != ' ':
        output_str += lines[row_number+1][column_index]
        column_index += 1

    return output_str

def get_ROS_class(ros_message_type, srv=True):
    """
    Returns the ROS message class from ros_message_type.
    :return AnyMsgClass: Class of the ROS message.
    """
    try:
        package_name, message_name = ros_message_type.split('/')
    except ValueError:
        raise ValueError(
            'ros_message_type should be in the shape of package_msgs/Message' +
            ' (it was ' + ros_message_type + ')')
    if not srv:
        msg_class = locate('{}.msg.{}'.format(package_name, message_name))
    else:
        msg_class = locate('{}.srv.{}'.format(package_name, message_name))
    if msg_class is None:
        if srv:
            msg_or_srv = '.srv'
        else:
            msg_or_srv = '.msg'
        raise ValueError(
            'ros_message_type could not be imported. (' +
            ros_message_type + ', as "from ' + package_name +
            msg_or_srv + ' import ' + message_name + '" failed.')
    return msg_class
