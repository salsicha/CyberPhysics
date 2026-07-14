#!/usr/bin/env python3

import argparse
import os
import json
import re
import shlex
import shutil
import signal
import subprocess
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

WHOLE = 'whole'
SKIM = 'skim'
BAGFILE_NAME = '{type}_{datetime}_bag'
MAX_BAG_SIZE = 30000000000  # 30 GB
MAX_BAG_DURATION = 300  # 5 min
RECORDING = 'recording_{num:02d}'
ENVLIST = '/env/env.list'
RUNLOG = 'run_log.json'


class RosbagRecord(Node):
    def __init__(self, proj_name, server_w_ip, server_eth_ip):
        super().__init__('python_backend')
        self.pilot_notes = None
        self.flight_number = None
        self.flight_id = None
        self.root_folder = None
        self.save_location = None

        self.recording_count = 0
        self.proj_name = proj_name
        self.server_w_ip = server_w_ip
        self.server_eth_ip = server_eth_ip

        self.get_logger().info(self.get_name() + ' start')
        self.create_service(Trigger, '/get_backend_status', self.backend_status_response)
        self.create_service(Trigger, '/start_recording', self.start_recording)
        self.create_service(Trigger, '/stop_recording', self.stop_recording)
        self.create_service(Trigger, '/w_ip', self.w_ip)
        self.create_service(Trigger, '/eth_ip', self.eth_ip)

        self.create_subscription(String, '/pilot_notes', self.pilot_notes_callback, 10)
        self.create_subscription(String, '/flight_number', self.flight_number_callback, 10)
        self.create_service(Trigger, '/shutdown', self.call_shutdown_service)

        self.disk_space_pub = self.create_publisher(String, '/disk_usage', 10)

        self.timer = self.create_timer(1, self.publish_disk_space)

    def publish_disk_space(self):
        path = self.root_folder or "/data"
        try:
            stat = shutil.disk_usage(path)
        except OSError as err:
            self.get_logger().warning(f"Could not read disk usage for {path}: {err}",
                                      throttle_duration_sec=60)
            return
        disk_usage_data = str(round((stat.used/stat.total)*100, 1))
        disk_avail_data = str(round(stat.free/1000000000, 1)) + "G"
        msg = disk_usage_data + "," + disk_avail_data

        connections = self.disk_space_pub.get_subscription_count()
        if connections > 0:  # ensuring there is a connection before publishing
            disk_space = String()
            disk_space.data = msg
            self.disk_space_pub.publish(disk_space)

    def w_ip(self, request, response):
        response.message = self.server_w_ip
        response.success = True

        return response

    def eth_ip(self, request, response):
        response.message = self.server_eth_ip
        response.success = True

        return response

    def call_shutdown_service(self, request, response):
        # Save data before shutdown
        self.stop_recording(request, response)

        # Linux shutdown command
        os.system("shutdown -h now")

        return response

    def backend_status_response(self, request, response):
        record_check = "false"

        list_cmd = subprocess.Popen("source /entrypoint.sh && ros2 node list", shell=True, executable='/bin/bash', stdout=subprocess.PIPE, universal_newlines=True)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()

        if retcode == 0:
            for str in list_output.split("\n"):
                if str == "/rosbag2_recorder":
                    record_check = "true"

        # Backend response
        response.success = True
        response.message = f"recording:{record_check},flight_number:{self.flight_number or ''},pilot_notes:{self.pilot_notes or ''}"

        return response

    def start_recording(self, request, response):
        if any(p is not None and p.poll() is None
               for p in (getattr(self, 'p1', None), getattr(self, 'p2', None))):
            response.success = False
            response.message = "Recording already in progress, stop it first."
            return response

        # House keeping. Clean up any zombies from the shared memory.
        os.system("fastdds shm clean")

        self.get_logger().info("Starting the recording")
        date_time = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")

        self.flight_id = f'{self.proj_name}_{self.flight_number}'
        self.root_folder = os.path.join('/data', self.flight_id)
        self.save_location = os.path.join(self.root_folder,
                                          RECORDING.format(num=self.recording_count))

        try:
            os.makedirs(self.save_location, exist_ok=True)
        except OSError as err:
            response.success = False
            response.message = str(err)

            return response

        subprocess.Popen(["chmod", "-R", "ugo+rwx", self.root_folder])
        self.get_logger().info("Save_location: " + self.save_location)

        if self.recording_count == 0:
            self.save_config_files()
        self.recording_count += 1

        # Compose files set TOPICS_EXCLUDED="-x '...'" so the value arrives
        # wrapped in literal double quotes; strip that one pair, then let
        # shlex tokenize and re-quote each token shell-safely.
        topics_excluded = os.environ.get("TOPICS_EXCLUDED", "")
        if len(topics_excluded) >= 2 and topics_excluded[0] == '"' and topics_excluded[-1] == '"':
            topics_excluded = topics_excluded[1:-1]
        topics_excluded = ' '.join(shlex.quote(token) for token in shlex.split(topics_excluded))

        whole_bagfile = os.path.join(self.save_location, BAGFILE_NAME.format(type=WHOLE, datetime=date_time))
        command1 = f"source /entrypoint.sh && ros2 bag record --include-hidden-topics -a " \
                   f"-o {shlex.quote(whole_bagfile)} -b {MAX_BAG_SIZE} -d {MAX_BAG_DURATION} {topics_excluded}"
        self.get_logger().info(f"command1: {command1}")

        skim_bagfile = os.path.join(self.save_location, BAGFILE_NAME.format(type=SKIM, datetime=date_time))
        command2 = f"source /entrypoint.sh && ros2 bag record " \
                   f"-a -o {shlex.quote(skim_bagfile)} -b {MAX_BAG_SIZE} -d {MAX_BAG_DURATION} -x '(.*)(image|camera)'"
        self.get_logger().info(f"command2: {command2}")

        self.p1 = subprocess.Popen(command1, stdin=subprocess.PIPE, shell=True, executable='/bin/bash', start_new_session=True) # rosbag containing all topics
        self.p2 = subprocess.Popen(command2, stdin=subprocess.PIPE, shell=True, executable='/bin/bash', start_new_session=True) # rosbag containing only Nav topics

        response.success = True
        response.message = whole_bagfile

        return response

    def stop_process_group(self, p):
        if p is None or p.poll() is not None:
            return
        try:
            pgid = os.getpgid(p.pid)
        except ProcessLookupError:
            return
        # SIGINT the whole group so ros2 bag record finalizes the bag cleanly.
        for sig, timeout in ((signal.SIGINT, 15), (signal.SIGTERM, 5)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            try:
                p.wait(timeout=timeout)
                return
            except subprocess.TimeoutExpired:
                continue
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            return
        p.wait()

    def stop_recording(self, request, response):
        self.get_logger().info("Stopping recording...")
        p1 = getattr(self, 'p1', None)
        p2 = getattr(self, 'p2', None)
        if any(p is not None and p.poll() is None for p in (p1, p2)):
            self.stop_process_group(p1)
            self.stop_process_group(p2)

            self.save_pilot_notes_to_file()

            response.message = "Recording stopped!"
            response.success = True
        else:
            response.message = "No active recording to stop."
            response.success = False

        # flush files to disk
        os.system("sync")

        return response

    def save_pilot_notes_to_file(self):
        # Check if the received pilot notes are available
        if self.pilot_notes:
            self.get_logger().info("Saving pilot notes")
            file_name = 'pilot_notes.txt'
            file_path = os.path.join(self.save_location, file_name)

            with open(file_path, 'w') as file:
                # Write the received pilot notes to txt file
                file.write(self.pilot_notes)

            self.get_logger().info(f'Pilot notes saved to file: {file_path}')

    def pilot_notes_callback(self, data):
        self.pilot_notes = data.data

    def flight_number_callback(self, data):
        flight_number = re.sub(r'[^A-Za-z0-9_-]', '', data.data)
        if not flight_number:
            self.get_logger().warning(f"Rejected invalid flight number: {data.data!r}")
            return
        if flight_number != data.data:
            self.get_logger().warning(f"Sanitized flight number {data.data!r} to {flight_number!r}")
        self.flight_number = flight_number
        self.get_logger().info(f"flight_number_callback: {data}")

    def save_config_files(self):
        self.save_env_list()
        self.save_run_logs()

    def save_run_logs(self):
        log_dict = {
            'git_commit': os.environ.get('GIT_COMMIT', 'unknown'),
            'git_branch': os.environ.get('GIT_BRANCH', 'unknown'),
            'git_tag': os.environ.get('GIT_TAG', 'unknown'),
            'flight_id': self.flight_number,
            'system_id': self.proj_name
        }

        json_filename = os.path.join(self.root_folder, RUNLOG)
        with open(json_filename, "w") as outfile:
            json.dump(log_dict, outfile)

    def save_env_list(self):
        try:
            source_file = ENVLIST

            file_name = os.path.basename(source_file)

            # Create the destination path
            destination_path = os.path.join(self.root_folder, file_name)

            # Read the content of source file
            with open(source_file, 'r') as f:
                source_content = f.readlines()

            # Append flight number
            flight_line = "\nFlight number: {}\n".format(self.flight_number)
            source_content.append(flight_line)

            # Write file
            with open(destination_path, 'w') as f:
                f.writelines(source_content)

            self.get_logger().info(f"File {file_name} copied to {destination_path} successfully.")
        except FileNotFoundError:
            self.get_logger().info("Source file not found.")


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--proj_name", nargs='?', default="local")
    parser.add_argument("--server_w_ip", nargs='?', help='Provide Wireless IP address of main server')
    parser.add_argument("--server_eth_ip", nargs='?', help='Provide Ethernet IP address of main server')

    args = parser.parse_args()

    rclpy.init()

    node = RosbagRecord(args.proj_name, args.server_w_ip, args.server_eth_ip)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
