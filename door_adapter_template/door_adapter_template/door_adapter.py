import sys
import yaml
import argparse

import time
import threading

import rclpy
from door_adapter_template.DoorClientAPI import DoorClientAPI
from rclpy.node import Node
from rmf_door_msgs.msg import DoorRequest, DoorState, DoorMode


class Door:
    def __init__(self,
                 id,
                 door_auto_closes,
                 door_signal_period,
                 continuous_status_polling):
        self.id = id
        self.door_mode = DoorMode.MODE_CLOSED
        self.open_door = False
        self.check_status = None  # set to None if not enabled
        self.door_auto_closes = door_auto_closes
        self.door_signal_period = door_signal_period
        if continuous_status_polling:
            self.check_status = False

###############################################################################

class DoorAdapter(Node):
    def __init__(self,config_yaml):
        super().__init__('door_adapter_template_node')
        self.get_logger().info('Starting door adapter...')

        # Get value from config file
        self.door_state_publish_period = config_yaml['door_publisher']['door_state_publish_period']

        door_pub = config_yaml['door_publisher']
        door_sub = config_yaml['door_subscriber']
        self.mock_adapter = config_yaml.get('mock', False)

        # Connect to doors
        if not self.mock_adapter:
            self.api = DoorClientAPI(self, config_yaml)

            assert self.api.connected, "Unable to establish connection with door"

            # Keep track of doors
            self.doors = {}
            for door_id, door_data in config_yaml['doors'].items():
                # We support both door_auto_closes and the deprecated
                # door_close_feature for backward compatibility
                auto_close = door_data.get('door_auto_closes', None)
                if auto_close is None:
                    if 'door_close_feature' in door_data:
                        auto_close = not door_data['door_close_feature']
                assert auto_close is not None

                self.doors[door_id] = Door(door_id,
                                           auto_close,
                                           door_data['door_signal_period'],
                                           door_data.get('continuous_status_polling', False))

        self.door_states_pub = self.create_publisher(
            DoorState, door_pub['topic_name'], 100)

        self.door_request_sub = self.create_subscription(
            DoorRequest, door_sub['topic_name'], self.door_request_cb, 100)

        self.periodic_timer = self.create_timer(
            self.door_state_publish_period, self.time_cb)

    def door_open_command_request(self, door_data: Door):
        # assume API doesn't have close door API
        # Once the door command is posted to the door API,
        # the door will be opened and then close after 5 secs    
        while door_data.open_door:
            success = self.api.open_door(door_data.id)
            if success:
                self.get_logger().info(f"Request to open door [{door_data.id}] is successful")
            else:
                self.get_logger().warning(f"Request to open door [{door_data.id}] is unsuccessful")
            time.sleep(door_data.door_signal_period)

    def time_cb(self):
        if self.mock_adapter:
            return
        for door_id, door_data in self.doors.items():

            if door_data.check_status is not None:
                # If continuous_status_polling is enabled, we will only update
                # the door state when there is a door open request. If there is
                # a close door request and the door state is closed, we will
                # assume the door state remains closed until the next door open
                # request. This implementation reduces the number of calls made
                # during state update.
                if door_data.check_status:
                    door_data.door_mode = self.api.get_mode(door_id)
                    if door_data.door_mode == DoorMode.MODE_CLOSED and not door_data.open_door:
                        door_data.check_status = False
            else:
                # If continuous_status_polling is not enabled, we'll just
                # update the door state as it is all the time
                door_data.door_mode = self.api.get_mode(door_id)
            state_msg = DoorState()
            state_msg.door_time = self.get_clock().now().to_msg()

            # publish states of the door
            state_msg.door_name = door_id
            state_msg.current_mode.value = door_data.door_mode
            self.door_states_pub.publish(state_msg)

    def door_request_cb(self, msg: DoorRequest):
        # Agree to every request automatically if this is a mock adapter
        if self.mock_adapter:
            state_msg = DoorState()
            state_msg.door_time = self.get_clock().now().to_msg()
            state_msg.door_name = msg.door_name
            state_msg.current_mode.value = msg.requested_mode.value
            self.door_states_pub.publish(state_msg)
            return

        # Check if this door has been stored in the door adapter. If not, ignore
        door_data = self.doors.get(msg.door_name)
        if door_data is None:
            return

        # When the adapter receives an open request, it will send an open
        # command to API. When the adapter receives a close request, it will
        # stop sending the open command to API
        self.get_logger().info(
            f"[{msg.door_name}] Door mode [{msg.requested_mode.value}] "
            f"requested by {msg.requester_id}"
        )
        if msg.requested_mode.value == DoorMode.MODE_OPEN:
            # open door implementation
            door_data.open_door = True
            if door_data.check_status is not None:
                # If check_status is enabled, we toggle it to true to allow
                # door state updates
                door_data.check_status = True
            if not door_data.door_auto_closes:
                self.api.open_door(msg.door_name)
            else:
                t = threading.Thread(target=self.door_open_command_request,
                                     args=(door_data,))
                t.start()
        elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
            # close door implementation
            door_data.open_door = False
            self.get_logger().info(f'[{msg.door_name}] Close Command to door received')
            if not door_data.door_auto_closes:
                self.api.close_door(msg.door_name)
        else:
            self.get_logger().error('Invalid door mode requested. Ignoring...')

###############################################################################

def main(argv=sys.argv):
    rclpy.init(args=argv)

    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="door_adapter",
        description="Configure and spin up door adapter for door ")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file for this door adapter")
    args = parser.parse_args(args_without_ros[1:])
    config_path = args.config_file

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    door_adapter = DoorAdapter(config_yaml)
    rclpy.spin(door_adapter)

    door_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
