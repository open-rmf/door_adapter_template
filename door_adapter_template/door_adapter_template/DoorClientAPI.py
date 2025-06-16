import time
from rmf_door_msgs.msg import DoorMode

class DoorClientAPI:
    def __init__(self, node, config):
        self.name = 'door_adapter_template'
        self.timeout = 5  # seconds
        self.debug = False
        self.connected = False
        self.node = node
        self.config = config  # use this config to establish connection

        count = 0
        self.connected = True
        while not self.check_connection():
            if count >= self.timeout:
                print("Unable to connect to door client API.")
                self.connected = False
                break
            else:
                print("Unable to connect to door client API. Attempting to reconnect...")
                count += 1
            time.sleep(1)

    def check_connection(self):
        ''' Return True if connection to the door API server is successful'''
        ## ------------------------ ##
        ## IMPLEMENT YOUR CODE HERE ##
        ## ------------------------ ##
        return False

    def open_door(self, door_id):
        ''' Return True if the door API server is successful receive open door command'''
        ## ------------------------ ##
        ## IMPLEMENT YOUR CODE HERE ##
        ## ------------------------ ##
        return False

    def close_door(self, door_id):
        ''' Return True if the door API server is successful receive open door command'''
        ## ------------------------ ##
        ## IMPLEMENT YOUR CODE HERE ##
        ## ------------------------ ##
        return False

    def get_mode(self, door_id):
        ''' Return the door status with reference rmf_door_msgs. 
            Return DoorMode.MODE_CLOSED when door status is closed.
            Return DoorMode.MODE_MOVING when door status is moving.
            Return DoorMode.MODE_OPEN when door status is open.
            Return DoorMode.MODE_OFFLINE when door status is offline.
            Return DoorMode.MODE_UNKNOWN when door status is unknown'''
        ## ------------------------ ##
        ## IMPLEMENT YOUR CODE HERE ##
        ## ------------------------ ##
        return False
