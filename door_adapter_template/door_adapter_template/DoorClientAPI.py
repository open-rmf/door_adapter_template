import requests
import json
import urllib3
import socket
import time

class DoorClientAPI:
    def __init__(self,url,api_key,api_value,door_id):
        self.url = url
        self.header = {api_key:api_value}
        self.data = {"id": door_id}

        count = 0
        self.connected = True
        while not self.check_connection():
            if count >= 5:
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

    def open_door(self):
        ''' Return True if the door API server is successful receive open door command'''
        ## ------------------------ ##
        ## IMPLEMENT YOUR CODE HERE ##
        ## ------------------------ ##
        return False

    def get_mode(self):
        ''' Return the door status with reference rmf_door_msgs. 
            Return 0 when door status is closed.
            Return 1 when door status is moving.
            Return 2 when door status is open.
            Return 3 when door status is offline.
            Return 4 when door status is unknown'''
        ## ------------------------ ##
        ## IMPLEMENT YOUR CODE HERE ##
        ## ------------------------ ##
        return False
