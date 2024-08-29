import requests
import urllib3
import socket
import time
from rmf_door_msgs.msg import DoorMode

class DoorClientAPI:
    def __init__(self, node, config):
        self.name = 'rmf_door_adapter'
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
        # Test connectivity
        try:
            res = requests.post(url=self.url+"/door/status", headers=self.header, json=self.data, timeout=1.0)
            res.raise_for_status()
            return True
        except (socket.gaierror, urllib3.exceptions.NewConnectionError, urllib3.exceptions.MaxRetryError, requests.exceptions.HTTPError ,requests.exceptions.ReadTimeout, requests.exceptions.ConnectionError) as e:
            print(f"Connection Error: {e}")
            return False

    def open_door(self, door_id):
        try:
            response = requests.post(url=self.url+f"/door/remoteopen/{door_id}",headers=self.header, json=self.data, timeout=1.0)
            if response:
                result = response.json()["body"]
                if (result.get("result") is not None):
                    return True
                else:
                    print("door could not perform open")
                    return False
            else:
                print("Invalid response received")
                return False
        except (socket.gaierror, urllib3.exceptions.NewConnectionError, urllib3.exceptions.MaxRetryError, requests.exceptions.HTTPError ,requests.exceptions.ReadTimeout, requests.exceptions.ConnectionError) as e:
            print("Connection Error. "+str(e))
            return False

    def get_mode(self, door_id):
        try:
            response = requests.post(url=self.url+f"/door/status/{door_id}", headers=self.header, json=self.data, timeout=1.0) 
            if response:
                state = response.json().get("body").get("doorState")
                if state is None:
                    return DoorMode.MODE_UNKNOWN
                elif state == "closed":
                    return DoorMode.MODE_CLOSED
                elif state == "betweenOpenandClosed ":
                    return DoorMode.MODE_MOVING
                elif state == "open":
                    return DoorMode.MODE_OPEN
                elif state == "OFFLINE":
                    return DoorMode.MODE_OFFLINE
            else:
                return 4
        except (socket.gaierror, urllib3.exceptions.NewConnectionError, urllib3.exceptions.MaxRetryError, requests.exceptions.HTTPError ,requests.exceptions.ReadTimeout, requests.exceptions.ConnectionError) as e:
            print("Connection Error. "+str(e))
            return DoorMode.MODE_UNKNOWN
