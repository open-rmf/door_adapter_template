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
        # Test connectivity
        try:
            res = requests.post(url=self.url+"/door/status", headers=self.header, json=self.data, timeout=1.0)
            res.raise_for_status()
            return True
        except (socket.gaierror, urllib3.exceptions.NewConnectionError, urllib3.exceptions.MaxRetryError, requests.exceptions.HTTPError ,requests.exceptions.ReadTimeout, requests.exceptions.ConnectionError) as e:
            print(f"Connection Error: {e}")
            return False

    def open_door(self):
        try:
            response = requests.post(url=self.url+"/door/remoteopen",headers=self.header, json=self.data, timeout=1.0)
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

    def get_mode(self):
        try:
            response = requests.post(url=self.url+"/door/status", headers=self.header, json=self.data, timeout=1.0) 
            if response:
                state = response.json().get("body").get("doorState")
                if state is None:
                    return 4
                elif state == "closed":
                    return 0
                elif state == "betweenOpenandClosed ":
                    return 1
                elif state == "open":
                    return 2
                elif state == "OFFLINE":
                    return 3
            else:
                return 4
        except (socket.gaierror, urllib3.exceptions.NewConnectionError, urllib3.exceptions.MaxRetryError, requests.exceptions.HTTPError ,requests.exceptions.ReadTimeout, requests.exceptions.ConnectionError) as e:
            print("Connection Error. "+str(e))
            return 4