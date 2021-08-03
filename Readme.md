# door_adapter_template

The objective of this package is to serve as a reference or template for writing a python based RMF door adapter.

> Note: This package is only one such example that may be helpful for users to quickly integrate door control with RMF.

## Step 1: Fill up missing code
Simply fill up certain blocks of code which make `REST` API calls to IOT door.
These blocks are highlighted as seen below and are found in `DoorClientAPI.py` 
```
## IMPLEMENT YOUR CODE HERE ##
```

The bulk of the work is in populating the `DoorClientAPI.py` file which defines a wrapper for communicating with the fleet of interest.

For example, if door offers a `REST API` with a `POST` method to obtain the status of the door, then the `DoorClientAPI::get_mode()` function may be implemented as below

```python
def get_mode(self):
    url = self.prefix + "/door/status" # example endpoint
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
    except HTTPError as http_err:
        print(f"HTTP error: {http_err}")
    except Exception as err:
        print(f"Other error: {err}")
    return None

```

Alternatively, if your door offers a websocket port for communication or allows for messages to be exchanged over ROS1/2, then these functions can be implemented using those protocols respectively.

>Note: you could refer to the example mock door_adapter in the `door_adapter_template/example` folder

## Step 2: Update config.yaml
The `config.yaml` file contains important parameters for setting up the door adapter. There are three broad sections to this file:

1. **name** : the door name to display and called by RMF
2. **api_endpoint** : REST API endpoint to communicate to the door

## Step 3: Run the door adapter:

Run the command below while passing the paths to the configuration file to operate on.

```bash
ros2 run door_adapter door_adapter -c CONFIG_FILE
```