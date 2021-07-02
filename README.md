# door_adapter_template

This package contains template for a door adapter to communicate with rmf and the door REST API server. 

## Table of Contents

1. [System Requirement](#system-requirements)
2. [General Working Principle](#general-working-principle)
3. [Package Overview](#package-overview)
    
    3a. [Directory Structure](#directory-structure)

4. [Usage](#usage)
5. [More Detail Reference](#more-detail-reference)


## System Requirements

The following software needs the below mentioned setup:
* <a href="https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html">ROS 2 Foxy</a>
* Robotics Middleware Framework (RMF) - <a href = "https://github.com/open-rmf/rmf"> RMF installation instruction</a>

  

## General Working Principle

* The door adapter is in charge of publishing its state and receiving commands over ROS 2
* The door adapter will keep publishing `/door_states` in 1 Hz
* The door adapter will assume the door status is closed when there is not door_request to the door adapter from `/adapter_door_requests`
* The door adapter will only being triggered to check door status and request door to open when it is being called by `/adapter_door_requests`

### Published Topics

The messages and topics used by rmf door adapter:

Publish:
*  Topic `/door_states` (message types: `rmf_door_msgs/DoorState`): topic that describes the status of the door and is published by door adapter 

Subscribe:
*  Topic `/adapter_door_requests` (message types: `rmf_door_msgs/DoorRequest`): topic that that is request by the rmf fleet adapter

The overview flow could be seen below:

![](doc/RMF_DOOR_Adapter_Architecture.png)


## Package Overview

This is the general overview of the structure of this Library, and the listed files are described.

### Directory Structure
Below is the directory structure of the validation package, including the general function of certain scripts.

    |--door_adapter_template
        |-- doc                                 #document file
        |-- door_adapter                        #source file of the rmf door node
            |-- __init__.py
            |-- door_adapter.py                 #door node direct commnuicate with RMF and doorclientAPI
            |-- DoorExampleClientAPI.py         # Example of Door Client Example in REST method, where user will need to modify according to the door REST API Server
            |-- mock_door_server.py             # a mock door server
        |-- resource
        |-- test
        |-- config.yaml                         #configuration file
        |-- package.xml
        |-- setup.cfg
        |-- setup.py
        |-- README.md                   #contains installation and build/usage instruction
        
           
The sub file structure could be seen inside the packages.

## Usage

Certain Assumption is made for this door adapter template:
- The communication protocol for door node to the physical door controller is through [REST](https://searchapparchitecture.techtarget.com/definition/RESTful-API) API
- The expected response from API as below:
    ![](doc/exampledoorAPI.png)
- Assume API doesn't have close door API

if the user's door condition is same as above, user can directly edit the parameter of `config.yaml` file as below:

    door:
        name: "example_door1"                               # input the door name as you wish
        api_endpoint: "https://door_endpoint_example/"      # API endpoint that would required to get from the API vendor/provider
        header_key: "example_key"                           # depend of header key and value required by door API server
        header_value: "example_header_value"                # depend of header key and value required by door API server
        door_id: "example_door_id"                          # depend of header key and value required by door API server

### Build Instruction
1. Install all non-ROS dependencies of RMF packages
    ```
    sudo apt update && sudo apt install \
        git cmake python3-vcstool curl \
        qt5-default \
        ignition-edifice \
        -y
    python3 -m pip install flask-socketio
    sudo apt-get install python3-colcon*
    ```

2. Build RMF
    ```
    mkdir -p ~/rmf_ws/src
    cd ~/rmf_ws
    wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
    vcs import src < rmf.repos
    cd ~/rmf_ws
    rosdep install --from-paths src --ignore-src --rosdistro foxy -yr
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

3. Build door_adapter
    ```
    mkdir -p ~/rmf_door_ws/src
    git clone https://github.com/open-rmf/door_adapter_template.git
    source /opt/ros2/foxy/setup.bash
    source ~/rmf_ws/install/setup.bash
    cd ~/rmf_door_ws
    colcon build
    ```

4. Usage

    a) Run Mock door Server
    ```
    cd ~/rmf_door_ws
    python src/door_adapter_template/door_adapter/mock_door_server.py
    ```

    b) Open another **terminal** and run
    ```
    cd ~/rmf_door_ws
    source /opt/ros2/foxy/setup.bash
    source ~/rmf_ws/install/setup.bash
    source install/setup.bash
    ros2 run door_adapter door_adapter -c src/door_adapter_template/config.yaml
    ```


## More detail reference

- https://osrf.github.io/ros2multirobotbook/integration_doors.html
- https://docs.python-requests.org/en/master/

[Back To Top of Page](#table-of-contents)