# V2X server

## Description:

**The V2X server acts as a central communication hub. It publishes EVCS (Electric Vehicle Communication Standard) messages, which include essential information about the status of parking spots. This server is crucial for facilitating efficient data exchange between the infrastructure and autonomous vehicles, enabling features like real-time updates on parking spot availability and payment information. The system also displays the parking spot status on a Tkinter-based GUI, showing whether each spot is occupied or available.**


## Description of the interfaces :

| Data | Topic Name| Ros2 Message  | Description | 
| --------- | ---------- | ---------- | ----------- |
| output | /parking_status| v2x/EVChargingSpotNotificationPOIMessage |It publishes the notifications regarding the status of parking spots. these notifications provide details such as payment status, spot ID, availability, and whether the spot is currently vacant or occupied.  |

## Installation Instructions

To install the V2X server, follow the steps  below:

**Establish a workspace, enter it, and subsequently create a "src" folder, navigating into it.**

### Step 1: Clone the Repository

Initiate the process by launching a terminal and navigating to the designated directory for cloning the package. Subsequently, execute the following command to clone the repository:

```shell
git clone https://git.hs-coburg.de/PARKONOMOUS/v2x_server.git

git clone https://git.hs-coburg.de/pau5849s/mocap_msgs.git

git clone https://git.hs-coburg.de/Autonomous_Driving/v2x.git
```

### Step 2: Build the Package
Upon successful completion of the cloning process, go into the workspace.
```shell
cd ..
```
Next, build the package by using the colcon command.
```shell
colcon build --symlink-install
```

### Step 3: Source the Workspace
After the build process is complete, you'll need to source the workspace to make it accessible for ROS2.

```shell
source install/setup.bash
```
### Step 4: Run the node
You can now launch the node:

```shell
ros2 run v2x_server v2x_server 
```

```
