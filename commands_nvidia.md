## Installing NVIDIA Container Toolkit with Apt:

1. Configure the production repository:
```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
2. Update the packages list from the repository:
```
sudo apt-get update
```
3. Install the NVIDIA Container Toolkit packages:
```
sudo apt-get install -y nvidia-container-toolkit
```

## Configuration NVIDIA Container Toolkit:
4. Configure the container runtime by using the nvidia-ctk command:
```
sudo nvidia-ctk runtime configure --runtime=docker
```
5. Restart the Docker daemon:
```
sudo systemctl restart docker
```
6. Configure the container runtime by using the `nvidia-ctk` command:
```
nvidia-ctk runtime configure --runtime=docker --config=$HOME/.config/docker/daemon.json
```
7. Restart the Rootless Docker daemon: 
```
systemctl --user restart docker
```
8. Configure `/etc/nvidia-container-runtime/config.toml` by using the sudo `nvidia-ctk` command:
```
sudo nvidia-ctk config --set nvidia-container-cli.no-cgroups --in-place
```

## Developer Environment Setup:
9. Restart Docker: 
```
sudo systemctl daemon-reload && sudo systemctl restart docker
```
10. Install Git LFS to pull down all large files:
```
sudo apt-get install git-lfs
git lfs install --skip-repo
```
11. Create a ROS 2 workspace for experimenting with Isaac ROS:
```
mkdir -p  ~/workspaces/isaac_ros-dev/src
echo "export ISAAC_ROS_WS=${HOME}/workspaces/isaac_ros-dev/" >> ~/.bashrc
source ~/.bashrc
```

## Installation Isaac AprilTag:
12. Clone `isaac_ros_common` under `${ISAAC_ROS_WS}/src`.
```
cd ${ISAAC_ROS_WS}/src && \
   git clone -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
```
13. Download quickstart data from NGC: (Make sure required libraries are installed.)
```
sudo apt-get install -y curl jq tar
```
14. Then, run these commands to download the asset from NGC:
```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_apriltag"
NGC_RESOURCE="isaac_ros_apriltag_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=1
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
```
15. Build isaac_ros_apriltag. Launch the Docker container using the `run_dev.sh` script:
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh
```
16. Install the prebuilt Debian package:
```
sudo apt-get install -y ros-humble-isaac-ros-apriltag
```
17. Continuing inside the Docker container, install the following dependencies:
```
sudo apt-get install -y ros-humble-isaac-ros-examples
```

---

# Sources:
1. [Developer Environment Setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)
2. [AprilTag NVIDIA ISAAC ROS ](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#quickstart)
