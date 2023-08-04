# starling_controller

A Starling compatible UAV controller using ROS2, Docker and Kubernetes

This is a UAV project involving PX4 and ROS2. 

## Installation
### Pre-requisits
#### Install Docker
##### Set up the repository
1. Update the apt package index and install packages to allow apt to use a repository over HTTPS:
``````
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
``````

2. Add Docker’s official GPG key:
```
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
```

3. Use the following command to set up the repository:
```
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```
##### Install Docker Engine
1. Update the apt package index:
```    
sudo apt-get update
```
2. Install Docker Engine, containerd, and Docker Compose.
```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
#### Preparing the template project
1. Clone the template
```
git clone https://github.com/duy12i1i7/starling_controller.git
```
2. Preparing the image
```
cd starling-controller
sudo make
```




### UAV simulator

To test this locally, you can run the simulation stack using docker-compose specified in the dokcer-compose file in the deployment directory

```
docker-compose -f deployment/docker-compose.yml up
```

## Basic Usage

### Run the docker container
If the container has been pushed to the cloud, the most straightforward method is using docker in conjunction with the docker-compose scripts in ProjectStarling or Murmuration

```bash 
docker run -it --rm --net=bridge duynd2357/starling-controller:latest
```

### Building the docker container locally
First you will need to recursively clone the repo into your workspace, cd into the directory and then run `make`:
```
git clone --recursive https://github.com/duy12i1i7/starling_controller.git
cd starling_controller
make build
```

## Running in Simulation

### Starling CLI
This project then uses the [Murmuration project](https://github.com/StarlingUAS/Murmuration) in order to simplify the multi-vehicle control and simulation. Murmuration includes a Command Line interface and example scripts to help run multi-vehicle simulations. In particular the structure of these simulations directly matches the real life deployment within the flight arena.

To use Starling CLI, please use the following command line:
```
cd starling_controller
```
Note that the command line to cd into the `starling_controller` project that you cloned.

Following the instructions from Murmuration, it is recommended you add the Murmuration file path to your PATH. Either run in each terminal:
```
export PATH="$(pwd)/bin":$PATH
```

or add the following to the bottom of your bashrc/zshrc file:
```
export PATH="<path to Murmuration root folder>/bin":$PATH
```

then with the Starling CLI on the path, you should be able to run
```
starling install
```
If the command is not available, to use the Starling CLI, you should use the following command:
```
<path to Murmuration root folder>/bin/starling <any element after starling>
```
For example:
```
~/starling_controller/Murmuration/bin/starling install
```

### Running the simulation

First start the simulated cluster (here we start an example with 2 vehicles)
```
starling start kind -n 2
starling simulator start --load
```

After locally building the container (running `make`) you will need to upload it to the simulator
```
make
starling utils kind-load uobflightlabstarling/position-trajectory-controller:latest
```

Optionally, you can also load in the allocator and the dashboard
```
starling utils kind-load uobflightlabstarling/starling-allocator:latest
starling utils kind-load mickeyli789/starling-ui-dashly:latest
```

Finally, you can start this simulation by running
```
starling deploy -f kubernetes
```

To stop or restart the simulation, you can run:
```
starling deploy -f kubernetes stop
starling deploy -f kubernetes restart
```

Then you can open a browser page at https://localhost:8080 for gazebo and https://localhost:3000 for the user interface page.

You may also want to start the dashboard using the following command which opens on https://localhost:31771
```
starling start dashboard
```

## Project Details

Fill in your project detals here

## License

This project is covered under the MIT License.