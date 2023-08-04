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

## Basic Usage
### Verify your controller
First, make sure you are in the root of your controller application. Now, let us verify that our own controller is at least compiling properly. We should get something like the following, and then stop it using `ctrl+c`:
```
make run
```

### Connect the simulator¶
Similar to previously, we can run the simulation stack provided for testing:

```bash 
docker-compose -f deployment/docker-compose.yml up
```
To access the simulator, go to `localhost:8080`, and to access the simple UI, go to `localhost:3000`.

In order for the controller to connect to the simulator, we need to find out which Docker network the simulator is running on.

To view the current docker networks, in a second terminal run
```bash
docker network ls
```
We are interested in the network named `<something>_default` and not named bridge and host which are Docker default networks. In our case `deployment_default` is the network created by docker compose when running the simulation stack. It should also be called `deployment_default` for you (the `<something>` is the parent folder of the docker compose file).

Then re-run (close the previously running one with `CTRL+C`) your controller with this new network. You can use the NETWORK variable from the Makefile
```
make NETWORK=deployment_default run
```

You could also simply specify the `--net=deployment_default` option with normal docker run

Success! It looks like the controller has received data from mavros and is currently waiting to start the mission!

To advance the mission, you can try pressing the green mission start button on the web interface. The vehicle should then indicate it is trying to take off, and you should be able to see it move in the simulator interface!

However you will notice that it gets stuck waiting for User Controller Ready. This is because it is waiting for a packet from the server with its initial location, but of course we have no server running so it never receives it!

To remedy this, you can open up another terminal in order to run the server. As mentioned previously, this can be achieved by setting the environment variable OFFBOARD to true using the following syntax

```
make NETWORK=deployment_default ENV="-e OFFBOARD=true" run
```

Repeating the previous steps to start the main controller on the network, we then get the following if we follow the instructions and press the go button every time the controller asks.
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
cd Murmuration
export PATH="$(pwd)/bin":$PATH
```

or add the following to the bottom of your bashrc/zshrc file:
```
export PATH="<path to Murmuration folder>/bin":$PATH
```

then with the Starling CLI on the path, you should be able to run
```
starling install
```
If the command is not available, to use the Starling CLI, you should use the following command:
```
<path to Murmuration  folder>/bin/starling <any element after starling>
```
For example:
```
~/starling_controller/Murmuration/bin/starling install
```

### Running the simulation
#### Requirement

If you have got this far, you will also need to install the kind utility. This can be done through the starling CLI:

```
starling install kind
```
#### Running the Multi-Drone Cluster

First start the simulated cluster (here we start an example with 2 vehicles)
```
starling start kind -n 2
```

IMPORTANT: The simulator can potentially be resource heavy to run, w.r.t both CPU usage and RAM.

This command can take as long as 30 minutes depending on your internet connection. It goes through the deployment files and downloads a couple of large containers e.g. the gazebo and sitl containers.

  Note: The `--brl` option automatically loads up the BRL flight arena simulated doubles

```
starling simulator load --brl
```

Once loading completes, you can then start the simulator using the `start` command:
```
starling simulator start --brl
# or both load and start at once
starling simulator start --brl --load
```

This should print the following output:
```
Starting simulator
Converting to use local registry at localhost:5001
deployment.apps/gazebo-v1 created
Converting to use local registry at localhost:5001
daemonset.apps/starling-px4-sitl-daemon created
Converting to use local registry at localhost:5001
daemonset.apps/starling-mavros-daemon created
```

#### Monitoring the cluster

A dashboard can be started to monitor the state of your current cluster.

```
starling start dashboard
```

This will start up the [Kubernetes dashboard](https://kubernetes.io/docs/tasks/access-application-cluster/web-ui-dashboard/). To access the dashboard, open up a browser and go to http://localhost:31771.

You will need the `<LONG TOKEN>` to log in. Copy and paste it into the Dashboard token. To get the access token again, run:
```
starling utils get-dashboard-token
```
Finally, for very quick diagnostics, you can also monitor the system using:
```
$ starling status
# or to continually watch
$ starling status --watch
```

The terminal looks like that:
```

Number of vehicles: 2
Nodes:
NAME                             STATUS   ROLES                  AGE   VERSION
starling-cluster-control-plane   Ready    control-plane,master   38m   v1.21.1
starling-cluster-worker          Ready    <none>                 38m   v1.21.1
starling-cluster-worker2         Ready    <none>                 38m   v1.21.1
Pods:
NAME                             READY   STATUS    RESTARTS   AGE
gazebo-v1-59d58485d8-pnt2g       1/1     Running   0          25m
starling-mavros-daemon-cqjhr     1/1     Running   0          25m
starling-mavros-daemon-nmptj     1/1     Running   0          25m
starling-px4-sitl-daemon-fllq5   1/1     Running   0          25m
starling-px4-sitl-daemon-gfxqs   1/1     Running   0          25m
Deployments:
NAME        READY   UP-TO-DATE   AVAILABLE   AGE
gazebo-v1   1/1     1            1           25m
StatefulSets:
No resources found in default namespace.
DaemonSets
NAME                       DESIRED   CURRENT   READY   UP-TO-DATE   AVAILABLE   NODE SELECTOR               AGE
starling-mavros-daemon     2         2         2       2            2           starling.dev/type=vehicle   25m
starling-px4-sitl-daemon   2         2         2       2            2           starling.dev/type=vehicle   25m
```

## Restarting on Stopping the Simulator
If the UAV simulation seems broken or you have put it in an unrecoverable state, you can restart the UAV simulation without deleting the cluster by running the `restart` command on the simulator:

```
starling simulator restart --brl
```
Note: This will stop the simulator by deleting all deployments within the cluster, and then restart. You can remove all deployments using `starling simulator stop`.

If you have finished testing for the day, something fundamental to the cluster has gone wrong (e.g. failed to connect to ports, networking etc), or you wish to change the number of drones in your cluster, you can stop and delete the cluster and everything in it by running:
```
starling stop kind
```


## Local Integration testing with KinD Digital Double
In your Starling application deployment folder, you should also see a `kubernetes.yaml` file. This file contains the instructions to tell Kubernetes which containers should be deployed to which places. It should be automatically generated to point to your project, so the image names here will be different to yours.

There are two configurations listed, one for the Onboard Controller, and one for the Offboard central controller, seperated by the `---`. Breaking both down together briefly.

In Starling we primarily make use of two types of deployment: the Deployment and the Daemonset. In general we use a deployment when we want a container to be deployed to a specific place, in our case the deployment of your applications offboard component into the central server. The Daemonset on the other hand is used for deploying something to any node which matches a given label, in our case, any drone on the network.

A bit further down on each you will see the nodeSelector and tolerations. All we'll say about these is that they enable us to choose where the containers are deployed. So the onboard controller is deployed to any node with the starling.dev/type: vehicle label, and similarly with the name: master label for the central monitor.

### Deploying your controller to KinD
First things first, we need to start up the testing stack. For more details, see the previous tutorial. Let's start it up with two drones within the BRL environment.
```
# Start the kind stack
starling start kind -n 2
# Start the dashboard for visualisation. Remember to grab the key and go to https://localhost:31771
starling start dashboard
# Load the images into KinD and start the simulator
starling simulator start --brl --load
```

As usual once everything is loaded, you should be able to see the simulator on https://localhost:8080.

    Note: If the simulator doesn't look like it's starting within 3 or 4 minutes, you can try and restart it by using restart instead of start.

### Deploying your controller
First, ensure that you have built your controller container. Do so by running the following from the root directory:
```
make
```

Also ensure that the default deployment file has been set up correctly and uses your image.

Then we can start, load and deploy your controller into the testing stack with the following command:
```
starling deploy -f deployment/kubernetes.yaml --load start
```

It may take a minute to load, but once it's done, you should see your new deployments on the Kubernetes dashboard.

If the vehicles just go straight into each other and crash in a very ugly way. In the real world, this would have ended up with 2 broken drones in need of repair. These are the exact situations that integration testing is meant to catch.

There are 3 steps involved:

  1. Rebuilding your container locally using make.
  2. Loading the updated image to the local repository so KinD has access to it.
  3. Restarting the deployment.

This can be achieved in the following line:
```
make && starling deploy -f deployment/kubernetes.yaml --load restart
```

## License

This project is covered under the MIT License.