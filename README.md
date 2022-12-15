# Readme

I used a Docker container to set up the environment for ros2 and turtlebot3 and used another container for the mysql database. The code of the node is in workspace/cpp_pubsub/src.

## Setup

If you want to recreate the environment I used:

1. Clone the repository

2. Get submodules for turtlebot3

```
git submodule update --init --recursive
```

3. Build the Docker image

```
docker build -t turtlebot3 .
```

4. Run the container stack

```
docker compose up
```

5. Open a terminal inside the container

```
docker exec -it ros2-turtlebot3-sim-turtlebot3-1 /bin/bash

su ubuntu
```

6. Build the turtlebot3 and its dependencies

```
colcon build --symlink-install
```

7. Source the setup.bash

```
source ./install/setup.bash
```

8. Build cpp_subpub

9. Run the node

```
ros2 run cpp_pubsub mysqlnode
```
