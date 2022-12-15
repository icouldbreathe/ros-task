FROM andrewpresland/ros2-turtlebot3-sim

RUN apt-get update -q && \
    apt-get install -y \
    mysql-client \
    libmysqlcppconn-dev