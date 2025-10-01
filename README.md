# Docker on MacOS

## MacOS Installation Process
### Install Docker Desktop
```
https://www.docker.com/products/docker-desktop/
```
- Choose the correct download file depending on your **CPU**
- You can check which type of CPU you have by going to the top left of your screen and clicking the '**about**' on your mac book
- Make a new folder for ros2 on your computer (make sure the folder is not connected to ONEDRIVE)
## Open folder location in terminal
- right click the folder and press "Open in Terminal"
# Grab necessary files from github
```
git init
git remote add origin 
git remote add origin https://github.com/UTSARobotics/ros2-docker-crashcourse.git
git fetch origin windows
git pull origin windows
```
#  Post Dependecies
### Test that docker is running correctly
```
docker run hello-world
```
##  Test ROS2 Docker
```
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_QPA_PLATFORM=xcb \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  utsarobotics/ros2-humble:1.0.0 \
  bash
```

