# Ros/Docker system overview:

### Docker overview
While the ros system can be hosted locally on device like the "jetson orion", this has the trade off of easy sensor intergration but at the cost of repeatability, so docker is used as a framework. It creates a vertual enveriment with instructions for running the robotic system and installed the required packages.

The core system files are the `start_docker.sh` and the `Dockerfiles`, start_docker.sh is a terminal executable `./start_docker.sh` which executes all of the code required to run the ros system. While the Dockerfiles define what packages to download and how.

### file overview
The list are used docker files are:
```
.dockerignore
.gitignore
.tripwire
Dockerfile.python
Dockerfile.ros
entry_point.sh
pull.sh
start_docker.sh

system_operations/system_control/
    Dockerfile.arduino
```

`.dockerignore` is a file used by docker to help reduce the number of files that docker attempts to rebuild when updating the docker image.

`.gitignore` is a file used by both github and the system to help decide what files should not be uplouded to the github repository and in turn wouldn't want to be used in the running system, they are "catche" files.

`.tripwire` helps prevent loss of work. To ensure efficenty of pull and uploding code to github, the local system must be capable of overwiting it's own files, so to prevent you accidently running the command on your own system the .tripwire file stops all overwrite operations. 

The `Dockerfile.python`, `Dockerfile.ros`, and `Dockerfile.arduino` files define how the system builds the python, ros, and arduino image. Due to the wide usage of ros the python image is unused but the ros image install the stuff required to run ros and the arduino image installs the drivers needed to uploude code to the arduino. 

`entry_point.sh` is an unused file, tipically it can execute code in a ros containor.

`pull.sh` Is a split file for pulling from the github or over a local ssh seassion, allowing it to be executed on it's own to pull live while inside of a containor.

`start_docker.sh` is the main start file which starts all robot systems and docker


### ./start_docker.sh overview
