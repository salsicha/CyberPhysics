# Dependencies
* Run this script (You must read output and follow manual instructions!!!)
    ```
    tools/install/install.sh
    ```
# Add an app
1. Navigate to the apps folder and create a new directory. Assuming starting from monorepo folder
    ```
    cd apps
    mkdir <your_app_name>
    ```
    where <your_app_name> is replaced by the name of the app you are creating formatted lower case and with underscores instead of spaces

2. Navigate to your newly created directory and create a dockerfile
    ```cd your_app_name
    touch dockerfile
    #to learn more about docker visit their documentation
    ```
3. Setup your docker file to run the application for a simple starting point look at the lostlink dockerfile
4. Test out the docker container
    ```
    make your_app_name
    docker run -t your_app_name
    ```
If no errors then you can go write your app and update the docker file as needed.
6. Setting up the build tools. Open the /cyberphysics/apps/Makefile go to the #Build section add
```build_your_app_name: cleanup
APP=your_app_name ./_build_it.sh
```
to the end of the #Build section

7. Create a docker compose yaml file
    1. Navigate to /cyberphysics/deployment/ assuming you were in apps before from step 6
        ```
        cd ../deployment
        ```
    2. Create a docker yml file for your application
        ```
        touch your_app_name.yml
        ```
    3. Populate your yaml file. The claw.yml can be used as an example. General but not exhaustive descriptions are listed below

8. Test everything
    1. Return to the app directory and run ```make build_your_app_name```
    2. If there were no errors return to the deployment file and run ```docker compose -f your_app_name.yml up --remove-orphans```
9. Add a README.md in your app directory describing purpose and usage

# About Docker multi-stage build
See https://docs.docker.com/build/building/multi-stage/.

We make use of multi-stage build to reduce the final image size. In a Dockerfile whenenver you call a RUN statement that installs some software (eg. compiler) it will increase the image size, even if you remove it later on in a different RUN. The only way to not increase the image size is to install your tools, do your thing, then remove the tool, in a single RUN. This cam get unwieldy and limits the use of layer caching.

# Build images
* How to build images
    * To build a app (e.g., `rviz`)
        ```bash
        make build_rviz
        ```

# Run individual containers
* How to run containers
    * To run bash from a ROS2 container
        ```bash
        docker run -ti --rm cyberphysics/ros2 /bin/bash
        ```

# Run containers with orchestration
* Docker compose blackfly example:
    * From the "deployments" folder:
        * Start:
            ```bash
            docker compose -f blackfly.yml up
            ```
        * Stop:
            ```bash
            docker compose -f blackfly.yml down --remove-orphans
            ```

