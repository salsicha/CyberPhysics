

# Configure the network
To configure the network on your local machine run
```
cd scripts
./setup_network.sh
```

The network changes are only temporary and will revert back to their original state on the next reboot.

# Quick test
Plug in a Blackly camera then run

```
cd deployment
docker compose -f blackfly.yml up --remove-orphans
```

Open up ```http://localhost:8888``` and view the topic /blackfly_camera/image_color. You should the camera streaming.

# Running the FTB system deployment
At the root of monorepo run
```sh
./start.sh FTB
```

Verify everything is working by going to ```http://localhost:8888```. Select the camera topics. You should see video streaming for all the cameras.

If you are seeing intermittent or no images it is possible the cameras are outputting too much data for the network to handle. Tweak the **CAMERA_THROUGHPUT_LIMIT** variable in ```systems/FTB/env/env.list```. You can also try lowering the frame rate by editing the blackflly yaml files.


# Spinview
This tool is very useful for playing around with the camera's settings.

Follow the instructions at http://softwareservices.flir.com/Spinnaker/latest/_spin_view_guide.html to install the Spinview tool.

# Useful links
* Troubleshooting image quality
    ```
    https://www.flir.com/support-center/iis/machine-vision/application-note/troubleshooting-image-consistency-errors/
    https://www.flir.com/support-center/iis/machine-vision/knowledge-base/lost-ethernet-data-packets-on-linux-systems/
    ```
* Firmware and documentation link:
    ```
    https://www.flir.com/support-center/iis/machine-vision/knowledge-base/technical-documentation-bfs-gige/
    ```

* Calibration
    * Monocluar
        ```
        http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
        ```
    * Multi-Inertial
        ```
        https://github.com/ethz-asl/kalir/wiki/camera-imu-calibration
        ```