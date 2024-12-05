
* Install Matlab

    * Install Matlab, currently this must be done manually
        1. Download MPM: wget -q https://www.mathworks.com/mpm/glnxa64/mpm
        2. Make executable: chmod +x mpm
        3. Install: ./mpm install

    * Running Matlab in docker:
        (Be sure to "export" the environment variables!)
        ```
        docker run -it --rm --net host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -e XAUTHORITY -e MLM_LICENSE_FILE=/network.lic -v /home/${whoami}/repos/CyberPhysics/applications/client/scripts/simulink:/home/matlab/simulink --shm-size=512M cyberphysics/matlab matlab
        ```

    * Example compose file
        ```
        volumes:
            - ../applications/unrealengine/client/scripts/simulink:/home/matlab/simulink
        environment:
            - MLM_LICENSE_FILE=/network.lic
            - shm-size=512M
        command: >
            stdbuf -o L
            /usr/bin/tini -s -- matlab
        ```

    * Run Matlab manually without GUI
        ```
        matlab -nodesktop -nosplash -r "./load_simulink_test_model ; ./waypoint_to_time_trajectory ; exit"
        ```

    * Using a personal license
        * Linux
            * Get License File
                1. Get Host ID (MAC address)
                    - (https://www.mathworks.com/matlabcentral/answers/101892-what-is-a-host-id-how-do-i-find-my-host-id-in-order-to-activate-my-license?s_tid=srchtitle)
                    - i.e.: ifconfig enp0s31f6 | grep ether
                2. Get Computer Login Name
                    - (https://www.mathworks.com/matlabcentral/answers/96800-how-do-i-find-my-user-name-in-order-to-install-or-activate-my-license)
                    - i.e.: whoami
                    - The docker container uses the username "matlab"
                3. Create License File
                    - (https://www.mathworks.com/matlabcentral/answers/96751-how-do-i-update-my-matlab-license-file-for-an-individual-or-designated-computer-license)
                    1. Sign into mathworks.com
                    2. Click profile picture -> My Account
                    3. Click license number
                    4. Click "Install and Activate" tab
                    5. Click "Activate to Retrieve License File" on right side
                    6. Fill out form, click "Continue", click "yes" to installed question
                    7. Click "Download License File"
                4. Put license file in home directory
                    - i.e.: ~/license.lic
                5. Install license
                    - Set MLM_LICENSE_FILE environment variable to location of license file
                    - Start Matlab, it may prompt you to update the license, do so
                    - You may also still need to log in
