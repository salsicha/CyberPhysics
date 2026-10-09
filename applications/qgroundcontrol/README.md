## QGroundControl

QGroundControl (QGC) provides full flight control and mission planning for any MAVLink enabled drones.

QGC enables users to command MAVLink enabled drones from a remote computer running QGC.

Our vehicle receives/transmits messages via ROS, whereas QGroundControl transmists/receives messages via
MAVLink. To be able to have our vehicle listen in to what is being commanded by QGroundControl, we need to 
use mavros, which enables MAVLink messages to be transcribed into ROS topics for our vehicle to subscribe
to. In order to do this, we need to create a gcs (ground control station) bridge, to enable mavros to be
the mediator between QGroundControl and our vehicle. 

The image installs MAVROS for ROS 2 Jazzy; `entrypoint.sh` shows an example `mavros_node` command with a
`gcs_url`, which is the communication node that will receive/transmit the MAVLink messages from QGroundControl
into ROS topics.

The image ships a pinned official QGroundControl release (`QGC_VERSION` build argument, extracted from the
release AppImage) and runs it as the non-root `ubuntu` user (uid 1000), because QGroundControl refuses to run
as root:

```bash
make -C applications build_qgroundcontrol
docker compose -f compositions/qgroundcontrol.yaml up
```