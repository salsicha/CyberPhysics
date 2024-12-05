## QGroundControl

QGroundControl (QGC) provides full flight control and mission planning for any MAVLink enabled drones.

QGC enables users to command MAVLink enabled drones from a remote computer running QGC.

Our vehicle receives/transmits messages via ROS, whereas QGroundControl transmists/receives messages via
MAVLink. To be able to have our vehicle listen in to what is being commanded by QGroundControl, we need to 
use mavros, which enables MAVLink messages to be transcribed into ROS topics for our vehicle to subscribe
to. In order to do this, we need to create a gcs (ground control station) bridge, to enable mavros to be
the mediator between QGroundControl and our vehicle. 

In entrypoint.sh we run the command: rosrun mavros mavros_node _gcs_url:='udp://:14551@localhost:14550' &",
which is the communication node that will recieve/transmit the MAVLink messages from QGroundControl into ROS topics.