# ROS 2 with native Zenoh

The ROS 2 API, topic names, services, actions and vehicle domain IDs stay the
same. The default backend is `rmw_zenoh_cpp`. ROS services load
[`middleware.env`](middleware.env), and the ROS base images install the Jazzy
Zenoh package. This replaces DDS in those processes; it is not a DDS bridge.

## Images and migration

Rebuild the connected stack together, including its derived images. Existing
`*-latest` containers do not acquire new middleware just by pulling this repo.
The common runtime is Ubuntu 24.04 / ROS 2 Jazzy. Aerostack2 is source-pinned
in its Dockerfile and installed at `/opt/aerostack2`; its descendant entrypoints
source that overlay. Isaac Sim 6.0.1 uses external Jazzy Python 3.12 libraries,
not its bundled ROS distribution. Space ROS rebuilds its pinned client libraries
and generated message support for the same native backend.
The SO101, UNO Q and VINS images also install Jazzy Zenoh directly.

For example, rebuild a flight stack with:

```bash
make -C applications build_aerostack2 build_demnav build_wildnav build_isaac
```

Build the other images used by your selected composition too (for example
`build_oak1` for the camera, or `build_ros2 build_so101` for the arm). Deploy the
same rebuilt tags to peers. Stop the old stack and `ros2 daemon stop` in host
shells before restarting with the new RMW. A host ROS shell also needs
`sudo apt install ros-jazzy-rmw-zenoh-cpp` and the same environment settings.
Native Zenoh Humble and Jazzy nodes do not interoperate because their type
hashes differ. Native Zenoh also does not interoperate with DDS nodes or with
`zenoh-bridge-ros2dds` without a separate application-level gateway.

## Local and LAN discovery

The defaults enable multicast scouting and dynamically assigned TCP listeners,
with no mandatory router. Host-networked containers on one computer discover
each other even when running under different UIDs. On a LAN, multicast must
reach peers and the firewall must allow their TCP listeners. This is intended
for the trusted robot LAN, not exposure to the public internet.

Keep `ROS_DOMAIN_ID` equal for nodes belonging to one robot and distinct for
unrelated test graphs. The domain is included in native Zenoh keys.

For a host ROS shell (Compose reads the shared file automatically):

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
export ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled=true;listen/endpoints=["tcp/0.0.0.0:0"];connect/endpoints=[]'
```

## Explicit router for routed networks

BlueOS occupies TCP 7447, so this repository's optional router listens on 7448:

```bash
docker compose -f compositions/zenoh_router.yaml up -d
```

On each ROS host, point the entire composition at a reachable router address:

```bash
export ZENOH_CONFIG_OVERRIDE='mode="client";scouting/multicast/enabled=false;listen/endpoints=[];connect/endpoints=["tcp/192.168.1.10:7448"]'
export ZENOH_ROUTER_CHECK_ATTEMPTS=0
docker compose -f compositions/so101_smolvla_isaac.yaml --profile sim up -d
```

Replace the address with the router's actual IP. Client mode is required here:
the native RMW router does not forward between listener-free peer sessions.
Here `0` waits for router availability; the default `-1` skips that check for
peer discovery. Docker's
`up -d` returns while a process waits. Allow TCP 7448 through the relevant
firewall. `ZENOH_PORT` changes the router port; clients must match it.
`ZENOH_ROUTER_PEERS='["tcp/192.168.1.11:7448"]'` connects routers. Do not set
client-only settings on the router: its composition provides its own listener.

## DDS fallback and exceptions

Set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` for the whole Zenoh composition to select
the installed Fast DDS fallback. All
communicating nodes must use the same middleware. The retained Fast DDS image
profiles and persistent 4 MiB host receive/send limits still serve this path.
The default Zenoh TCP transport does not need the DDS UDP buffer preflight.

`compositions/arduino.yaml` deliberately keeps its micro-ROS XRCE agent and
paired ROS subscriber on Fast DDS. The agent's DDS implementation is not an
RMW selection switch. Native Zenoh integration for microcontrollers requires
a separate firmware/agent migration.

The pinned `osrf/space-ros:jazzy-2026.07.0` base originally hard-wired Cyclone
DDS and introspection-only message dispatch. `applications/spaceros/enable_zenoh.sh`
rebuilds the RMW dispatcher, C/C++/Python clients and installed interfaces from
the exact source revisions in that image's manifest. Both its core and message
support need rebuilding; an environment override alone is insufficient.
Rebuild `spaceros` before starting the Space ROS or lunar rover compositions.
They now load the shared middleware settings, including explicit DDS fallback.
External rover drivers must select the same backend. Full Gazebo controller,
sensor/service traffic and the two-node lunar simulation passed native Zenoh
validation; this local rebuild does not claim upstream flight qualification.

The Navigator SITL image retains its
existing base; the flight compositions run its ArduPilot executable directly,
without ROS nodes. MAVLink, Gazebo Transport, ZeroMQ policy
RPC, camera USB transport and Foxglove's WebSocket connection are unchanged.

Upstream references: [native ROS RMW](https://github.com/ros2/rmw_zenoh),
[Isaac ROS installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html),
[Aerostack2 Jazzy support](https://github.com/aerostack2/aerostack2/issues/852).
