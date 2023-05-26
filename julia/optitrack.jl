using OptiTrack
using Sockets: IPv4

multicast_group = "239.255.42.99"
interface = IPv4("192.168.0.12")

connection = OptiTrackConnection(;multicast_group=multicast_group, local_interface=interface)

mocap_frame = receive(connection)