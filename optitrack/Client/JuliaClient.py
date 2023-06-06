#Copyright © 2018 Naturalpoint
#
#Licensed under the Apache License, Version 2.0 (the "License")
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.


# OptiTrack NatNet direct depacketization sample for Python 3.x
#
# Uses the Python NatNetClient.py library to establish a connection (by creating a NatNetClient),
# and receive data via a NatNet connection and decode it using the NatNetClient library.

import os
import re
import sys
import stat

import time
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData

from multiprocessing import shared_memory
import numpy as np

SHARED_MEM_NAME = "test_sharedmem"

def print_configuration(natnet_client):
    print("Connection Configuration:")
    print("  Client:          %s"% natnet_client.local_ip_address)
    print("  Server:          %s"% natnet_client.server_ip_address)
    print("  Command Port:    %d"% natnet_client.command_port)
    print("  Data Port:       %d"% natnet_client.data_port)

    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s"% natnet_client.multicast_address)
    else:
        print("  Using Unicast")

    #NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()

    print("  NatNet Server Info")
    print("    Application Name %s" %(application_name))
    print("    NatNetVersion  %d %d %d %d"% (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))
    print("    ServerVersion  %d %d %d %d"% (server_version[0], server_version[1], server_version[2], server_version[3]))
    print("  NatNet Bitstream Requested")
    print("    NatNetVersion  %d %d %d %d"% (nat_net_requested_version[0], nat_net_requested_version[1],\
       nat_net_requested_version[2], nat_net_requested_version[3]))
    #print("command_socket = %s"%(str(natnet_client.command_socket)))
    #print("data_socket    = %s"%(str(natnet_shared_array[4]

def get_ip(environment_variable_name):
    ip = os.getenv(environment_variable_name)
    errmsg = f"Please set environment variable '{environment_variable_name}' to a valid IP address"
    if ip == None:
        raise Exception(errmsg)
    IPV4_REGEX_PATTERN = r"^((25[0-5]|(2[0-4]|1\d|[1-9]|)\d)\.?\b){4}$"
    if not re.match(IPV4_REGEX_PATTERN, ip):
        raise Exception(f"IP address does not match IPV4 regex pattern: '{ip}'. " + errmsg)
    return ip


def start(rb_callback):
    streaming_client = NatNetClient()
    streaming_client.set_client_address(get_ip("MY_IP"))
    streaming_client.set_server_address(get_ip("OPTITRACK_PC"))
    streaming_client.set_use_multicast(True)

    # Define shared memory and callbacks
    shared_block = shared_memory.SharedMemory(size=7 * 8, name=SHARED_MEM_NAME, create=True)
    shared_array = np.ndarray(shape=(7,), dtype=np.float64, buffer=shared_block.buf)

    # This is a callback function that gets connected to the NatNet client
    # and called once per mocap frame.
    def receive_new_frame(data_dict):
        order_list=[ "frameNumber", "markerSetCount", "unlabeledMarkersCount", "rigidBodyCount", "skeletonCount",
                    "labeledMarkerCount", "timecode", "timecodeSub", "timestamp", "isRecording", "trackedModelsChanged" ]
        dump_args = False
        if dump_args == True:
            out_string = "    "
            for key in data_dict:
                out_string += key + "="
                if key in data_dict :
                    out_string += str(data_dict[key]) + " "
                out_string+="/"
            print(out_string)

    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receive_rigid_body_frame( new_id, position, rotation ):
        #print( "Received frame for rigid body", new_id," ",position," ",rotation )
        # In this example: rigid body 1 is the only one that exists, and it's the one we care about
        if new_id == 1:
            idx = 0
            for p in position:
                shared_array[idx] = p
                idx += 1

            for r in rotation:
                shared_array[idx] = r
                idx += 1
        success = rb_callback(shared_array)

    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.new_frame_listener = receive_new_frame
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    try:
        is_running = streaming_client.run()
    except OSError as e:
        raise(f"Failed to run streaming client. This is probably because you have the IP addresses wrong: server: '{streaming_client.get_server_address()}', client: '{streaming_client.get_client_address()}'") from e

    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    is_looping = True
    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on.")
        try:
            sys.exit(2)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    print_configuration(streaming_client)
    print("\n")

    def get_rigidbody_data():
        x, y, z = shared_array[0], shared_array[1], shared_array[2]
        qw, qx, qy, qz = shared_array[3], shared_array[4], shared_array[5], shared_array[6]
        return (x,y,z), (qw, qx, qy, qz)

    return streaming_client, shared_block, shared_array, get_rigidbody_data

class PipeManager:
    def __init__(self) -> None:
        self._pipe_name = "/tmp/cam_optitrack_pipe"
        self.pipe = None
        self.i = 0

    def __enter__(self):
        if os.path.exists(self._pipe_name):
            assert stat.S_ISFIFO(os.stat(self._pipe_name).st_mode), "Internal error: Path exists but does not point to a pipe."
        else:
            os.mkfifo(self._pipe_name)
        print("Opening pipe. Waiting for other side to opened with read permissions...")
        self.pipe = open(self._pipe_name, 'wb')

    def __exit__(self, exception_type, exception_value, exception_traceback):
        _, self.pipe = self.pipe.close(), None
        os.remove(self._pipe_name)

    def write(self, data):
        if self.pipe is not None:
            print(f"\rWriting{'.'* (self.i % 2) :<5}", end="")
            self.i = (self.i + 1)
            self.pipe.write(data)
            try:
                self.pipe.flush()
            except BrokenPipeError:
                return False
        return True
        

if __name__ == "__main__":
    pm = PipeManager()
    def rb_callback(shared_array):
        return pm.write(bytes(shared_array))

    while True:
        print("Starting OptiTrack NatNetClient...\n")
        streaming_client, shared_block, shared_array, get_rigidbody_data = start(rb_callback)
        try:
            # streaming_client, shared_block, shared_array, get_rigidbody_data = start(lambda a: a)
            with pm:
                while True:
                    time.sleep(1)
        except BrokenPipeError:
            print("BrokenPipeError: pipe closed at other end.")
            print("Reopening pipe and continuing.")
            streaming_client.shutdown()
        except KeyboardInterrupt:
            streaming_client.shutdown()
            break
        
        
    print("Exiting...")
    shared_block.close()
    exit(0)