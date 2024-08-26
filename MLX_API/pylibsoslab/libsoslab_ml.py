import threading
import queue
import socket
import struct
from typing import List

# Point class to represent a point in 3D space
class Point:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

# Scene class to represent the captured data from the ML
class Scene:
    def __init__(self):
        self.timestamp: List[float] = []
        self.rows: int = 56
        self.cols: int = 192
        self.ambient_image: List[int] = []
        self.depth_image: List[int] = []
        self.intensity_image: List[int] = []
        self.pointcloud: List[Point] = []

    # Initialize the scene data structure
    def initialize(self, depth_completion: bool):
        if depth_completion:
            self.cols = 576

        self.timestamp = [0] * self.rows
        self.ambient_image = [0] * (self.rows * 576)
        self.depth_image = [0] * (self.rows * self.cols)
        self.intensity_image = [0] * (self.rows * self.cols)
        self.pointcloud = [Point(0.0, 0.0, 0.0) for _ in range((self.rows * self.cols))]

# LidarML class to handle ML operations
class LidarML:
    def __init__(self):
        self._udp_thread: threading.Thread = None
        self._parser_thread: threading.Thread = None

        self._thread_run: threading.Event = threading.Event()
        self._shared_var_port: int = 0
        self._shared_var_queue_packet: queue.Queue = queue.Queue(128)

        self._tcp_socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._udp_socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._max_packet_size = 65532
        #Default ML IP/Port
        self._ml_ip = '192.168.1.10'
        self._ml_port = 2000

        self._scene: Scene = Scene()
        self._initialize_data: bool = False
        self._shared_var_queue_data: queue.Queue = queue.Queue(128)
        self._shared_queue_data_lock: threading.Lock = threading.Lock()

    # API information
    def api_info(self) -> str:
        return "SOSLAB LiDAR ML-X API v2.3.1 build"

    # Check if ML is connected
    def is_connected(self) -> bool:
        try:
            self._tcp_socket.getsockname()
            return True
        except Exception:
            return False

    # Connect to the ML
    def connect(self, ml_ip: str = "192.168.1.10", ml_port: int = 2000) -> bool:
        self._ml_ip = ml_ip
        self._ml_port = ml_port
        self._tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self._tcp_socket.connect((self._ml_ip, self._ml_port))
            bind_address, temp_port = self._tcp_socket.getsockname()
            self._shared_var_port = temp_port
            print("Connected to %s on port %s" % (self._ml_ip, self._ml_port))
            return True
        except socket.error as error:
            print("Connection to %s on port %s failed: %s" % (self._ml_ip, self._ml_port, error))
            return False

    # Disconnect from the ML
    def disconnect(self):
        self._tcp_socket.close()
        self._udp_socket.close()
        print("Disconnected from %s on port %s" % (self._ml_ip, self._ml_port))

    # Start the ML data acquisition
    def run(self):
        self._initialize_data = False
        retval = self._tcp_thread_func('run', 0)

        if retval:
            self._thread_run.clear()
            if self._parser_thread is None:
                self._parser_thread = threading.Thread(target=self._parser_thread_func)
                self._parser_thread.start()
            if self._udp_thread is None:
                self._udp_thread = threading.Thread(target=self._udp_thread_func)
                self._udp_thread.start()

    # Stop the ML data acquisition
    def stop(self):
        self._thread_run.set()
        if self._udp_thread is not None:
            self._udp_thread.join()
            self._udp_thread = None
        if self._parser_thread is not None:
            self._parser_thread.join()
            self._parser_thread = None

        retval = self._tcp_thread_func('stop', 0)
        if retval:
            self._udp_socket.close()

    # Set frame interval to 10 FPS
    def fps10(self, enable: bool) -> bool:
        return self._tcp_thread_func('"frame_interval":10' if enable else '"frame_interval":20', 1)
    
    # Enable or disable depth completion
    def depth_completion(self, enable: bool) -> bool:
        return self._tcp_thread_func('"depth_complt_en":1' if enable else '"depth_complt_en":0', 1)

    # Enable or disable ambient data packet
    def ambient_enable(self, enable: bool) -> bool:
        return self._tcp_thread_func('"packet_ambient_en":1' if enable else '"packet_ambient_en":0', 1)

    # Enable or disable depth data packet
    def depth_enable(self, enable: bool) -> bool:
        return self._tcp_thread_func('"packet_depth_en":1' if enable else '"packet_depth_en":0', 1)

    # Enable or disable intensity data packet
    def intensity_enable(self, enable: bool) -> bool:
        return self._tcp_thread_func('"packet_intensity_en":1' if enable else '"packet_intensity_en":0', 1)

    # TCP communication function
    def _tcp_thread_func(self, msg: str = None, send_type: int = None, warning_print: int = 0) -> bool:
        send_msg = '{"command":"' + msg + '"}' if send_type == 0 else '{"PL_Reg":{' + msg + '}}'

        try:
            self._tcp_socket.sendall(send_msg.encode('utf-8'))
            recv_msg = self._tcp_socket.recv(self._max_packet_size).decode('utf-8')

            if "json_ack" in recv_msg:
                if "true" in recv_msg:
                    if warning_print != 1:
                        print(f"{send_msg} : Success")
                    return True
                elif "false" in recv_msg:
                    if warning_print != 1:
                        print(f"{send_msg} : Fail")
                    return False
            else:
                if warning_print != 1:
                    print(f"Recv: {recv_msg}")

        except socket.error as error:
            print("Connection to %s on port %s failed: %s" % (self._ml_ip, self._ml_port, error))

    # UDP communication thread function
    def _udp_thread_func(self):
        try:
            self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp_socket.bind(('0.0.0.0', self._shared_var_port))
            print("Connected to %s on port %s" % (self._ml_ip, self._shared_var_port))
            self._thread_run.clear()

            while not self._thread_run.is_set():
                data, addr = self._udp_socket.recvfrom(self._max_packet_size)

                try:
                    decoded_data = data[:8].decode('utf-8')
                    if decoded_data == "LIDARPKT":
                        self._shared_var_queue_packet.put(data)
                    else:
                        print("Received data:", decoded_data)

                except UnicodeDecodeError:
                    print("NON LiDAR DATA")

        except socket.error as error:
            print("Connection to '0.0.0.0' on port %s failed: %s" % (self._shared_var_port, error))

    # Data parser thread function
    def _parser_thread_func(self):
        while not self._thread_run.is_set():
            try:
                packet_data = self._shared_var_queue_packet.get(timeout=0.1)
                buffer_read = 8

                # Parse timestamp
                timestamp_byte_data = packet_data[buffer_read:buffer_read + 8]
                value = int.from_bytes(timestamp_byte_data, byteorder='little')
                sec = (value >> 32) & 0xFFFFFFFF
                nsec = value & 0xFFFFFFFF
                timestamp = sec + (nsec / 1e9)
                buffer_read += 16

                # Parse status
                status_byte_data = packet_data[buffer_read:buffer_read + 1]
                status_value = struct.unpack('B', status_byte_data)[0]
                depth_completion = (status_value >> 4) & 0x01
                ambient_disable = (status_value >> 5) & 0x01
                depth_disable = (status_value >> 6) & 0x01
                intensity_disable = (status_value >> 7) & 0x01
                buffer_read += 2

                # Parse row number
                row_packet = packet_data[buffer_read:buffer_read + 1]
                row_num = int.from_bytes(row_packet, byteorder='little')
                buffer_read += 6

                if row_num == 0 and not self._initialize_data:
                    self._scene.initialize(depth_completion)
                    self._initialize_data = True

                if not self._initialize_data:
                    continue

                self._scene.timestamp[row_num] = timestamp
                ambient_data_size = 576
                start_idx = row_num * ambient_data_size
                end_idx = (row_num + 1) * ambient_data_size

                if not ambient_disable:
                    # Unpack ambient image
                    ambient_byte_data = packet_data[buffer_read:buffer_read + 4 * ambient_data_size]
                    ambient_values = struct.unpack('<' + 'I' * ambient_data_size, ambient_byte_data)
                    self._scene.ambient_image[start_idx:end_idx] = ambient_values
                    buffer_read += 4 * ambient_data_size

                depth_unpack_format = '<I'
                intensity_unpack_format = '<H'
                pointcloud_unpack_format = '<Q'

                depth_unpack = struct.Struct(depth_unpack_format).unpack_from
                intensity_unpack = struct.Struct(intensity_unpack_format).unpack_from
                pointcloud_unpack = struct.Struct(pointcloud_unpack_format).unpack_from

                cols = self._scene.cols
                scene_depth_image = self._scene.depth_image
                scene_intensity_image = self._scene.intensity_image
                scene_pointcloud = self._scene.pointcloud

                for i in range(self._scene.cols):
                    idx = (row_num * self._scene.cols) + i

                    if not depth_disable:
                        # Unpack depth image
                        scene_depth_image[idx] = depth_unpack(packet_data, buffer_read)[0]
                        buffer_read += 4

                    if not intensity_disable:
                        # Unpack intensity image
                        scene_intensity_image[idx] = intensity_unpack(packet_data, buffer_read)[0]
                        buffer_read += 4 

                    # Unpack point cloud data
                    pcd_byte_data = pointcloud_unpack(packet_data, buffer_read)[0]
                    buffer_read += 8

                    x = pcd_byte_data & 0x1FFFFF
                    if x & 0x100000:
                        x -= 0x200000
                    y = (pcd_byte_data >> 21) & 0x1FFFFF
                    if y & 0x100000:
                        y -= 0x200000
                    z = (pcd_byte_data >> 42) & 0x1FFFFF
                    if z & 0x100000:
                        z -= 0x200000

                    scene_pointcloud[idx].x = x
                    scene_pointcloud[idx].y = y
                    scene_pointcloud[idx].z = z

                if row_num == 55:
                    self._shared_var_queue_data.put(self._scene)
            except queue.Empty:
                continue

    # Get the latest scene data
    def get_scene(self) -> Scene:
        try:
            return self._shared_var_queue_data.get(timeout=0.1)
        except queue.Empty:
            return None
