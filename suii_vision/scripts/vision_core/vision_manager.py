import socket
import time
import select
import struct
import json
import cv2
import pyrealsense2 as rs
import numpy

from post_processing_v3 import PostProcessing
from yolo import Yolo

class VisionManager(object):
    def __init__(self, port):
        #Build Socket
        self.connections = []
        self.connections_limit = 2
        self.shock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.shock.bind(("0.0.0.0", port))
        self.shock.setblocking(True)
        self.shock.listen(10)
        self.connections.append(self.shock)

        #Start Stream
        self.dev = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.dev.start(config)

        self.signature_dict = {
            0x00: self.scan_all
        }

        #Start modules
        self.post_processing = PostProcessing()
        self.yolo = Yolo(None)
        self.yolo.load_model()

        while True:
            try:
                self.spin_socket()
            except KeyboardInterrupt:
                self.dev.close()
                self.shock.close()

    def spin_socket(self):
        rs, _, _ = select.select(self.connections, [], [])
        for sock in rs:
            if sock == self.shock:
                sockfd, _ = self.shock.accept()
                if len(self.connections) < self.connections_limit:
                    self.connections.append(sockfd)
                    print("INFO: New client added")
                else:
                    sockfd.close()
                    print("INFO: Refused client, limit reached")       
            else:
                self.process_socket(sock)
    
    def process_socket(self, sock):
        try:
            hr = sock.recv(8)
            h = struct.unpack("BBI", hr)
            rb = ""
            while len(rb) < h[2]:
                rb += self.shock.recv(h[2])

            if h[1] in self.signature_dict:
                write_back = self.signature_dict[h[1]](rb)
            else:
                b = "Invalid function call"
                write_back = struct.pack('BBI', 0xFF, h[1], len(b))
                write_back += bytes(b, 'ascii')
            sock.sendall(write_back)

        except Exception as e:
            print("INFO: Client Disconnected")
            sock.close()
            self.connections.remove(sock)

    def scan_all(self, data):
        mf = self.dev.wait_for_frames()
        rc = mf.get_color_frame()
        img_raw = numpy.asarray(rc.get_data())
        img_edges, img_undistord = self.post_processing.filter_img(img_raw,0,0,0,0)
        bb = self.yolo.run(img_undistord, False)
        ra = []
        for x in bb:
            self.post_processing.build_center(x[0], x[1], img_raw, False)
            dt = self.post_processing.build_view()
            self.post_processing.reset_view()

            if len(dt) > 0:
                rb = {'x': dt[0][1], 'y':dt[0][2], 'z':dt[0][3], 'n':x[0], 't':1}
            else:
                rb = {'x':0.0, 'y':0.0, 'z':0.0, 'n':x[0], 't':0}
            ra.append(rb)

        js = json.dumps(ra)
        h = struct.pack('BBI', 0x01, 0x00, len(js))
        return h + bytes(js, 'ascii')

if __name__ == "__main__":
    v = VisionManager(9001)
