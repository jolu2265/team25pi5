import io
import logging
import socketserver
from threading import Condition, Thread
from http import server
from picamera2 import Picamera2
import cv2
import numpy as np

from multiprocessing import Manager, Process
from stereo_calc import compute_stereo_and_pose
import time

RPI5_PAGE = """\
<html>
  <head>
    <link rel="shortcut icon" href="https://www.protostax.com/cdn/shop/files/Sridhar_rajagop.png?crop=center&height=32&v=1613523611&width=32" type="image/x-icon">
    <style>
      .viewfinder {
        display: flex;
        flex-wrap: wrap;
      }
      .viewfinder-item {
        height: 50%;
        width:  50%;
        object-fit: contain;
      }
    </style>
    <script type="text/javascript" src="https://richtr.github.io/NoSleep.js/dist/NoSleep.min.js"></script>
  </head>
  <body>
    <input type="button" id="toggle" value="Wake Lock is disabled" />
    <script>
      var noSleep = new NoSleep();
      var wakeLockEnabled = false;
      var toggleEl = document.querySelector("#toggle");
      toggleEl.addEventListener('click', function() {
        if (!wakeLockEnabled) {
          noSleep.enable();
          wakeLockEnabled = true;
          toggleEl.value = "Wake Lock is enabled";
          document.body.style.backgroundColor = "black";
        } else {
          noSleep.disable();
          wakeLockEnabled = false;
          toggleEl.value = "Wake Lock is disabled";
          document.body.style.backgroundColor = "";
        }
      }, false);
    </script>
    <div class="viewfinder">
      <img class="viewfinder-item" src="leftstream.mjpg" />
      <img class="viewfinder-item" src="rightstream.mjpg" />
    </div>
  </body>
</html>
"""

# Initialize the ArUco dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250) # Can change marker size here
parameters = cv2.aruco.DetectorParameters_create()

'''
[[963.00972841   0.         396.65286966]
 [  0.         964.08509764 194.55687155]
 [  0.           0.           1.        ]]
 
 [[-1.27577416e+00  1.45309335e+01  2.69256843e-03  3.89030412e-02
  -8.63815273e+01]]
  
  [[-0.27, 0.05, 0.0001, -0.0005, -0.01]]
  
  [[ 0.06470456 -0.07532992  0.00215549  0.00390056  0.04023774]]
'''

# Dummy camera calibration values (replace with actual calibration if available)
camera_matrix = np.array([[964, 0, 397], [0, 964, 195], [0, 0, 1]]) # previous values: [600, 0, 320], [0, 600, 240], [0, 0, 1]
distortion_mtx =  np.array([[0.06470456, -0.07532992, 0.00215549, 0.00390056, 0.04023774]])
newcam_mtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_mtx, (146,146), 1, (146,146)) 
x1, y1, w1, h1 = roi
dist_coeffs = np.zeros((4, 1))

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

# NEW
manager = Manager()
left_data = manager.list()
left_coords = manager.list()
right_data = manager.list()
right_coords = manager.list()
stereo_proc = Process(target=compute_stereo_and_pose, args=(left_data, right_data, left_coords, right_coords))
stereo_proc.start()

# added data store for sending data
def detect_aruco_and_stream(picam, output, data_store, coords_store):
    marker_length = 0.14605  # Marker side length in meters (adjust as needed) WAS: 0.1524

    while True:
        frame = picam.capture_array()
        frame = cv2.undistort(frame, camera_matrix, distortion_mtx, None, newcam_mtx)
        # frame = frame[y1:y1+h1, x1:x1+w1]
        # frame = cv2.flip(frame, 0) # flip
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # print(corners)
        # (topLeft, topRight, bottomRight, bottomLeft) = corners
        
        '''
        # Convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        '''

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            for i, tvec in enumerate(tvecs):
                distance = np.linalg.norm(tvec)
                pose = tvec.flatten().tolist() # added
                # print(f"Marker ID: {ids[i][0]}, Distance: {distance:.2f} m, Pose: {tvec.flatten()}")
                
                # added
                # if data_store:
                data_store[:] = [(distance, pose)]
                coords_store[:] = corners

        # Encode frame as JPEG
        ret, jpeg = cv2.imencode('.jpg', frame)
        if ret:
            output.write(jpeg.tobytes())

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = RPI5_PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path in ['/stream.mjpg', '/leftstream.mjpg', '/rightstream.mjpg']:
            #output = outputleft if 'left' in self.path or 'stream' in self.path else outputright
            if 'left' in self.path: output = outputleft
            elif 'right' in self.path: output = outputright
            else: output = None
            if output:
                self.send_response(200)
                self.send_header('Age', 0)
                self.send_header('Cache-Control', 'no-cache, private')
                self.send_header('Pragma', 'no-cache')
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
                self.end_headers()
                try:
                    while True:
                        with output.condition:
                            output.condition.wait()
                            frame = output.frame
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', len(frame))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                except Exception as e:
                    logging.warning(f'Removed streaming client {self.client_address}: {e}')
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

# Initialize cameras and outputs
picam2left = None
picam2right = None
outputleft = None
outputright = None

try:
    picam2left = Picamera2(1)
    picam2left.configure(picam2left.create_video_configuration(main={"size": (640, 480)}))
    outputleft = StreamingOutput()
except Exception as e:
    print("Left camera error:", e)

try:
    picam2right = Picamera2(0)
    picam2right.configure(picam2right.create_video_configuration(main={"size": (640, 480)}))
    outputright = StreamingOutput()
except Exception as e:
    print("Right camera error:", e)

# Start ArUco detection threads
if picam2left:
	print("Starting left camera stream")
	picam2left.start()
	Thread(target=detect_aruco_and_stream, args=(picam2left, outputleft, left_data, left_coords), daemon=True).start()

if picam2right:
	print("Starting right camera stream")
	picam2right.start()
	Thread(target=detect_aruco_and_stream, args=(picam2right, outputright, right_data, right_coords), daemon=True).start()

# Start the streaming server
try:
    address = ('', 8000)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()
finally:
    if picam2left:
        picam2left.stop()
    if picam2right:
        picam2right.stop()
