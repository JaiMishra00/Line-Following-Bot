import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from flask import Flask, Response, render_template_string, jsonify
import threading
import time
import logging

# 1. Mute Flask/Werkzeug logs to keep the terminal clean
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)
last_detection = "NONE"
mission_start_time = time.time()
timer_running = True

@app.route('/')
def index():
    return render_template_string('''
        <html>
          <head>
            <title>Rover Mission Control</title>
            <style>
              body { background:#000; color:#0f0; text-align:center; font-family:monospace; margin:0; padding:20px; }
              .stream-container { position: relative; display: inline-block; width: 85%; border: 2px solid #333; }
              #mission-clock { 
                position: absolute; top: 15px; left: 15px; 
                background: rgba(0, 0, 0, 0.6); color: #0f0; 
                padding: 8px 15px; font-size: 22px; border: 1px solid #0f0;
                backdrop-filter: blur(5px); z-index: 10;
              }
              .controls { margin-top: 20px; }
              .stop-btn {
                background: #440000; color: #ff4444; border: 1px solid #ff4444; 
                padding: 12px 30px; cursor: pointer; font-family: monospace; font-weight: bold;
                text-transform: uppercase;
              }
              .stop-btn:hover { background: #ff4444; color: #000; }
            </style>
          </head>
          <body>
            <h1>🛰️ ROVER MISSION CONTROL</h1>
            <div class="stream-container">
              <div id="mission-clock">MISSION TIME: <span id="time-display">00:00</span></div>
              <img src="/video_feed" style="width:100%; display:block; border: 1px solid #0f0;">
            </div>
            <div class="controls">
                <button class="stop-btn" onclick="stopMissionClock()">🛑 STOP MISSION CLOCK</button>
            </div>

            <script>
              function formatTime(seconds) {
                let mins = Math.floor(seconds / 60);
                let secs = seconds % 60;
                return (mins < 10 ? '0' : '') + mins + ":" + (secs < 10 ? '0' : '') + secs;
              }

              function syncClock() {
                fetch('/timer_status')
                  .then(response => response.json())
                  .then(data => {
                    let display = document.getElementById('time-display');
                    let clockBox = document.getElementById('mission-clock');
                    if (!data.active) {
                        clockBox.style.borderColor = "#ff4444";
                        clockBox.style.color = "#ff4444";
                        return; 
                    }
                    display.innerText = formatTime(data.elapsed);
                  })
                  .catch(err => console.error("Sync error:", err));
              }
              function stopMissionClock() {
                fetch('/stop_timer', {method: 'POST'});
              }
              setInterval(syncClock, 1000);
            </script>
          </body>
        </html>
    ''')

def gen_frames():
    global last_detection
    
    # Using GStreamer pipeline to read from the mirrored loopback device (/dev/video17)
    # This is often more stable when multiple nodes are accessing virtual devices.
    gst_pipeline = (
        "v4l2src device=/dev/video17 ! "
        "video/x-raw, format=BGR ! "
        "videoconvert ! appsink"
    )
    
    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    
    # Fallback to standard index if GStreamer backend isn't available
    if not cap.isOpened():
        cap = cv2.VideoCapture(17)

    while True:
        success, frame = cap.read()
        if not success:
            break

        # Orientation correction
        frame = cv2.flip(frame, -1)

        # Overlay Mission Data
        text = f"LAST DETECTED: {last_detection}"
        cv2.putText(frame, text, (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4) 
        cv2.putText(frame, text, (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    
    cap.release()

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/timer_status')
def timer_status():
    global mission_start_time, timer_running
    elapsed = 0
    if timer_running:
        elapsed = int(time.time() - mission_start_time)
    return jsonify({"elapsed": elapsed, "active": timer_running})

@app.route('/stop_timer', methods=['POST'])
def stop_timer():
    global timer_running
    timer_running = False
    return '', 204

class StreamerNode(Node):
    def __init__(self):
        super().__init__('streamer_node')
        self.subscription = self.create_subscription(String, 'detection/label', self.callback, 10)
        self.get_logger().info("Mission Control Streamer Active on Port 8080")

    def callback(self, msg):
        global last_detection
        if msg.data.lower() != "none":
            last_detection = msg.data.upper()

def main(args=None):
    rclpy.init(args=args)
    node = StreamerNode()
    
    # Launch Flask in background
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=8080, threaded=True, use_reloader=False), daemon=True).start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
