import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # <--- Added for detection topic
import cv2
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string
import threading

import os
import sys
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

# Flask setup for remote viewing
app = Flask(__name__)
latest_frame = None

@app.route('/')
def index():
    return render_template_string('<h1>Rover Vision Feed</h1><img src="/video_feed" width="640">')

def gen_frames():
    global latest_frame
    while True:
        if latest_frame is not None:
            ret, buffer = cv2.imencode('.jpg', latest_frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b' \r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

class ImageScannerNode(Node):
    def __init__(self):
        super().__init__('image_scanner_node')
        self.br = CvBridge()
        
        # Publishers
        self.publisher_image = self.create_publisher(Image, 'camera/annotated_image', 10)
        self.publisher_detection = self.create_publisher(String, 'detection/label', 10) # <--- New Topic

        # Model Loading
        try:
            package_share = get_package_share_directory('robot_control')
            model_path = os.path.join(package_share, 'unbiased_best_ncnn_model')            
            self.get_logger().info(f"--- Loading NCNN Brain: {model_path} ---")
            self.model = YOLO(model_path)
        except:
            self.get_logger().warn("NCNN model not found in share, trying local .pt...")
            self.model = YOLO("/root/ros2_robot_ws/src/robot_control/robot_control/unbiased_best.pt", task='classify')

        self.cap = cv2.VideoCapture(17)
        self.timer = self.create_timer(0.3, self.timer_callback) 
        self.CONF_THRESHOLD = 75.0

    def timer_callback(self):
        global latest_frame
        ret, frame = self.cap.read()
        if not ret: return
        
        frame = cv2.flip(frame, -1)
        results = self.model(frame, verbose=False, imgsz=160, half=True)
        r = results[0]

        annotated_frame = frame.copy()
        status_text = "🔎 Scanning..."
        color = (0, 0, 255) 

        # Create String message
        detection_msg = String()

        if hasattr(r, 'probs') and r.probs is not None:
            top_idx = r.probs.top1
            conf = r.probs.top1conf.item() * 100
            name = r.names[top_idx]

            if conf >= self.CONF_THRESHOLD:
                status_text = f"[{name}] - {conf:.1f}%"
                color = (0, 255, 0)
                
                # --- NEW: Publish Detection Label ---
                detection_msg.data = name
                self.publisher_detection.publish(detection_msg)
                
                sys.stdout.write(f"\r🚀 DETECTED: {status_text}                ")               
                sys.stdout.flush()
            else:
                detection_msg.data = "none" # Clear status
                self.publisher_detection.publish(detection_msg)
                sys.stdout.write(f"\r🔎 Scanning... (Best: {name} {conf:.0f}%)    ")
                sys.stdout.flush()

        cv2.putText(annotated_frame, status_text, (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)

        latest_frame = annotated_frame
        self.publisher_image.publish(self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8"))

def run_flask():
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    node = ImageScannerNode()
    threading.Thread(target=run_flask, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
