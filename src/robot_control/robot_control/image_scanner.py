import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string
import threading
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

# Flask app setup
app = Flask(__name__)
latest_frame = None

@app.route('/')
def index():
    return render_template_string('<h1>ROS 2 Rover Brain: Live Feed</h1><img src="/video_feed" width="640">')

def gen_frames():
    global latest_frame
    while True:
        if latest_frame is not None:
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpg', latest_frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

class CameraMLNode(Node):
    def __init__(self):
        super().__init__('camera_ml_node')
        self.publisher_ = self.create_publisher(Image, 'camera/raw_image', 10)
        
        # 1. Load the model using the absolute path or package share
        # Change 'unbiased_best.pt' to match your exact filename
        try:
            package_share_directory = get_package_share_directory('robot_control')
            model_path = os.path.join(package_share_directory, 'unbiased_best.pt')
            self.get_logger().info(f"--- Loading Brain from: {model_path} ---")
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Could not load model: {e}")
            # Fallback to local path if package share fails
            self.model = YOLO("/root/ros2_robot_ws/src/robot_control/robot_control/unbiased_best_ncnn_model")

        self.timer = self.create_timer(0.2, self.timer_callback) # 5Hz
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video device!")

    def timer_callback(self):
        global latest_frame
        ret, frame = self.cap.read()
        if ret:
            # Run Inference
            # imgsz=320 is critical for RPi 4 speed
            results = self.model(frame, verbose=False, imgsz=320)
            
            annotated_frame = frame.copy()

            for result in results:
                # Handle Classification Models (Predicting the whole image)
                if hasattr(result, 'probs') and result.probs is not None:
                    top_class_index = result.probs.top1
                    top_class_name = result.names[top_class_index]
                    conf = result.probs.top1conf.item() * 100
                    
                    if conf > 50:
                        label = f"Brain: {top_class_name} ({conf:.1f}%)"
                        print(f"🚀 {label}")
                        # Draw text on the frame for the web stream
                        cv2.putText(annotated_frame, label, (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Handle Object Detection Models (Drawing boxes)
                elif hasattr(result, 'boxes') and result.boxes is not None:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cls = int(box.cls[0])
                        name = result.names[cls]
                        conf = box.conf[0].item() * 100
                        
                        if conf > 50:
                            print(f"🎯 Detected: {name} ({conf:.1f}%)")
                            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                            cv2.putText(annotated_frame, f"{name} {conf:.0f}%", (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Update Flask and ROS
            latest_frame = annotated_frame
            msg = self.br.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.publisher_.publish(msg)

def run_flask():
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    node = CameraMLNode()
    
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()

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
