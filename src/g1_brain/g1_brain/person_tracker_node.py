import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__("person_tracker_node")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("show_debug", True)

        model_path = self.get_parameter("model_path").value
        self.conf = float(self.get_parameter("conf_threshold").value)
        color_topic = self.get_parameter("color_topic").value
        self.show_debug = bool(self.get_parameter("show_debug").value)

        self.model = YOLO(model_path)
        self.sub = self.create_subscription(Image, color_topic, self.image_cb, 10)
        self.get_logger().info(
            f"PersonTrackerNode démarré ({model_path}, conf={self.conf}) sur {color_topic}"
        )

    def image_cb(self, msg: Image):
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"Encodage image non géré : {msg.encoding}")
            return

        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        ).copy()
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        results = self.model.track(
            frame, classes=[0], conf=self.conf, persist=True, verbose=False
        )
        annotated = results[0].plot()

        boxes = results[0].boxes
        if boxes is not None and boxes.id is not None:
            center_img_x = msg.width // 2
            for xyxy, tid, conf in zip(
                boxes.xyxy.cpu().numpy(),
                boxes.id.int().cpu().numpy(),
                boxes.conf.cpu().numpy(),
            ):
                x1, y1, x2, y2 = xyxy.astype(int)
                cx = (x1 + x2) // 2
                err_x = cx - center_img_x
                self.get_logger().info(
                    f"track_id={tid} | conf={conf:.2f} | cx={cx}px | err_x={err_x:+d}px"
                )

        if self.show_debug:
            cv2.imshow("G1 Person Tracker (q pour quitter)", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        if node.show_debug:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
