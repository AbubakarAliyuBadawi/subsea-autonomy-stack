#!/usr/bin/env python3
"""ROS 2 node for YOLO fish detection."""

import json
import os
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String


class FishDetectionNode(Node):
    def __init__(self):
        super().__init__('fish_detection_node')

        self.declare_parameter('camera_topic', '/blueye/camera_1/image_raw')
        self.declare_parameter('weights_path', 'merge_yolov4.weights')
        self.declare_parameter('config_path', 'yolov4.cfg')
        self.declare_parameter('confidence_threshold', 0.55)
        self.declare_parameter('nms_threshold', 0.7)
        self.declare_parameter('publish_annotated', False)
        self.declare_parameter('display_window', False)

        camera_topic = self.get_parameter('camera_topic').value
        weights_path = self._resolve_model_file(
            self.get_parameter('weights_path').value,
            subdir='weights',
        )
        config_path = self._resolve_model_file(
            self.get_parameter('config_path').value,
            subdir='configs',
        )
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        self.display_window = self.get_parameter('display_window').value

        self.get_logger().info(f'Loading YOLO config: {config_path}')
        self.get_logger().info(f'Loading YOLO weights: {weights_path}')
        self.net = cv2.dnn.readNetFromDarknet(str(config_path), str(weights_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, camera_topic, self.image_callback, 10)
        self.fish_count_pub = self.create_publisher(Int32, '/fish_detection/count', 10)
        self.detections_pub = self.create_publisher(String, '/fish_detection/detections', 10)

        if self.publish_annotated:
            self.annotated_image_pub = self.create_publisher(
                Image,
                '/fish_detection/annotated_image',
                10,
            )

        self.frames_processed = 0
        self.total_detections = 0

        self.get_logger().info('Fish Detection Node initialized')
        self.get_logger().info(f'Subscribing to: {camera_topic}')
        self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'NMS threshold: {self.nms_threshold}')

    def _resolve_model_file(self, path_value, subdir):
        requested_path = Path(os.path.expanduser(path_value))
        if requested_path.is_absolute() and requested_path.exists():
            return requested_path
        if requested_path.exists():
            return requested_path

        model_dir = os.environ.get('FISH_DETECTION_MODEL_DIR')
        candidates = []
        if model_dir:
            candidates.append(Path(model_dir) / subdir / requested_path.name)
            candidates.append(Path(model_dir) / requested_path.name)

        package_path = Path(__file__).resolve()
        for parent in package_path.parents:
            candidates.append(parent / 'models' / 'fish_detection' / subdir / requested_path.name)
            candidates.append(parent / 'fish_model' / requested_path)
            candidates.append(parent / 'fish_model' / subdir / requested_path.name)

        for candidate in candidates:
            if candidate.exists():
                return candidate

        candidate_text = '\n  '.join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(
            f'Could not find {requested_path}. Checked:\n  {candidate_text}'
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = cv_image.shape[:2]

            blob = cv2.dnn.blobFromImage(
                cv_image,
                1 / 255.0,
                (416, 416),
                swapRB=True,
                crop=False,
            )
            self.net.setInput(blob)
            outputs = self.net.forward(self.output_layers)

            boxes = []
            confidences = []
            for output in outputs:
                for detection in output:
                    confidence = detection[5:][0]
                    if confidence <= self.conf_threshold:
                        continue

                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    box_width = int(detection[2] * width)
                    box_height = int(detection[3] * height)
                    x = int(center_x - box_width / 2)
                    y = int(center_y - box_height / 2)

                    boxes.append([x, y, box_width, box_height])
                    confidences.append(float(confidence))

            indices = cv2.dnn.NMSBoxes(
                boxes,
                confidences,
                self.conf_threshold,
                self.nms_threshold,
            )
            fish_count = len(indices) if len(indices) > 0 else 0

            self.frames_processed += 1
            self.total_detections += fish_count

            count_msg = Int32()
            count_msg.data = fish_count
            self.fish_count_pub.publish(count_msg)

            detections = []
            if len(indices) > 0:
                for i in indices.flatten():
                    x, y, box_width, box_height = boxes[i]
                    detections.append({
                        'bbox': [int(x), int(y), int(box_width), int(box_height)],
                        'confidence': float(confidences[i]),
                    })

            detections_msg = String()
            detections_msg.data = json.dumps({
                'timestamp': self.get_clock().now().nanoseconds,
                'fish_count': fish_count,
                'detections': detections,
            })
            self.detections_pub.publish(detections_msg)

            if self.publish_annotated or self.display_window:
                self._draw_detections(cv_image, boxes, confidences, indices, fish_count)

            if self.display_window:
                cv2.imshow('Fish Detection - Press Q to quit', cv_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info('User pressed Q - shutting down')
                    rclpy.shutdown()

            if self.publish_annotated:
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_image_pub.publish(annotated_msg)

            if self.frames_processed % 30 == 0:
                avg_fish = self.total_detections / self.frames_processed
                self.get_logger().info(
                    f'Processed {self.frames_processed} frames | '
                    f'Current: {fish_count} fish | '
                    f'Avg: {avg_fish:.2f} fish/frame'
                )
        except Exception as exc:
            self.get_logger().error(f'Error processing image: {exc}')

    def _draw_detections(self, cv_image, boxes, confidences, indices, fish_count):
        if fish_count <= 0:
            return

        for i in indices.flatten():
            x, y, box_width, box_height = boxes[i]
            confidence = confidences[i]
            cv2.rectangle(cv_image, (x, y), (x + box_width, y + box_height), (0, 255, 0), 2)
            cv2.putText(
                cv_image,
                f'Fish: {confidence:.2f}',
                (x, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        cv2.putText(
            cv_image,
            f'Fish: {fish_count}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )


def main(args=None):
    rclpy.init(args=args)
    node = FishDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

        if node.frames_processed > 0:
            avg_fish = node.total_detections / node.frames_processed
            node.get_logger().info('=' * 60)
            node.get_logger().info('Final Statistics:')
            node.get_logger().info(f'  Total frames: {node.frames_processed}')
            node.get_logger().info(f'  Total detections: {node.total_detections}')
            node.get_logger().info(f'  Average fish/frame: {avg_fish:.2f}')
            node.get_logger().info('=' * 60)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
