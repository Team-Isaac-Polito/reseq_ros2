class AprilTagDetector:
    def __init__(self, node):
        self.node = node

    def process_detections(self, msg, depth_image, create_detection_msg_func):
        """
        Processes detections from the apriltag_ros node and converts them
        into the custom reseq_interfaces/msg/Detection format.
        """
        detection_msgs = []
        for detection in msg.detections:
            center = detection.centre
            corners = detection.corners
            tag_id = detection.id
            depth_value = 0.0  # Default depth to 0.0

            x_min = min(corner.x for corner in corners)
            y_min = min(corner.y for corner in corners)
            x_max = max(corner.x for corner in corners)
            y_max = max(corner.y for corner in corners)

            mid_x = int(center.x)
            mid_y = int(center.y)

            # If a depth image IS available, try to get the real depth
            if depth_image is not None:
                if 0 <= mid_y < depth_image.shape[0] and 0 <= mid_x < depth_image.shape[1]:
                    depth_value = float(depth_image[mid_y, mid_x]) / 1e3  # convert mm to meters
                else:
                    self.node.get_logger().warn(
                        f'AprilTag center ({mid_x}, {mid_y}) is out of depth image bounds.'
                    )

            # Use the provided function to create the message
            det_msg = create_detection_msg_func(
                header=msg.header,
                det_type='apriltag',
                label=str(tag_id),
                conf=1.0,  # AprilTags don't have a confidence score, so we use 1.0
                x1=x_min,
                y1=y_min,
                x2=x_max,
                y2=y_max,
                depth_value=depth_value,
            )
            detection_msgs.append(det_msg)

        return detection_msgs
