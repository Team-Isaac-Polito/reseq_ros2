from apriltag import apriltag
import cv2

def process_apriltags(frame):
    detector = apriltag("tagStandard41h12")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert frame to grayscale
    detections = detector.detect(gray)

    for detection in detections:
        # Access tag corners and tag ID
        corners = detection.get('lb-rb-rt-lt', None)
        tag_id = detection.get('id', 'Unknown')

        if corners is not None:
            # Unpack the corners, which are stored as a 4x2 numpy array
            ptA, ptB, ptC, ptD = map(tuple, map(lambda pt: tuple(map(int, pt)), corners))

            # Draw bounding box for the AprilTag
            cv2.line(frame, ptA, ptB, (0, 0, 255), 2)
            cv2.line(frame, ptB, ptC, (0, 0, 255), 2)
            cv2.line(frame, ptC, ptD, (0, 0, 255), 2)
            cv2.line(frame, ptD, ptA, (0, 0, 255), 2)

            # Display the tag ID on the frame
            cv2.putText(frame, f'AT_ID: {tag_id}', (ptA[0], ptA[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            print(f"AprilTag Detected: ID {tag_id}")

            # Display additional tag information (optional)
            center_text = f'Center: ({int(detection["center"][0])}, {int(detection["center"][1])})'
            margin_text = f'Margin: {detection["margin"]:.2f}'

            # Display the center and margin info below the AT_ID
            cv2.putText(frame, center_text, (int(detection["center"][0]) + 10, int(detection["center"][1]) + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(frame, margin_text, (int(detection["center"][0]) + 10, int(detection["center"][1]) + 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    return frame