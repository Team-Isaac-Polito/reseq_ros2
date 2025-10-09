import cv2


class MotionDetection:
    def __init__(self):
        # Initialize the background subtractor.
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows=True)
        self.first_image_received = False

        # Tunable parameters for processing.
        self.learning_rate = -1  # compute the learning rate based on the frame count
        self.erosion_iterations = 1
        self.dilation_iterations = 1
        self.close_iterations = 1
        self.moving_average_weight = 0.5
        self.activation_threshold = 127
        self.min_area = 500
        self.bx_thickness = 10

        self.debug_contours = False

    def process_image(self, frame):
        # Apply background subtraction.
        fgimg = self.bg_subtractor.apply(frame, learningRate=self.learning_rate)

        # Use an elliptical kernel to better close small gaps.
        kernel_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fgimg = cv2.morphologyEx(
            fgimg, cv2.MORPH_CLOSE, kernel_ellipse, iterations=self.close_iterations
        )
        fgimg = cv2.erode(fgimg, kernel_ellipse, iterations=self.erosion_iterations)
        fgimg = cv2.dilate(fgimg, kernel_ellipse, iterations=self.dilation_iterations)

        # Create an accumulated moving average to reduce noise.
        if not self.first_image_received:
            self.accumulated_image = fgimg.copy()
            self.first_image_received = True
        else:
            cv2.addWeighted(
                self.accumulated_image,
                1 - self.moving_average_weight,
                fgimg,
                self.moving_average_weight,
                0,
                self.accumulated_image,
            )

        # Threshold the accumulated image to get a binary mask.
        _, thresholded = cv2.threshold(
            self.accumulated_image, self.activation_threshold, 255, cv2.THRESH_BINARY
        )

        # Find contours on the processed mask.
        contours, _ = cv2.findContours(
            thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        if contours:
            if self.debug_contours:
                cv2.drawContours(frame, contours, -1, (0, 0, 255), 2)

            # Draw the bounding box for each individual contour.
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w * h >= self.min_area:
                    cv2.rectangle(
                        frame, (x, y), (x + w, y + h), (0, 255, 0), self.bx_thickness
                    )

        return frame


if __name__ == '__main__':
    md = MotionDetection()
    cap = cv2.VideoCapture(0)  # Capture video from webcam

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        processed_frame = md.process_image(frame)

        cv2.imshow('Motion Detection', processed_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
