import cv2


def process_qr_codes(image):
    """
    Detects QR codes in an image and returns the annotated image along with detection data.
    """
    qr_decoder = cv2.QRCodeDetector()
    data, bbox, _ = qr_decoder.detectAndDecode(image)

    detections = []
    # If a QR code is detected, bbox will not be None
    if bbox is not None and data:
        points = bbox[0].astype(int)
        detection_data = {'text': data, 'bbox': points}
        detections.append(detection_data)

    return image, detections
