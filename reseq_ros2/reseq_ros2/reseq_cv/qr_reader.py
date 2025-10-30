import cv2
from ament_index_python.packages import get_package_share_directory

# pip install opencv-contrib-python

share_folder = get_package_share_directory('reseq_ros2')
models_path = f'{share_folder}/ml-ckpt'

model_proto = f'{models_path}/detect.prototxt'
model_weights = f'{models_path}/detect.caffemodel'
sr_proto = f'{models_path}/sr.prototxt'
sr_weights = f'{models_path}/sr.caffemodel'

# Start WeChat QRCode detector
detector = cv2.wechat_qrcode_WeChatQRCode(model_proto, model_weights, sr_proto, sr_weights)


def process_qr_codes(image):
    """
    Detects QR codes in an image and returns the annotated image along with detection data.
    """
    qr_texts, points = detector.detectAndDecode(image)

    detections = []
    if qr_texts:
        for text, pts in zip(qr_texts, points):
            pts = pts.astype(int)  # corners (4x2)
            detection_data = {'text': text, 'bbox': pts}
            detections.append(detection_data)

    return image, detections
