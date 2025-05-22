from pyzbar.pyzbar import decode
import cv2


def process_qr_codes(frame):
    qr_codes = decode(frame)
    for qr in qr_codes:
        # Get bounding box and data
        x, y, w, h = qr.rect
        qr_data = qr.data.decode('utf-8')

        # Draw bounding box and display data
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            frame,
            f'QR: {qr_data}',
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
        )
        print(f'QR Code Data: {qr_data}')
    return frame
