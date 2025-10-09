# test_yolo.py
import cv2
from ultralytics import YOLO


def run_test():
    # Load the Hazmat model
    model_path = 'src/reseq_ros2/ml-ckpt/hazmat_detection.pt'
    print(f'Loading model from: {model_path}')
    try:
        model = YOLO(model_path)
        print('Model loaded successfully!')
    except Exception as e:
        print(f'--- FATAL ERROR: Model could not be loaded. ---')
        print(f'Error details: {e}')
        return

    # Load the test image
    image_path = '/home/kbd/Desktop/hazmat.jpeg'
    print(f'Loading image from: {image_path}')
    img = cv2.imread(image_path)
    if img is None:
        print(f'--- FATAL ERROR: Could not load image. Check the path. ---')
        return

    # Run inference
    print('Running model prediction...')
    try:
        results = model(img)
        print('Prediction completed.')
    except Exception as e:
        print(f'--- FATAL ERROR: Model failed during prediction. ---')
        print(f'Error details: {e}')
        return

    # Print results
    if not results:
        print('\n--- RESULT: The model ran but returned no results. ---')
        return

    found_something = False
    for result in results:
        if len(result.boxes) > 0:
            found_something = True
            print('\n--- SUCCESS: Detections Found! ---')
            print(result.boxes)

    if not found_something:
        print('\n--- RESULT: The model ran but did not detect anything in this image. ---')


if __name__ == '__main__':
    run_test()
