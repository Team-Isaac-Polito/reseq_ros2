"""Convert ROS camera images to small JPEGs for the models and the API."""

import io

from PIL import Image

# ROS encoding: (PIL raw decoder, PIL image mode, bytes per pixel)
CAMERA_ENCODINGS = {
    'rgb8': ('RGB', 'RGB', 3),
    'bgr8': ('BGR', 'RGB', 3),
    'rgba8': ('RGBA', 'RGBA', 4),
    'bgra8': ('BGRA', 'RGBA', 4),
    'mono8': ('L', 'L', 1),
}
MAX_PIXELS = 4096 * 4096
JPEG_SIZE = (640, 480)


def check_image(message):
    """Raise ValueError if the image cannot be converted."""
    if message.encoding not in CAMERA_ENCODINGS:
        raise ValueError(f'Unsupported camera encoding: {message.encoding}')
    _, _, channels = CAMERA_ENCODINGS[message.encoding]
    width = message.width
    height = message.height
    if width <= 0 or height <= 0 or width * height > MAX_PIXELS:
        raise ValueError('Invalid image dimensions')
    if message.step < width * channels:
        raise ValueError('Truncated camera image')
    if len(message.data) < message.step * height:
        raise ValueError('Truncated camera image')


def to_jpeg(message):
    """Return (jpeg bytes, (width, height)) for a sensor_msgs/Image."""
    check_image(message)
    decoder, image_mode, _ = CAMERA_ENCODINGS[message.encoding]
    size = (message.width, message.height)
    image = Image.frombytes(
        image_mode, size, bytes(message.data), 'raw', decoder, message.step, 1
    )
    image = image.convert('RGB')
    # Shrink in place, keeping the aspect ratio.
    image.thumbnail(JPEG_SIZE)
    result = io.BytesIO()
    image.save(result, format='JPEG', quality=85)
    return result.getvalue(), image.size
