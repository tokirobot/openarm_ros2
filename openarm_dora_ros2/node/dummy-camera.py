import dora
import cv2
import numpy as np
import pyarrow as pa
import time


def main():
    node = dora.Node()
    W, H = 960, 600

    # Initialize variables for color and timing
    current_color = [120, 0, 255]  # Initial BGR color
    last_update_time = time.time()

    while True:
        now = time.time()

        # Update color every 1.0 second
        if now - last_update_time >= 1.0:
            # Generate a new random BGR color
            current_color = np.random.randint(0, 256, size=3).tolist()
            last_update_time = now

        # Create the frame with the current color
        frame = np.full((H, W, 3), current_color, dtype=np.uint8)

        # Encode frame as JPEG
        success, encoded_img = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])

        if success:
            # Send as a flat uint8 array (ravel() ensures correct flattening)
            node.send_output("image", pa.array(encoded_img.ravel()))

        # Keep the loop running at roughly 30 FPS
        time.sleep(1/30.0)


if __name__ == "__main__":
    main()
