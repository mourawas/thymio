import cv2
import numpy as np
import time

class Vision:
    def __init__(self, target_height=10, fps=10):
        """
        Initialize the Vision class with default settings.
        
        :param target_height: Height of the resulting matrix (default is 10).
        :param fps: Number of frames per second to capture (default is 10).
        """
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            raise Exception("Error: Unable to access the camera")
        
        self.target_height = target_height
        self.fps = fps
        self.frame_delay = 1 / fps  # Time delay between frames
        self.matrix = None
        self.image = None

    def update(self):
        """
        Capture a new frame from the camera and update the image and matrix.
        """
        ret, frame = self.camera.read()
        if not ret:
            raise Exception("Error: Unable to capture image from the camera")
        
        # Store the current image (frame)
        self.image = frame

        # Resize the image while keeping the aspect ratio
        height, width, _ = frame.shape
        aspect_ratio = width / height
        target_width = int(self.target_height * aspect_ratio)
        resized_frame = cv2.resize(frame, (target_width, self.target_height), interpolation=cv2.INTER_AREA)

        # Convert the image to HSV for color detection
        hsv_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for colors
        # Black
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 30])
        mask_black = cv2.inRange(hsv_frame, lower_black, upper_black)

        # White
        # White (more relaxed thresholds)
        lower_white = np.array([0, 0, 180])  # Increase the lower value to 180 for lighter shades
        upper_white = np.array([180, 60, 255])  # Allow slightly more saturation and value range
        mask_white = cv2.inRange(hsv_frame, lower_white, upper_white)

        # Red (two ranges for the HSV circle)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv_frame, lower_red1, upper_red1) | cv2.inRange(hsv_frame, lower_red2, upper_red2)

        # Blue
        lower_blue = np.array([90, 100, 50])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv_frame, lower_blue, upper_blue)

        # Create the color matrix
        self.matrix = np.zeros((self.target_height, target_width), dtype=int)
        self.matrix[mask_white > 0] = 0  # White
        self.matrix[mask_black > 0] = -1  # Black
        self.matrix[mask_red > 0] = -2    # Red
        self.matrix[mask_blue > 0] = -3   # Blue

    def get_matrix(self):
        """
        Get the current color matrix.
        
        :return: Current color matrix (as numpy array).
        """
        if self.matrix is None:
            raise Exception("Matrix has not been initialized. Call update() first.")
        return self.matrix

    def get_image(self):
        """
        Get the current image (frame).
        
        :return: Current image.
        """
        if self.image is None:
            raise Exception("Image has not been initialized. Call update() first.")
        return self.image

    def release(self):
        """
        Release the camera resource.
        """
        self.camera.release()
        cv2.destroyAllWindows()


# Main function to test the Vision class
# if __name__ == "__main__":
#     try:
#         vision = Vision(target_height=20, fps=1)  # Set FPS to 10
        
#         while True:
#             start_time = time.time()
            
#             # Update the vision system
#             vision.update()

#             # Get the current matrix and display it
#             matrix = vision.get_matrix()
#             print("Color Matrix:")
#             print(matrix)

#             # Display the current image
#             cv2.imshow("Captured Image", vision.get_image())

#             # Wait to match the desired FPS
#             elapsed_time = time.time() - start_time
#             if elapsed_time < vision.frame_delay:
#                 time.sleep(vision.frame_delay - elapsed_time)

#             # Quit with 'q'
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
#     finally:
#         vision.release()