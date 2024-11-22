import cv2
import numpy as np
import time
from colorama import Fore, Style


class Vision:
    def __init__(self, target_height=10, fps=10, threshold=128):
        """
        Initialize the Vision class with default settings.
        """
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            raise Exception("Error: Unable to access the camera")

        self.target_height = target_height
        self.fps = fps
        self.threshold = threshold
        self.frame_delay = 1 / fps
        self.matrix = None
        self.image = None
        self.goal = None
        self.start = None
        self.angle = None

    def update(self):
        """
        Capture a new frame and update the matrix with colors for start and goal.
        """
        start_time = time.time()
        ret, frame = self.camera.read()
        if not ret:
            raise Exception("Error: Unable to capture image from the camera")

        self.image = frame
        resized_frame = self._resize_image(frame)
        self.matrix = self._generate_matrix(resized_frame)
        hsv_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)

        self.goal = self._find_color(hsv_frame, [([0, 70, 50], [10, 255, 255]), ([170, 70, 50], [180, 255, 255])])
        self.start, self.angle = self._find_start(hsv_frame)

        #self._update_matrix_with_colors()
        time.sleep(max(0, self.frame_delay - (time.time() - start_time)))

    def _resize_image(self, frame):
        """
        Resize the input frame while keeping the aspect ratio.
        """
        height, width, _ = frame.shape
        target_width = int(self.target_height * (width / height))
        return cv2.resize(frame, (target_width, self.target_height), interpolation=cv2.INTER_AREA)

    def _generate_matrix(self, resized_frame):
        """
        Generate a binary matrix for black and white representation.
        """
        gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
        _, binary_mask = cv2.threshold(gray_frame, self.threshold, 255, cv2.THRESH_BINARY)
        return np.where(binary_mask == 255, 0, -1)

    def _find_color(self, hsv_frame, ranges):
        """
        Find the largest contour of a given color range.
        """
        mask = sum(cv2.inRange(hsv_frame, np.array(lower), np.array(upper)) for lower, upper in ranges)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            moments = cv2.moments(largest)
            if moments["m00"] != 0:
                cx, cy = int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])
                return (cx, cy)
        return None

    def _find_start(self, hsv_frame):
        """
        Detect the start (green and blue regions) and compute the orientation angle.

        :return: Tuple (start position, angle in degrees) or (None, None) if not found.
        """
        lower_blue = np.array([85, 50, 50])
        upper_blue = np.array([135, 255, 255])
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])

        mask_blue = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)

        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_blue and contours_green:
            largest_blue = max(contours_blue, key=cv2.contourArea)
            largest_green = max(contours_green, key=cv2.contourArea)

            moments_blue = cv2.moments(largest_blue)
            moments_green = cv2.moments(largest_green)

            if moments_blue["m00"] != 0 and moments_green["m00"] != 0:
                cx_blue, cy_blue = int(moments_blue["m10"] / moments_blue["m00"]), int(moments_blue["m01"] / moments_blue["m00"])
                cx_green, cy_green = int(moments_green["m10"] / moments_green["m00"]), int(moments_green["m01"] / moments_green["m00"])

                dx = cx_blue - cx_green
                dy = cy_blue - cy_green
                angle = -(np.arctan2(dy, dx) + 2 * np.pi) % (2 * np.pi)  # Angle in radians normalized to [0, 2π)
                start_x = (cx_blue + cx_green) // 2
                start_y = (cy_blue + cy_green) // 2

                return (start_x, start_y), angle

        return None, None

    def _update_matrix_with_colors(self):
        """
        Update the matrix with specific values for start and goal.
        """
        # Reset the matrix areas corresponding to start and goal
        self.matrix[self.matrix == -2] = 0  # Reset blue regions
        self.matrix[self.matrix == -3] = 0  # Reset red regions

        # Update the matrix with new start and goal positions
        if self.start:
            x, y = self.start
            self.matrix[y, x] = -2  # Start marked as -2
        if self.goal:
            x, y = self.goal
            self.matrix[y, x] = -3  # Goal marked as -3

    def get_matrix(self):
        """
        Get the current color matrix. """
        if self.matrix is None:
            raise Exception("Matrix has not been initialized. Call update() first.")
        return self.matrix

    def get_image(self):
        """
        Get the current image (frame).
        """
        if self.image is None:
            raise Exception("Image has not been initialized. Call update() first.")
        return self.image

    def get_goal(self):
        """
        Get the position of the goal (center of the largest red area).
        """
        return self.goal

    def get_start(self):
        """
        Get the position of the start (center of the detected region).
        """
        return self.start

    def get_angle(self):
        """
        Get the orientation angle of the detected start.
        """
        return self.angle

    def display_matrix(self):
        """
        Print the matrix with consistent spacing and colors in the console.
        """
        result = ""
        for row in self.matrix:
            result += "".join(
                f"{Fore.BLUE}{x:3} " if x == -2 else
                f"{Fore.RED}{x:3} " if x == -3 else
                f"{Fore.MAGENTA}{x:3} " if x == -1 else
                f"{Fore.WHITE}{x:3} "
                for x in row
            ) + "\n"
        print(result + Style.RESET_ALL)

    def display_info(self):
        """
        Display the detected goal and start positions with angle.
        """
        print(f"Goal position: {self.get_goal()}" if self.get_goal() else "Goal not detected")
        if self.get_start():
            print(f"Start position: {self.get_start()}, Angle: {self.get_angle():.2f}°")
        else:
            print("Start not detected")
        cv2.imshow("Captured Image", self.get_image())

    def release(self):
        """
        Release the camera resource.
        """
        self.camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        vision = Vision(target_height=20, fps=20, threshold=128)

        while True:
            vision.update()
            vision.display_matrix()
            vision.display_info()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        vision.release()