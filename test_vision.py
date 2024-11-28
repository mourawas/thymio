import cv2
import time
from pupil_apriltags import Detector
import numpy as np
from colorama import Fore, Style


class Vision:
    def __init__(self, target_height=10, fps=10, threshold=128, tag_size_cm=3.6, default_image_path=None):
        """
        Initialize the Vision class with camera settings and a default image.

        Parameters:
        - fps: Frames per second for controlling capture speed.
        - default_image_path: Path to an image file to use as default if the camera is not accessible.
        """
        self.camera = cv2.VideoCapture(0)

        self.target_height = target_height
        self.tag_size_cm = tag_size_cm  # Size of one side of the tag in centimeters
        self.fps = fps
        self.threshold = threshold
        self.frame_delay = 1 / fps
        self.matrix = None
        self.image = None
        self.goal = None
        self.start = None
        self.angle = None
        self.crop_corners = None
        self.pixel_to_cm_scale = 1 

        # Load a default image if provided
        if default_image_path:
            self.set_image(cv2.imread(default_image_path))

    def set_image(self, image):
        """
        Manually set an image to be processed (useful when the camera is not available).
        """
        if image is not None:
            self.image = image
        else:
            raise ValueError("The provided image is None.")

    def capture_image(self):
        """
        Capture a single frame from the camera.
        """
        ret, frame = self.camera.read()
        if ret:
            return frame
        else:
            raise Exception("Error: Unable to capture image from the camera.")

    def detect_and_crop(self):
        """
        Detect AprilTags and crop the image based on the detected corners.

        Parameters:
        - display: If True, display the detected tags on the image.

        Returns:
        - cropped_image: The cropped and perspective-transformed image.
        """
        if self.image is None:
            raise ValueError("No image available for processing.")

        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        detector = Detector(families="tagStandard41h12")
        results = detector.detect(gray)

        # Debug: Affichez tous les IDs détectés
        # print(f"Tag IDs: {[result.tag_id for result in results]}")

        # Check if at least 4 tags are detected
        if len(results) < 4:
            print(f"Warning: Only {len(results)} tags detected. Cannot crop the image.")
            self.croped_image = self.image
            return None
        print(f"Detected {len(results)} tags.")

        # Map tags to corners (top-left, top-right, bottom-left, bottom-right)
        tag_positions = self._parse_tag_positions(results)
        if len(tag_positions) < 4:
            raise Exception("Error: Failed to identify all four corner tags.")

        # Define the corners of the rectangle in the source image
        corners = np.array([
            tag_positions['top_left'],
            tag_positions['top_right'],
            tag_positions['bottom_right'],
            tag_positions['bottom_left']
        ], dtype="float32")

        print(self.image.shape)


        self.calculate_scale(corners)   # Calculate the scale (pixels per centimeter)


        output_width = 1200
        output_height = 800
        dst = np.array([
            [0, 0],  # Top-left corner
            [output_width - 1, 0],  # Top-right corner
            [output_width - 1, output_height - 1],  # Bottom-right corner
            [0, output_height - 1]  # Bottom-left corner
        ], dtype="float32")

        # Perform perspective transform
        transform_matrix = cv2.getPerspectiveTransform(corners, dst)
        cropped_image = cv2.warpPerspective(self.image, transform_matrix, (output_width, output_height))

        cv2.imshow("Cropped Image", transform_matrix)

        self.croped_image = cropped_image

    def _parse_tag_positions(self, results):
        """
        Identify positions of detected AprilTags as corners.

        Parameters:
        - results: List of detected tags.

        Returns:
        - tag_positions: Dictionary mapping corner names to tag centers.
        """
        centers = {result.tag_id: result.center for result in results}

        sorted_by_y = sorted(centers.items(), key=lambda x: x[1][1])  # Sort by vertical position (Y-axis)


        # Split into top and bottom halves
        top_tags = sorted(sorted_by_y[:2], key=lambda x: x[1][0])  # Top 2 tags sorted by X-axis
        bottom_tags = sorted(sorted_by_y[2:], key=lambda x: x[1][0])  # Bottom 2 tags sorted by X-axis

        # Check if we have enough tags
        if len(top_tags) < 2 or len(bottom_tags) < 2:
            raise Exception("Error: Not enough tags detected to identify all four corners.")

        return {
            "top_left": top_tags[0][1],
            "top_right": top_tags[1][1],
            "bottom_left": bottom_tags[0][1],
            "bottom_right": bottom_tags[1][1]
        }

    def calculate_scale(self, corners):
        """
        Calculate the scale (pixels per centimeter) based on detected tag corners.

        Parameters:
        - corners: Array of tag corners in the form [(x1, y1), (x2, y2), ...]

        Updates:
        - self.pixel_to_cm_scale: Number of pixels per centimeter.
        """
        if corners.shape[0] < 4:
            raise ValueError("Not enough corners to calculate scale.")

        # Calculate distances between adjacent corners
        dist_top = np.linalg.norm(corners[0] - corners[1])  # Top side (top_left to top_right)
        dist_right = np.linalg.norm(corners[1] - corners[2])  # Right side (top_right to bottom_right)
        dist_bottom = np.linalg.norm(corners[2] - corners[3])  # Bottom side (bottom_right to bottom_left)
        dist_left = np.linalg.norm(corners[3] - corners[0])  # Left side (bottom_left to top_left)

        # Average the distances to account for potential perspective distortion
        avg_dist = (dist_top + dist_right + dist_bottom + dist_left) / 4

        # Compute the scale (pixels per centimeter)
        self.pixel_to_cm_scale = avg_dist / self.tag_size_cm
        print(f"Scale calculated: {self.pixel_to_cm_scale:.2f} pixels/cm")


    def _resize_image(self, frame):
        """
        Resize the input frame while keeping the aspect ratio,
        and adjust the coordinates of the goal, start, and scale accordingly.
        """
        height, width, _ = frame.shape
        target_width = int(self.target_height * (width / height))

        # Calculate the scale factors
        scale_x = target_width / width
        scale_y = self.target_height / height
        print(self.goal, "goal")
        print(scale_x, scale_y, "scale")

        # Resize the image
        resized_frame = cv2.resize(frame, (target_width, self.target_height), interpolation=cv2.INTER_AREA)

        # Adjust the goal and start coordinates if they exist
        if self.goal:
            self.goal = (int(self.goal[0] * scale_x), int(self.goal[1] * scale_y))
            print(self.goal, "goal")
        if self.start:
            self.start = (int(self.start[0] * scale_x), int(self.start[1] * scale_y))

        # Adjust the pixel-to-cm scale if it exists
        if self.pixel_to_cm_scale:
            self.pixel_to_cm_scale *= scale_x  # Assuming uniform scaling
            return resized_frame
        






    def find_goal(self):
        """
        Detect the goal (red region) in the cropped image, save its center coordinates,
        and replace the detected red region with white pixels.

        Updates:
        - self.goal: Tuple of (y, x) coordinates representing the center of the red region.
        """
        if self.croped_image is None:
            raise ValueError("No cropped image available. Run detect_and_crop first.")

        # Convert the cropped image to HSV color space
        hsv_image = cv2.cvtColor(self.croped_image, cv2.COLOR_BGR2HSV)

        # Define HSV range for red color (both lower and upper ranges due to HSV wrap-around)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red regions
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the red mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("No red region detected.")
            self.goal = None
            return

        # Find the largest red contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the center of the largest contour
        moments = cv2.moments(largest_contour)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])  # x-coordinate (horizontal position)
            cy = int(moments["m01"] / moments["m00"])  # y-coordinate (vertical position)
            self.goal = (cy, cx)  # Store as (row, column) for consistency
            print(f"Goal detected at: {self.goal}")
        else:
            self.goal = None
            print("Red region detected, but could not calculate center.")
            return

        # Debug: Verify goal coordinates are within bounds
        if 0 <= self.goal[1] < self.croped_image.shape[1] and 0 <= self.goal[0] < self.croped_image.shape[0]:
            print(f"Coordinates {self.goal} are within bounds.")
        else:
            print(f"Coordinates {self.goal} are out of bounds!")

        # Replace the detected red region with white pixels in the cropped image
        white_color = (255, 255, 255)
        cv2.drawContours(self.croped_image, [largest_contour], -1, white_color, thickness=cv2.FILLED)







    def find_start(self):
        """
        Detect the start point based on the 'tag36h11' AprilTag and update the image.
        Updates:
        - self.start: Tuple of (x, y) coordinates representing the center of the tag.
        - self.angle: Orientation angle in radians (circle trigonométrique).
        """
        if self.croped_image is None:
            raise ValueError("No cropped image available. Run detect_and_crop first.")

        # Convert the cropped image to grayscale for tag detection
        gray_image = cv2.cvtColor(self.croped_image, cv2.COLOR_BGR2GRAY)
        
        # Detect 'tag36h11' tags
        detector = Detector(families="tag36h11")
        results = detector.detect(gray_image)

        if not results:
            print("No 'tag36h11' tags detected.")
            self.start = None
            self.angle = None
            return




        tag_corners = results[0].corners
        # Calculate distances between adjacent corners
        dist_top = np.linalg.norm(tag_corners[1] - tag_corners[0])  # Top side
        dist_right = np.linalg.norm(tag_corners[2] - tag_corners[1])  # Right side
        dist_bottom = np.linalg.norm(tag_corners[3] - tag_corners[2])  # Bottom side
        dist_left = np.linalg.norm(tag_corners[0] - tag_corners[3])  # Left side

        # Average the distances to get a more robust measurement
        apparent_size_pixels = (dist_top + dist_right + dist_bottom + dist_left) / 4

        # tag_corners = results[0].corners
        # apparent_size_pixels = np.linalg.norm(tag_corners[0] - tag_corners[1])
        # Calculate the scale (pixels per cm)
        self.pixel_to_cm_scale = apparent_size_pixels / self.tag_size_cm
        print(f"Scale calculated: {self.pixel_to_cm_scale:.2f} pixels/cm")


        # Assuming we are interested in the first detected tag (modify if multiple tags exist)
        tag = results[0]
        # Calculate the center of the tag
        cx, cy = int(tag.center[0]), int(tag.center[1])
        self.start = (cx, cy)

        # Calculate the orientation angle
        # Use two corners (ptA, ptB) to define the orientation
        ptA = tag.corners[0]  # Corner A
        ptB = tag.corners[1]  # Corner B (next to A)
        dx, dy = ptB[0] - ptA[0], ptB[1] - ptA[1]
        self.angle = (np.arctan2(dy, dx) - np.pi/2 ) % (2 * np.pi)  # Angle in radians (0 to 2*pi)

        print(f"Start detected")

        # Replace the pixels corresponding to the tag with white in the cropped image
        white_color = (255, 255, 255)
        corners = np.array(tag.corners, dtype=np.int32)  # Convert corners to integer format
        cv2.fillPoly(self.croped_image, [corners], white_color)

        # Optionally visualize the start point and orientation
        # if self.start:
        #     cv2.circle(self.croped_image, self.start, 10, (0, 255, 0), -1)  # Green circle for the start
        #     cv2.line(self.croped_image, (int(ptA[0]), int(ptA[1])), (int(ptB[0]), int(ptB[1])), (255, 0, 0), 2)  # Blue line for orientation





    def _generate_matrix(self, resized_frame):
        """
        Generate a binary matrix for black and white representation.
        """
        gray_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2GRAY)
        _, binary_mask = cv2.threshold(gray_frame, self.threshold, 255, cv2.THRESH_BINARY)
        return np.where(binary_mask == 255, 0, -1)


    def display_image(self):
        """
        Display the image in a window.
        """
        if self.croped_image is not None:
            cv2.imshow("Image", self.croped_image)
        elif self.image is not None:
            cv2.imshow("Original (without croped) Image", self.image)
        else:
            raise Exception("No image to display. Call set_image() first.")


    def update_image(self, live=True):
        """
        Update the image displayed in the window.
        """
        if self.image is not None:
            start_time = time.time()

            ret, frame = self.camera.read()
            if not ret:
                raise Exception("Error: Unable to capture image from the camera")
            
            output_width = 1200  # Largeur réelle du terrain en cm
            output_height = 800  # Hauteur réelle du terrain en cm

            self.image = cv2.resize(self.image, (output_width, output_height))
            
            if live:
                self.set_image(frame)

            self.detect_and_crop()

            #print(self.croped_image.shape)

            self.find_goal()

            self.find_start()

            self.croped_image = self._resize_image(self.croped_image)


            self.matrix = self._generate_matrix(self.croped_image)
            time.sleep(max(0, self.frame_delay - (time.time() - start_time)))

        else:
            raise Exception("No image to update. Call set_image() first.")
        
    def getStart(self):
        return self.start
    
    def getGoal(self):
        return self.goal
    
    def getAngle(self):
        return self.angle
    
    def getScale(self):
        return self.pixel_to_cm_scale
    
    def getMatrix(self):
        return self.matrix

    def display_matrix(self):
        result = ""
        for row in self.matrix:
            result += "".join(
                f"{Fore.MAGENTA}{x:3} " if x == -1 else
                f"{Fore.WHITE}{x:3} "
                for x in row
            ) + "\n"
        print(result + Style.RESET_ALL)

    def display_all(self):
        self.display_image()
        self.display_matrix()
        print(self.matrix.shape)
        if self.start is not None and self.angle is not None:
            print(f"Start: {self.start}, Angle: {self.angle:.2f} rad")
        else:
            print("Start not detected.")

        if self.goal is not None:
            print(f"Goal: {self.goal}")
        else:
            print("Goal not detected.")
        if self.pixel_to_cm_scale is not None:
            print(f"pixel-to-cm scale: {self.pixel_to_cm_scale:.2f} cells/cm")


    def release(self):
        """
        Release the camera resource and close any OpenCV windows.
        """
        self.camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        image_path1 = "images/IMG_7018.jpeg"
        image_path2 = "images/IMG_7020.jpeg"
        image_path3 = "images/IMG_7028.jpeg"
        vision = Vision(fps=3,target_height=80, default_image_path=image_path3)

        vision.update_image(live=False)
    
        vision.display_all()

        if cv2.waitKey(0) & 0xFF == ord('q'):  # Close the window when 'q' is pressed
            pass
    finally:
        vision.release()



# if __name__ == "__main__":
#     try:
#         image_path1 = "images/IMG_7018.jpeg"
#         image_path2 = "images/IMG_7020.jpeg"
#         vision = Vision(fps=3,target_height=200, default_image_path=image_path2)

#         while True:

#             vision.update_image()

            
#             vision.display_all()


#             if cv2.waitKey(1) & 0xFF == ord('q'):  # Close the window when 'q' is pressed
#                 pass
#     finally:
#         vision.release()
