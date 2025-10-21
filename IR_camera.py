#!/usr/bin/env python3
import pyrealsense2 as rs
import cv2
import numpy as np
import os
from datetime import datetime

class RealsenseCamera:
    def __init__(self, bag_file=None, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.width = width
        self.height = height
        self.fps = fps

        # Enable streams
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        self.pipeline.start(self.config)

    # ------------------------
    # FRAME PROCESSING
    # ------------------------
    def get_frames(self):
        """Return latest color and depth frames along with intrinsics"""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None, None

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        color_image = cv2.resize(color_image, (self.width, self.height))

        profile = depth_frame.get_profile().as_video_stream_profile()
        intrinsics = profile.get_intrinsics()

        return color_image, depth_image, intrinsics

    def detect_colored_objects(self, color_image, depth_image, intrinsics):
        """
        Detect multiple color objects (red, green, blue) in the image.
        Returns:
            color_image: image with overlays drawn
            mask_dict: dictionary of masks per color
            detections: list of dicts with color, position, size, and 3D info
        """
    
        # HSV color boundaries (tuned for your environment)
        color_ranges = {
            "red1": ([0, 90, 160], [40, 255, 255]),     # Red low hue range
            "red2": ([160, 90, 160], [180, 255, 255]),  # Red high hue range
        }
    
        font = cv2.FONT_HERSHEY_SIMPLEX
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (9, 9), 0)
    
        mask_dict = {}
        detections = []
    
        # Iterate through each color range
        for color_name, (lower, upper) in color_ranges.items():
            lower_np = np.array(lower)
            upper_np = np.array(upper)
    
            mask = cv2.inRange(hsv, lower_np, upper_np)
    
            # Special case for red: combine both hue ranges
            if "red1" in color_ranges and "red2" in color_ranges and color_name == "!!!":
                mask += cv2.inRange(hsv,
                                    np.array(color_ranges["red2"][0]),
                                    np.array(color_ranges["red2"][1]))
    
            mask_dict[color_name] = mask
    
            # Find contours for this color
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                area = w * h
    
                # Filter out small noise areas
                if area < 500:
                    continue
    
                # Compute 3D position from depth
                center = (int(x + w / 2), int(y + h / 2))
                depth_at_center = depth_image[center[1]][center[0]]
                if depth_at_center == 0 or depth_at_center > 800: #filters out anything more than 80cm
                    continue
    
                point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_at_center)
    
                # Classify size (you can tune the threshold)
                label = "FIRE"
    
                # Draw on the image
                color_draw = {
                    "red1": (0, 0, 255),
                    "red2": (0, 0, 255),
                }.get(color_name, (255, 255, 255))
    
                cv2.rectangle(color_image, (x, y), (x + w, y + h), color_draw, 2)
                cv2.putText(color_image, f"{label} {color_name}", (x, y - 10),
                            font, 0.5, color_draw, 1)
                cv2.circle(color_image, center, 2, color_draw, -1)
    
                area_m2 = (
                    area
                    * (depth_at_center / 1000 / intrinsics.fx)
                    * (depth_at_center / 1000 / intrinsics.fy)
                    * 10000
                )
    
                detections.append({
                    "color": color_name,
                    "object": label,
                    "area_px": area,
                    "area_m2": area_m2,
                    "center_px": center,
                    "point_3d": point_3d,
                })
    
        return color_image, mask_dict, detections
    
    def stream_with_detection(self):
        """
        Continuously stream color & depth frames,
        detect red objects,
        and highlight them in real-time.
        Press 'q' to quit.
        """
        try:
            while True:
                color_img, depth_img, intrinsics = self.get_frames()
                if color_img is None:
                    continue

                # Run color object detection
                annotated_img, masks, detections = self.detect_colored_objects(color_img, depth_img, intrinsics)

                # Show output
                cv2.imshow("Annotated Color Image", annotated_img)
                cv2.imshow("Red Mask", masks.get("red1", np.zeros_like(color_img[:, :, 0])))

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("🛑 Stream stopped by user.")
                    break

        finally:
            self.stop()

    # ------------------------
    # CLEANUP
    # ------------------------
    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    cam = RealsenseCamera()
    cam.stream_with_detection()