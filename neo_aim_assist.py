import mss
import cv2
import numpy as np
import pyautogui
import time
import threading
import tkinter as tk
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class AimAssist:
    def __init__(self):
        self.aim_assist_running = False
        self.screen_center = None
        self.lock = threading.Lock()
        self.prev_position = None

    def capture_screen(self):
        try:
            with mss.mss() as sct:
                monitor = sct.monitors[1]  # Capture the first monitor
                screenshot = sct.grab(monitor)
                img = np.array(screenshot)
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                return cv2.resize(img, (0, 0), fx=0.5, fy=0.5)  # Downscale for performance
        except Exception as e:
            logging.error(f"Error capturing screen: {e}")
            return None

    def detect_target(self, screenshot):
        if screenshot is None:
            return []
        hsv = cv2.cvtColor(screenshot, cv2.COLOR_BGR2HSV)
        lower_color = np.array([0, 150, 150])
        upper_color = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def find_closest_target(self, contours):
        if not contours or self.screen_center is None:
            return None
        min_dist = float('inf')
        closest_target = None
        for contour in contours:
            if cv2.contourArea(contour) < 100:  # Filter out small contours
                continue
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cX = int(M['m10'] / M['m00']) * 2  # Upscale coordinates
                cY = int(M['m01'] / M['m00']) * 2
                dist = np.sqrt((cX - self.screen_center[0]) ** 2 + (cY - self.screen_center[1]) ** 2)
                if dist < min_dist:
                    min_dist = dist
                    closest_target = (cX, cY)
        return closest_target

    def smooth_move_to_target(self, target_position):
        if target_position:
            current_x, current_y = pyautogui.position()
            target_x, target_y = target_position
            step_count = np.random.randint(15, 25)
            x_step = (target_x - current_x) / step_count
            y_step = (target_y - current_y) / step_count
            for i in range(1, step_count + 1):
                pyautogui.moveTo(current_x + i * x_step, current_y + i * y_step, duration=np.random.uniform(0.01, 0.03))

    def assist_aim(self):
        while self.aim_assist_running:
            start_time = time.time()
            screenshot = self.capture_screen()
            contours = self.detect_target(screenshot)
            if self.screen_center is None:
                self.screen_center = (screenshot.shape[1] // 2, screenshot.shape[0] // 2)
            target_position = self.find_closest_target(contours)
            if target_position is not None and len(target_position) > 0:
                self.smooth_move_to_target(target_position)
            elapsed_time = time.time() - start_time
            time.sleep(max(np.random.uniform(0.05, 0.1) - elapsed_time, 0))  # Dynamic sleep time

    def start_aim_assist(self):
        with self.lock:
            if not self.aim_assist_running:
                self.aim_assist_running = True
                threading.Thread(target=self.assist_aim).start()

    def stop_aim_assist(self):
        with self.lock:
            self.aim_assist_running = False

def create_gui(aim_assist):
    root = tk.Tk()
    root.title("Aim Assist")

    start_button = tk.Button(root, text="Start", command=aim_assist.start_aim_assist)
    start_button.pack(pady=10)

    stop_button = tk.Button(root, text="Stop", command=aim_assist.stop_aim_assist)
    stop_button.pack(pady=10)

    root.mainloop()

if __name__ == "__main__":
    aim_assist = AimAssist()
    create_gui(aim_assist)
