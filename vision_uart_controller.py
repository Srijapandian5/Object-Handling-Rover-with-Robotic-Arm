"""
Perception & Navigation Controller for Object Handling Rover (Raspberry Pi Zero 2 W).
Captures frames from the OV5647 camera using OpenCV, locates target objects by color/contour,
and transmits UART commands ('F', 'B', 'L', 'R', 'S', 'P') to the STM32 Blue Pill.
"""

import time
import cv2
import numpy as np
import serial


# UART configuration (adjust port/baud to match STM32 CubeMX settings)
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200

# HSV thresholds for target object detection (tune for your lighting/object color)
LOWER_HSV = np.array([0, 120, 70])
UPPER_HSV = np.array([10, 255, 255])

MIN_CONTOUR_AREA = 800
CENTER_TOLERANCE_PX = 60
PICK_AREA_THRESHOLD = 18000


def decide_command(frame_width: int, cx: int | None, area: float) -> str:
    """Return a single-character command based on detected object position and size."""
    if cx is None or area < MIN_CONTOUR_AREA:
        return "S"

    if area >= PICK_AREA_THRESHOLD:
        return "P"

    frame_center = frame_width // 2
    error_x = cx - frame_center

    if abs(error_x) <= CENTER_TOLERANCE_PX:
        return "F"
    elif error_x < -CENTER_TOLERANCE_PX:
        return "L"
    else:
        return "R"


def main() -> None:
    uart = None
    try:
        uart = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
        time.sleep(2.0)
        print(f"Connected to STM32 on {SERIAL_PORT} @ {BAUD_RATE} baud.")
    except serial.SerialException as exc:
        print(f"[WARN] Serial port unavailable ({exc}). Running in vision-only preview mode.")

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        raise RuntimeError("Failed to open camera module (OV5647).")

    last_cmd = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            height, width, _ = frame.shape
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            cx, cy, max_area = None, None, 0.0
            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                if area >= MIN_CONTOUR_AREA:
                    moments = cv2.moments(largest)
                    if moments["m00"] != 0:
                        cx = int(moments["m10"] / moments["m00"])
                        cy = int(moments["m01"] / moments["m00"])
                        max_area = area
                        cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)

            cmd = decide_command(width, cx, max_area)

            if uart and uart.is_open and cmd != last_cmd:
                uart.write(cmd.encode("ascii"))
                last_cmd = cmd
                print(f"Sent command: {cmd} (area={max_area:.0f})")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping vision controller...")
    finally:
        if uart and uart.is_open:
            uart.write(b"S")
            uart.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()