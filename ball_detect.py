import math

import cv2
import naoqi
import numpy as np
from config import *

# Connect to NAO robot
motion_proxy = naoqi.ALProxy("ALMotion", IP, PORT)
video_proxy = naoqi.ALProxy("ALVideoDevice", IP, PORT)
face_leds = ALProxy("ALLeds", IP, PORT)
motion_proxy.wakeUp()
head_pitch = 0.0
head_yaw = -1.0
max_head_yaw = 1.0  # Maximum head yaw angle in radians
max_head_pitch = 0.5
head_move_duration = 5  # Duration for head movement in seconds
max_walk_velocity = 1.0

# Set up camera parameters
camera_index = 1
resolution = 2  # VGA resolution
color_space = 11  # RGB color space
fps = 30

# Create a window to display the camera feed
cv2.namedWindow("NAO Camera Feed", cv2.WINDOW_NORMAL)

# Load the ball recognition model (assuming you have trained one)
model = cv2.CascadeClassifier("/home/student/PycharmProjects/pythonProject/bottom_cascade.xml")
motion_proxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
stop_head_rotation = False
ball_found = False
# Capture and process camera feed
while True:
    state = "ballSearch"
    balls = None
    while state == "ballSearch":
        color_name = "blue"
        duration = 1.0  # Duration of the LED animation in seconds
        face_leds.fadeRGB("EarLeds", color_name, duration)
        for yaw in np.arange(head_yaw, max_head_yaw + 0.2, 0.2):  # Increase head yaw angle by 0.2 radians
            for pitch in np.arange(head_pitch, max_head_pitch + 0.1, 0.1):  # Increase head pitch angle by 0.1 radians
                motion_proxy.setAngles(["HeadYaw", "HeadPitch"], [yaw, pitch], 0.2)
                # Capture image from NAO camera
                video_client = video_proxy.subscribeCamera("ball_detection", camera_index, resolution, color_space, fps)
                img = video_proxy.getImageRemote(video_client)
                video_proxy.unsubscribe(video_client)
                # Process the image for ball detection
                img_data = img[6]
                img_width = img[0]
                img_height = img[1]
                img_array = np.frombuffer(img_data, dtype=np.uint8).reshape((img_height, img_width, 3))
                gray = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)
                # Detect balls in the image
                balls = model.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=20, minSize=(30, 30))
                # Draw bounding boxes around detected balls
                for (x, y, w, h) in balls:
                    cv2.rectangle(img_array, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(img_array, "ball", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    print(balls)
                    print("Searching for ball")
                print("LENGTH: " + str(len(balls)))
                cv2.imshow("NAO Camera Feed", img_array)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                if len(balls) > 0:
                    color_name = "red"
                    duration = 1.0  # Duration of the LED animation in seconds
                    face_leds.fadeRGB("EarLeds", color_name, duration)
                    print("BALL DETECTED")
                    state = "rotateToBall"
                    motion_proxy.setStiffnesses("Head", 0.0)
                    state = "rotateToBall"
                    break

            if state != "ballSearch":
             break
    rotate = False
    rotate_once = False
    while state == "rotateToBall":
        print(" ROTATE TO BALL")
        rotate = True
        # Ohr LEDS auf rot
        color_name = "green"
        duration = 1.0  # Duration of the LED animation in seconds
        face_leds.fadeRGB("EarLeds", color_name, duration)
        video_client = video_proxy.subscribeCamera("ball_detection", camera_index, resolution, color_space, fps)
        img = video_proxy.getImageRemote(video_client)
        video_proxy.unsubscribe(video_client)

        # Process the image for ball detection
        img_data = img[6]
        img_width = img[0]
        img_height = img[1]
        img_array = np.frombuffer(img_data, dtype=np.uint8).reshape((img_height, img_width, 3))
        gray = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)

        # Detect balls in the image
        balls = model.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=20, minSize=(30, 30))
        for (x, y, w, h) in balls:
            cv2.rectangle(img_array, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img_array, "ball", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        for (x, y, w, h) in balls:
            # Ball found, stop head rotation
            stop_head_rotation = True

            # Draw bounding boxes around detected balls
            cv2.rectangle(img_array, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img_array, "ball", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            ball_center_x = x + w / 2
            ball_center_y = y + h / 2
            robot_x = img_width / 2
            robot_y = img_height / 2

            # Calculate the relative position of the ball
            ball_rel_x = float(ball_center_x) / img_width - 0.5
            ball_rel_y = float(ball_center_y) / img_height - 0.5
            print("Ball location - X: {:.2f}, Y: {:.2f}".format(ball_rel_x, ball_rel_y))

            # Calculate the angle to rotate the body
            # Ball in cneter of image bringen bzgl bildspalte (x)
            angle_to_ball = np.arctan2(ball_rel_x, ball_rel_y)
            body_rotation_angle = angle_to_ball - head_yaw

            # Check if the robot is already facing the ball
            if abs(body_rotation_angle) < 0.1:
                stop_head_rotation = False
                ball_found = True
                state = "walkToBall"
                break

            # Determine the direction to move
            if body_rotation_angle < 0:
                # Rotate left
                print("Rotate rigt: " + str(body_rotation_angle))
                motion_proxy.moveTo(0, 0, -0.4)
                head_pitch = 0.0
                head_yaw = 0.0
                state = "walkToBall"
                # Kopf entgegengesetzt drehen
            else:
                # Rota2te right
                print("Rotate left: " + str(body_rotation_angle))
                motion_proxy.moveTo(0, 0, 0.4)
                head_pitch = 0.0
                head_yaw = 0.0
                state = "walkToBall"

            ball_found = True
        # Check if the robot has rotated once
        if rotate_once:
            break

        # Check if the robot has completed one full rotation
    if rotate and abs(body_rotation_angle) < 0.1:
        rotate_once = True
        state="walkToBall"
        break
    while state == "walkToBall":

        head_pitch = 0.0
        head_yaw = 0.0
        # Capture image from NAO camera
        video_client = video_proxy.subscribeCamera("ball_detection", camera_index, resolution, color_space, fps)
        img = video_proxy.getImageRemote(video_client)
        video_proxy.unsubscribe(video_client)

        # Process the image for ball detection
        img_data = img[6]
        img_width = img[0]
        img_height = img[1]
        img_array = np.frombuffer(img_data, dtype=np.uint8).reshape((img_height, img_width, 3))
        gray = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)

        # Detect balls in the image
        balls = model.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=20, minSize=(30, 30))

        # Draw bounding boxes around detected balls
        for (x, y, w, h) in balls:
            cv2.rectangle(img_array, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img_array, "ball", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # Move the robot towards the ball
            ball_center_x = x + w / 2
            ball_center_y = y + h / 2
            robot_x = img_width / 2
            robot_y = img_height / 2

            # Calculate the relative position of the ball
            ball_rel_x = float(ball_center_x) / img_width
            ball_rel_y = float(ball_center_y) / img_height

            # Move the robot's head to look at the ball
            head_yaw = max_head_yaw * (2 * ball_rel_x)
            head_pitch = max_head_pitch * (1 - 1 * ball_rel_y)
            motion_proxy.setAngles(["HeadYaw", "HeadPitch"], [head_yaw, head_pitch], 0.2)

            # Move the robot towards the ball
            walk_velocity = 1.0
            walk_rotation = 0.0
            print("LaufeZumBall")
            motion_proxy.setWalkTargetVelocity(walk_velocity, 0, walk_rotation, 0.1)

            # Print the coordinates of the ball location
            print("Ball location - X: {:.2f}, Y: {:.2f}".format(ball_rel_x, ball_rel_y))
            # Stop processing other balls
            break

        if len(balls) == 0:
            # Ball not found, return head to the center position
            motion_proxy.setAngles(["HeadYaw", "HeadPitch"], [0.0, 0.0], 0.2)
            # Stop walking
            motion_proxy.stopWalk()

        # Display the processed image with bounding boxes
        cv2.imshow("NAO Camera Feed", img_array)

        # Check for key press to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release resources
cv2.destroyAllWindows()