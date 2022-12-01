# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Image recognition imports
import cv2
import mediapipe as mp
import time
import numpy as np
import pickle

# ROS imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Global variables
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

width = 640
height = 480
tol = 10

keyPoints = [0, 4, 5, 9, 13, 17, 8, 12, 16, 20]

cap = cv2.VideoCapture(0)

class velocityPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()

        def get_label(index, hand, results):
            output = None
            for idx, classification in enumerate(results.multi_handedness):
                if classification.classification[0].index == index:

                    # process results
                    label = classification.classification[0].label
                    score = classification.classification[0].score
                    #text = '{} {}'.format(label, round(score,2))

                    # coordinates
                    coords = tuple(np.multiply(np.array(
                        (hand.landmark[mp_hands.HandLandmark.WRIST].x, hand.landmark[mp_hands.HandLandmark.WRIST].y)), [width, height]).astype(int))
                    text2 = '{} {}'.format(label, coords)
                    output = text2, coords
            return output


        def findDistances(myHands):  # distance matrix (every single possible distance between points)
            distMatrix = np.zeros([len(myHands), len(myHands)], dtype='float')
            palmSize = ((myHands[0][0]-myHands[9][0])**2 +
                        (myHands[0][1]-myHands[9][1])**2)**(1./2.)
            for row in range(0, len(myHands)):
                for column in range(0, len(myHands)):
                    distMatrix[row][column] = (((myHands[row][0]-myHands[column][0])**2+(
                        myHands[row][1]-myHands[column][1])**2)**(1./2.))/palmSize
            return distMatrix


        def findError(gestureMatrix, unknownMatrix, keyPoints):  # error between known and unknown array
            error = 0
            for row in keyPoints:
                for column in keyPoints:
                    error = error+abs(gestureMatrix[row]
                                    [column]-unknownMatrix[row][column])
            return error


        # compare the unknown gestures with the known gestures and anaylyzes the error value in order to determine the gesture being displayed
        def findGesture(unknownGesture, knownGestures, keyPoints, gestNames, tol):
            errorArray = []
            for i in range(0, len(gestNames), 1):
                error = findError(knownGestures[i], unknownGesture, keyPoints)
                errorArray.append(error)
            errorMin = errorArray[0]
            minIndex = 0
            for i in range(0, len(errorArray), 1):
                if errorArray[i] < errorMin:
                    errorMin = errorArray[i]
                    minIndex = i
            if errorMin < tol:
                gesture = gestNames[minIndex]
            if errorMin >= tol:
                gesture = 'Unknown'
            return gesture

        trainName = '/home/soluopomonoki/Programs/ME369P/src/myRobot/myRobot/ROS Inputs.pkl'
        with open(trainName, 'rb') as f:
            gestNames = pickle.load(f)
            knownGestures = pickle.load(f)

        with mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands: #define hand model with tolerances

            start = time.time()
            sendCommand = True
            
            while cap.isOpened():

                success, image = cap.read() 

                image = cv2.resize(image, (width, height)) #resize image to allign with keypoints

                # Flip the image horizontally for a later selfie-view display
                # Convert the BGR image to RGB.
                image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False

                # Process the image and find hands
                results = hands.process(image)

                image.flags.writeable = True

                # Draw the hand annotations on the image.
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                myHands = [] #model of hands beign used in video feed
                if results.multi_hand_landmarks:
                    for num, hand in enumerate(results.multi_hand_landmarks):

                        mp_drawing.draw_landmarks(
                            image, hand, mp_hands.HAND_CONNECTIONS, mp_drawing.DrawingSpec(color=(121, 22, 76), thickness=2, circle_radius=4), mp_drawing.DrawingSpec(color=(250, 44, 250), thickness=2, circle_radius=2),)

                        # Render left or right hand detection
                        if get_label(num, hand, results):
                            text, coord = get_label(num, hand, results)
                            cv2.putText(image, text, coord, cv2.FONT_HERSHEY_SIMPLEX,
                                        1, (255, 255, 255), 2, cv2.LINE_AA)

                        myHand = []
                        for landMark in hand.landmark:
                            myHand.append(
                                (int(landMark.x*width), int(landMark.y*height)))
                            myHands.append(myHand)

                if myHands != []:
                    unknownGesture = findDistances(myHands[0])
                    myGesture = findGesture(
                        unknownGesture, knownGestures, keyPoints, gestNames, tol)
                    cv2.putText(image, myGesture, (100, 175),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 8)
                    #prints gesture to console
                    if myGesture == 'stop':
                        msg.linear.x = 0.0
                        msg.angular.z = 0.0
                        print('Stop')
                    elif myGesture == 'left':
                        msg.angular.z = 2.0
                        print('Turn left')
                    elif myGesture == 'right':
                        msg.angular.z = -2.0
                        print('Turn right')
                    elif myGesture == 'speed1':
                        msg.linear.x = 1.0
                        print('Speed 1')
                    elif myGesture == 'speed2':
                        msg.linear.x = 2.5
                        print('Speed 2')
                    elif myGesture == 'speed3':
                        msg.linear.x = 5.0
                        print('Speed 3')
                    elif myGesture == 'Unknown':
                        print('Unknown command')
                        sendCommand = False
                
                end = time.time()
                totalTime = end - start
                if totalTime >= 0.125:
                    break
        
        if sendCommand:
            self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    velocity = velocityPublisher()  # creates publisher
    rclpy.spin(velocity)  # runs node continuously

    velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
