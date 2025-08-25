# Imports
import cv2
import mediapipe as mp
import pyautogui
import math
from enum import IntEnum
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from google.protobuf.json_format import MessageToDict
import screen_brightness_control as sbcontrol

pyautogui.FAILSAFE = False
mpDrawing = mp.solutions.drawing_utils
mpHands = mp.solutions.hands

# Gesture Encodings 
class Gest(IntEnum):
    # Binary Encoded
    """
    Enum for mapping all hand gesture to binary number.
    """

    FIST = 0
    PINKY = 1
    RING = 2
    MID = 4
    LAST3 = 7
    INDEX = 8
    FIRST2 = 12
    LAST4 = 15
    THUMB = 16    
    PALM = 31
    
    # Extra Mappings
    V_GEST = 33
    TWO_FINGER_CLOSED = 34
    PINCH_MAJOR = 35
    PINCH_MINOR = 36

# Multi-handedness Labels
class HLabel(IntEnum):
    MINOR = 0
    MAJOR = 1

# Convert Mediapipe Landmarks to recognizable Gestures
class HandRecog:
    def __init__(self, handLabel):
        self.finger = 0
        self.oriGesture = Gest.PALM
        self.prevGesture = Gest.PALM
        self.frameCount = 0
        self.handResult = None
        self.handLabel = handLabel
    
    def updateHandResult(self, handResult):
        self.handResult = handResult

    def getSignedDist(self, point):
        sign = -1
        if self.handResult.landmark[point[0]].y < self.handResult.landmark[point[1]].y:
            sign = 1
        dist = (self.handResult.landmark[point[0]].x - self.handResult.landmark[point[1]].x)**2
        dist += (self.handResult.landmark[point[0]].y - self.handResult.landmark[point[1]].y)**2
        dist = math.sqrt(dist)
        return dist * sign
    
    def getDist(self, point):
        dist = (self.handResult.landmark[point[0]].x - self.handResult.landmark[point[1]].x)**2
        dist += (self.handResult.landmark[point[0]].y - self.handResult.landmark[point[1]].y)**2
        dist = math.sqrt(dist)
        return dist
    
    def getDz(self, point):
        return abs(self.handResult.landmark[point[0]].z - self.handResult.landmark[point[1]].z)
    
    # Function to find Gesture Encoding using current fingerState.
    # fingerState: 1 if finger is open, else 0
    def setFingerState(self):
        if self.handResult is None:
            return

        points = [[8,5,0],[12,9,0],[16,13,0],[20,17,0]]
        self.finger = 0
        self.finger = self.finger | 0  # thumb
        for idx, point in enumerate(points):
            dist = self.getSignedDist(point[:2])
            dist2 = self.getSignedDist(point[1:])
            
            try:
                ratio = round(dist/dist2, 1)
            except:
                ratio = round(dist1/0.01, 1)

            self.finger = self.finger << 1
            if ratio > 0.5:
                self.finger = self.finger | 1

    # Handling Fluctuations due to noise
    def getGesture(self):
        """
        Returns int representing gesture corresponding to Enum 'Gest'.
        Sets 'frameCount', 'oriGesture', 'prevGesture', 
        handles fluctuations due to noise.
        
        Returns
        -------
        int
        """
        if self.handResult is None:
            return Gest.PALM

        currentGesture = Gest.PALM
        if self.finger in [Gest.LAST3, Gest.LAST4] and self.getDist([8,4]) < 0.05:
            if self.handLabel == HLabel.MINOR:
                currentGesture = Gest.PINCH_MINOR
            else:
                currentGesture = Gest.PINCH_MAJOR
        elif Gest.FIRST2 == self.finger:
            point = [[8,12],[5,9]]
            dist1 = self.getDist(point[0])
            dist2 = self.getDist(point[1])
            ratio = dist1/dist2
            if ratio > 1.7:
                currentGesture = Gest.V_GEST
            else:
                if self.getDz([8,12]) < 0.1:
                    currentGesture = Gest.TWO_FINGER_CLOSED
                else:
                    currentGesture = Gest.MID
        else:
            currentGesture = self.finger
        
        if currentGesture == self.prevGesture:
            self.frameCount += 1
        else:
            self.frameCount = 0

        self.prevGesture = currentGesture

        if self.frameCount > 4:
            self.oriGesture = currentGesture
        return self.oriGesture

# Executes commands according to detected gestures
class Controller:
    txOld = 0
    tyOld = 0
    trial = True
    flag = False
    grabFlag = False
    pinchMajorFlag = False
    pinchMinorFlag = False
    pinchStartXCoord = None
    pinchStartYCoord = None
    pinchDirectionFlag = None
    prevPinchLv = 0
    pinchLv = 0
    frameCount = 0
    prevHand = None
    pinchThreshold = 0.3
    
    @staticmethod
    def getPinchYlv(handResult):
        dist = round((Controller.pinchStartYCoord - handResult.landmark[8].y)*10, 1)
        return dist

    @staticmethod
    def getPinchXlv(handResult):
        """Returns distance between starting pinch x coord and current hand position x coord."""
        dist = round((handResult.landmark[8].x - Controller.pinchStartXCoord)*10, 1)
        return dist
    
    @staticmethod
    def changeSystemBrightness():
        """Sets system brightness based on 'Controller.pinchLv'."""
        currentBrightnessLv = sbcontrol.get_brightness(display=0)/100.0
        currentBrightnessLv += Controller.pinchLv/50.0
        if currentBrightnessLv > 1.0:
            currentBrightnessLv = 1.0
        elif currentBrightnessLv < 0.0:
            currentBrightnessLv = 0.0       
        sbcontrol.fade_brightness(int(100*currentBrightnessLv) , start = sbcontrol.get_brightness(display=0))
    
    @staticmethod
    def changeSystemVolume():
        """Sets system volume based on 'Controller.pinchLv'."""
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        volume = cast(interface, POINTER(IAudioEndpointVolume))
        currentVolumeLv = volume.GetMasterVolumeLevelScalar()
        currentVolumeLv += Controller.pinchLv/50.0
        if currentVolumeLv > 1.0:
            currentVolumeLv = 1.0
        elif currentVolumeLv < 0.0:
            currentVolumeLv = 0.0
        volume.SetMasterVolumeLevelScalar(currentVolumeLv, None)
    
    @staticmethod
    def scrollVertical():
        """Scrolls on screen vertically."""
        pyautogui.scroll(120 if Controller.pinchLv > 0.0 else -120)
    
    @staticmethod
    def scrollHorizontal():
        """Scrolls on screen horizontally."""
        pyautogui.keyDown('shift')
        pyautogui.keyDown('ctrl')
        pyautogui.scroll(-120 if Controller.pinchLv > 0.0 else 120)
        pyautogui.keyUp('ctrl')
        pyautogui.keyUp('shift')

    # Locate Hand to get Cursor Position
    # Stabilize cursor by Dampening
    @staticmethod
    def getPosition(handResult):
        """
        Returns coordinates of the current hand position.

        Locates hand to get cursor position also stabilizes cursor by 
        dampening jerky motion of the hand.

        Returns
        -------
        tuple(float, float)
        """
        point = 9
        position = [handResult.landmark[point].x, handResult.landmark[point].y]
        sx, sy = pyautogui.size()
        xOld, yOld = pyautogui.position()
        x = int(position[0] * sx)
        y = int(position[1] * sy)
        if Controller.prevHand is None:
            Controller.prevHand = x, y
        delta_x = x - Controller.prevHand[0]
        delta_y = y - Controller.prevHand[1]

        distsq = delta_x**2 + delta_y**2
        ratio = 1
        Controller.prevHand = [x, y]

        if distsq <= 25:
            ratio = 0
        elif distsq <= 900:
            ratio = 0.07 * (distsq ** (1/2))
        else:
            ratio = 2.1
        x, y = xOld + delta_x * ratio, yOld + delta_y * ratio
        return (x, y)

    @staticmethod
    def pinchControlInit(handResult):
        """Initializes attributes for the pinch gesture."""
        Controller.pinchStartXCoord = handResult.landmark[8].x
        Controller.pinchStartYCoord = handResult.landmark[8].y
        Controller.pinchLv = 0
        Controller.prevPinchLv = 0
        Controller.frameCount = 0

    # Hold the final position for 5 frames to change the status
    @staticmethod
    def pinchControl(handResult, controlHorizontal, controlVertical):
        """
        Calls 'controlHorizontal' or 'controlVertical' based on pinch flags, 
        'frameCount' and sets 'pinchLv'.

        Parameters
        ----------
        handResult : Object
            Landmarks obtained from Mediapipe.
        controlHorizontal : callback function associated with horizontal
            pinch gesture.
        controlVertical : callback function associated with vertical
            pinch gesture. 
        
        Returns
        -------
        None
        """
        if Controller.frameCount == 5:
            Controller.frameCount = 0
            Controller.pinchLv = Controller.prevPinchLv

            if Controller.pinchDirectionFlag is True:
                controlHorizontal()  # x

            elif Controller.pinchDirectionFlag is False:
                controlVertical()  # y

        lvx = Controller.getPinchXlv(handResult)
        lvy = Controller.getPinchYlv(handResult)
            
        if abs(lvy) > abs(lvx) and abs(lvy) > Controller.pinchThreshold:
            Controller.pinchDirectionFlag = False
            if abs(Controller.prevPinchLv - lvy) < Controller.pinchThreshold:
                Controller.frameCount += 1
            else:
                Controller.prevPinchLv = lvy
                Controller.frameCount = 0

        elif abs(lvx) > Controller.pinchThreshold:
            Controller.pinchDirectionFlag = True
            if abs(Controller.prevPinchLv - lvx) < Controller.pinchThreshold:
                Controller.frameCount += 1
            else:
                Controller.prevPinchLv = lvx
                Controller.frameCount = 0

    @staticmethod
    def handleControls(gesture, handResult):  
        """Implements all gesture functionality."""      
        x, y = None, None
        if gesture != Gest.PALM:
            x, y = Controller.getPosition(handResult)
        
        # Flag reset
        if gesture != Gest.FIST and Controller.grabFlag:
            Controller.grabFlag = False
            pyautogui.mouseUp(button="left")

        if gesture != Gest.PINCH_MAJOR and Controller.pinchMajorFlag:
            Controller.pinchMajorFlag = False

        if gesture != Gest.PINCH_MINOR and Controller.pinchMinorFlag:
            Controller.pinchMinorFlag = False

        # Implementation
        if gesture == Gest.V_GEST:
            Controller.flag = True
            pyautogui.moveTo(x, y, duration=0.1)

        elif gesture == Gest.FIST:
            if not Controller.grabFlag: 
                Controller.grabFlag = True
                pyautogui.mouseDown(button="left")
            pyautogui.moveTo(x, y, duration=0.1)

        elif gesture == Gest.MID and Controller.flag:
            pyautogui.click()
            Controller.flag = False

        elif gesture == Gest.INDEX and Controller.flag:
            pyautogui.click(button='right')
            Controller.flag = False

        elif gesture == Gest.TWO_FINGER_CLOSED and Controller.flag:
            pyautogui.doubleClick()
            Controller.flag = False

        elif gesture == Gest.PINCH_MINOR:
            if not Controller.pinchMinorFlag:
                Controller.pinchControlInit(handResult)
                Controller.pinchMinorFlag = True
            Controller.pinchControl(handResult, Controller.scrollHorizontal, Controller.scrollVertical)
        
        elif gesture == Gest.PINCH_MAJOR:
            if not Controller.pinchMajorFlag:
                Controller.pinchControlInit(handResult)
                Controller.pinchMajorFlag = True
            Controller.pinchControl(handResult, Controller.changeSystemBrightness, Controller.changeSystemVolume)


class GestureController:

    gcMode = 0
    cap = None
    CAM_HEIGHT = None
    CAM_WIDTH = None
    hrMajor = None  # Right Hand by default
    hrMinor = None  # Left hand by default
    domHand = True

    def __init__(self):
        """Initializes attributes."""
        GestureController.gcMode = 1
        GestureController.cap = cv2.VideoCapture(0)
        GestureController.CAM_HEIGHT = Gesture
        GestureController.CAM_HEIGHT = GestureController.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        GestureController.CAM_WIDTH = GestureController.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    
    @staticmethod
    def classifyHands(results):
        left, right = None, None
        try:
            handednessDict = MessageToDict(results.multi_handedness[0])
            if handednessDict['classification'][0]['label'] == 'Right':
                right = results.multi_hand_landmarks[0]
            else:
                left = results.multi_hand_landmarks[0]
        except:
            pass

        try:
            handednessDict = MessageToDict(results.multi_handedness[1])
            if handednessDict['classification'][0]['label'] == 'Right':
                right = results.multi_hand_landmarks[1]
            else:
                left = results.multi_hand_landmarks[1]
        except:
            pass
        
        if GestureController.domHand is True:
            GestureController.hrMajor = right
            GestureController.hrMinor = left
        else:
            GestureController.hrMajor = left
            GestureController.hrMinor = right

    def start(self):
        handMajor = HandRecog(HLabel.MAJOR)
        handMinor = HandRecog(HLabel.MINOR)

        with mpHands.Hands(max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
            while GestureController.cap.isOpened() and GestureController.gcMode:
                success, image = GestureController.cap.read()

                if not success:
                    print("Ignoring empty camera frame.")
                    continue
                
                image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
                image.flags.writeable = False
                results = hands.process(image)
                
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                if results.multi_hand_landmarks:                   
                    GestureController.classifyHands(results)
                    handMajor.updateHandResult(GestureController.hrMajor)
                    handMinor.updateHandResult(GestureController.hrMinor)

                    handMajor.setFingerState()
                    handMinor.setFingerState()
                    gestName = handMinor.getGesture()

                    if gestName == Gest.PINCH_MINOR:
                        Controller.handleControls(gestName, handMinor.handResult)
                    else:
                        gestName = handMajor.getGesture()
                        Controller.handleControls(gestName, handMajor.handResult)
                    
                    for handLandmarks in results.multi_hand_landmarks:
                        mpDrawing.draw_landmarks(image, handLandmarks, mpHands.HAND_CONNECTIONS)
                else:
                    Controller.prevHand = None
                cv2.imshow('Gesture Controller', image)
                if cv2.waitKey(5) & 0xFF == 13:
                    break
        GestureController.cap.release()
        cv2.destroyAllWindows()

gc1 = GestureController()
gc1.start()
