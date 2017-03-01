#!/usr/bin/env python
progname = "motion_track.py"
ver = "version 0.96"

print("%s %s using python2 and OpenCV2" % (progname, ver))
print("Loading Please Wait ....")

import os ,telegram
import RPi.GPIO as GPIO
import time

PORT = 11

GPIO.setmode(GPIO.BCM)
GPIO.setup(PORT, GPIO.OUT)

mypath = os.path.abspath(__file__)  # Find the full path of this python script
baseDir = mypath[0:mypath.rfind("/") + 1]  # get the path location only (excluding script name)
baseFileName = mypath[mypath.rfind("/") + 1:mypath.rfind(".")]
progName = os.path.basename(__file__)

# Check for variable file to import and error out if not found.
configFilePath = baseDir + "config.py"
if not os.path.exists(configFilePath):
    print("ERROR - Missing config.py file - Could not find Configuration file %s" % (configFilePath))
    import urllib2

    config_url = "https://raw.github.com/pageauc/motion-track/master/config.py"
    print("   Attempting to Download config.py file from %s" % (config_url))
    try:
        wgetfile = urllib2.urlopen(config_url)
    except:
        print("ERROR - Download of config.py Failed")
        print("   Try Rerunning the motion-track-install.sh Again.")
        print("   or")
        print("   Perform GitHub curl install per Readme.md")
        print("   and Try Again")
        print("Exiting %s" % (progName))
        quit()
    f = open('config.py', 'wb')
    f.write(wgetfile.read())
    f.close()
# Read Configuration variables from config.py file
from config import *

# import the necessary packages
import io
import time, picamera
import cv2

from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread


# -----------------------------------------------------------------------------------------------
class PiVideoStream:
    def __init__(self, resolution=(CAMERA_WIDTH, CAMERA_HEIGHT), framerate=CAMERA_FRAMERATE, rotation=0, hflip=False,
                 vflip=False):
        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.rotation = rotation
        self.camera.framerate = framerate
        self.camera.hflip = hflip
        self.camera.vflip = vflip
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True)
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update2(self):
        stream = io.BytesIO()
        for foo in camera.capture_continuous(stream, 'jpeg',
                                             use_video_port=True):
            # store frame
            stream.seek(0)
            self.frame = stream.read()

            # reset stream for next frame
            stream.seek(0)
            stream.truncate()

    def update(self):
        # keep looping infinitely until the thread is stopped
        stream = io.BytesIO()
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            #            print type(f) , f
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def update3(self):
        # keep looping infinitely until the thread is stopped
        stream = io.BytesIO()
        with picamera.PiCamera() as camera:

            for f in camera.capture_continuous(stream, format='jpeg'):  # self.stream:
                # grab the frame from the stream and clear the stream in
                # preparation for the next frame
                self.frame = f.array
                self.rawCapture.truncate(0)

                # if the thread indicator variable is set, stop the thread
                # and resource camera resources
                if self.stopped:
                    self.stream.close()
                    self.rawCapture.close()
                    self.camera.close()
                    return

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


# -----------------------------------------------------------------------------------------------
def show_FPS(start_time, frame_count):
    if debug:
        if frame_count >= FRAME_COUNTER:
            duration = float(time.time() - start_time)
            FPS = float(frame_count / duration)
            print("Processing at %.2f fps last %i frames" % (FPS, frame_count))
            frame_count = 0
            start_time = time.time()
        else:
            frame_count += 1
    return start_time, frame_count


# -----------------------------------------------------------------------------------------------
frame = None
thread_recorder = None

import threading
import datetime


def start_recording(camera , t):
    global thread_recorder
    chat_id= '116577547'
    path = '/home/pi/images/%s.jpg' % t
    try:
        bot = telegram.Bot(token='286145165:AAEDy8PgEqKAv-NS7-deHfwCaAfh6XCcgQQ')
        """
        bot.sendMessage(chat_id=chat_id,
                        text="Motion at cx=%3i cy=%3i  total_Contours=%2i  biggest_area:%3ix%3i=%5i" % (
                        cx, cy, total_contours, cw, ch, biggest_area))
        """

        bot.sendMessage( chat_id = chat_id,text="Path is %s"%path )

        camera.capture(path)

        time.sleep(2)
        img = open(path, 'rb')
        bot.sendPhoto(chat_id=chat_id, photo=img)

        camera.start_preview()
        camera.start_recording('/home/pi/videos/video.%s.h264' % t)
        time.sleep(60 * 10)
        camera.stop_recording()
        camera.stop_preview()
    except Exception as e:
        print (str(e))
    finally:
        thread_recorder = None


def motion_track():
    global frame, thread_recorder
    print("Initializing Camera ....")
    # Save images to an in-program stream
    # Setup video stream on a processor Thread for faster speed
    vs = PiVideoStream().start()
    vs.camera.rotation = CAMERA_ROTATION
    vs.camera.hflip = CAMERA_HFLIP
    vs.camera.vflip = CAMERA_VFLIP
    time.sleep(2.0)
    if window_on:
        print("press q to quit opencv display")
    else:
        print("press ctrl-c to quit")
    print("Start Motion Tracking ....")
    cx = 0
    cy = 0
    cw = 0
    ch = 0
    frame_count = 0
    start_time = time.time()
    # initialize image1 using image2 (only done first time)
    image2 = vs.read()
    image1 = image2
    grayimage1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
    first_image = False
    still_scanning = True
    while still_scanning:
        image2 = vs.read()
        frame = image2
        #        yield (b'--frame\r\n'
        #               b'Content-Type: image/jpeg\r\n\r\n' + image2 + b'\r\n')

        start_time, frame_count = show_FPS(start_time, frame_count)
        # initialize variables
        motion_found = False
        biggest_area = MIN_AREA
        # At this point the image is available as stream.array
        # Convert to gray scale, which is easier
        grayimage2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
        # Get differences between the two greyed, blurred images
        differenceimage = cv2.absdiff(grayimage1, grayimage2)
        differenceimage = cv2.blur(differenceimage, (BLUR_SIZE, BLUR_SIZE))
        # Get threshold of difference image based on THRESHOLD_SENSITIVITY variable
        retval, thresholdimage = cv2.threshold(differenceimage, THRESHOLD_SENSITIVITY, 255, cv2.THRESH_BINARY)
        # Get all the contours found in the thresholdimage
        contours, hierarchy = cv2.findContours(thresholdimage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        total_contours = len(contours)
        # save grayimage2 to grayimage1 ready for next image2
        grayimage1 = grayimage2
        # find contour with biggest area
        for c in contours:
            # get area of next contour
            found_area = cv2.contourArea(c)
            # find the middle of largest bounding rectangle
            if found_area > biggest_area:
                motion_found = True
                biggest_area = found_area
                (x, y, w, h) = cv2.boundingRect(c)
                cx = int(x + w / 2)  # put circle in middle of width
                cy = int(y + h / 6)  # put circle closer to top
                cw = w
                ch = h


        if motion_found:
            GPIO.output(PORT, GPIO.HIGH)

            # Do Something here with motion data
            if thread_recorder == None:
                t = int(time.time())


                thread_recorder = threading.Thread(target=start_recording, args=(vs.camera,t,))
                thread_recorder.start()

            if window_on:
                # show small circle at motion location
                if SHOW_CIRCLE:
                    cv2.circle(image2, (cx, cy), CIRCLE_SIZE, (0, 255, 0), LINE_THICKNESS)
                else:
                    cv2.rectangle(image2, (cx, cy), (x + cw, y + ch), (0, 255, 0), LINE_THICKNESS)
            if debug:
                print("Motion at cx=%3i cy=%3i  total_Contours=%2i  biggest_area:%3ix%3i=%5i" % (
                cx, cy, total_contours, cw, ch, biggest_area))
        else:
            GPIO.output(PORT, GPIO.LOW)

        if window_on:
            if diff_window_on:
                cv2.imshow('Difference Image', differenceimage)
            if thresh_window_on:
                cv2.imshow('OpenCV Threshold', thresholdimage)
            if WINDOW_BIGGER > 1:  # Note setting a bigger window will slow the FPS
                image2 = cv2.resize(image2, (big_w, big_h))
            cv2.imshow('Movement Status  (Press q in Window to Quit)', image2)

            # Close Window if q pressed while movement status window selected
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                vs.stop()
                print("End Motion Tracking")
                still_scanning = False


# -----------------------------------------------------------------------------------------------
thread = None


def get_frame():
    global frame
    global thread
    if thread is None:
        thread = threading.Thread(target=motion_track)
        thread.start()

    while frame is None:
        print("Waint untile frame ready")
        time.sleep(2)
    return frame


"""
if __name__ == '__main__':
    try:
        motion_track()
    finally:
        print("")
        print("+++++++++++++++++++++++++++++++++++")
        print("%s %s - Exiting" % (progname, ver))
        print("+++++++++++++++++++++++++++++++++++")
        print("")                                

"""
