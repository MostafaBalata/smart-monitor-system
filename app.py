#!/usr/bin/env python
from flask import Flask, render_template, Response

# emulated camera
#from camera import Camera
from camera_pi import Camera
# Raspberry Pi camera module (requires picamera package)
# from camera_pi import Camera
from motion_tracker import get_frame
app = Flask(__name__)


@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


def gen(camera):
    """Video streaming generator function."""
    #motion_track()
    while True:
        frame = camera.get_frame()
#        print type(frame),frame
#	yield (frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
from StringIO import StringIO
def gen2():
#    from PIL import Image
#    import io
    import cv2
    #im = Image.fromarray(A)
    while True:
        frame = get_frame()
	frame = cv2.imencode('.jpg', frame)[1].tostring()


#        print type(frame),frame
#	frame = Image.fromarray(frame)
#	frame = frame.tostring()
#	print(type(frame), frame)
#	yield (frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/tracking')
def tracker():
#    while True:
#        frame = get_frame()
#        yield (b'--frame\r\n'
#               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
		
    return Response(gen2(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, threaded=True)
	
