# import the necessary packages
from imutils.video import VideoStream
from pyzbar import pyzbar
import argparse
import datetime
import time
import imutils
import time
import cv2
import numpy as np
import parser_help as ph
from pynput.keyboard import Key, Listener
import key_help as kh




# dt = str(datetime.datetime.now())

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="CameraTracking-{}.csv".format(datetime.datetime.now()),
	help="path to output CSV file containing barcodes")

ap.add_argument("-s", "--scale", type=int, default=2,
	help="resolution scale to 320x240, default 2")
ap.add_argument("-q", "--qrlength", type=float, default=11.5,
	help="qr-code side length, default 11.5 cm")
ph.add_boolean_argument(ap,'show',default=False,messages='Show openCV videoStream, Default=False')
ph.add_boolean_argument(ap,'print',default=False,messages='print openCV reading, Default=False')
args = vars(ap.parse_args())


QRCODE_SIDE_LENGTH = args["qrlength"]#6.5 #cm
PERCEIVED_FOCAL_LENGTH = 6200/QRCODE_SIDE_LENGTH #pixel
DEFAULT_RESOLUTION_WIDTH = 320
DEFAULT_RESOLUTION_HEIGHT = 240
RESOLUTION_SCALE = args["scale"]
RESOLUTION_WIDTH = RESOLUTION_SCALE*DEFAULT_RESOLUTION_WIDTH
RESOLUTION_HEIGHT = RESOLUTION_SCALE*DEFAULT_RESOLUTION_HEIGHT
HALF_RESOLUTION_WIDTH = RESOLUTION_WIDTH//2
HALF_RESOLUTION_HEIGHT = RESOLUTION_HEIGHT//2
RAD_TO_DEG = 180/np.pi
is_image_shown = args["show"]
is_printed = args["print"]

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
# vs = VideoStream(src=0).start()
vs = VideoStream(usePiCamera=True,resolution=(RESOLUTION_WIDTH,RESOLUTION_HEIGHT)).start()
time.sleep(2.0)
 
# open the output CSV file for writing and initialize the set of
# barcodes found thus far
csv = open(args["output"], "w")
# found = set()
frame = vs.read()


# listener = Listener(on_release=kh.on_release)
# listener.start()
# listener.wait()

# loop over the frames from the video stream until escape is pressed
while True:
# while (listener.running):
	#time-stamp
	timestamp = time.time()

	# grab the frame from the threaded video stream and resize it to
	# have a maximum width of 400 pixels
	frame = vs.read()
	# frame = imutils.resize(frame, width=400)

 
	barcodes = pyzbar.decode(frame)
	# print('Detected {0} barcodes in the image'.format(len(barcodes)))
    
	# loop over the detected barcodes
	for barcode in barcodes:
		# the barcode data is a bytes object so if we want to draw it
		# on our output image we need to convert it to a string first
		barcodeData = barcode.data.decode("utf-8")
		barcodeType = barcode.type

		# extract the bounding box location of the barcode and draw
		# the bounding box surrounding the barcode on the image
		(x, y, w, h) = barcode.rect
		c_x = x + w//2 - HALF_RESOLUTION_WIDTH
		c_y = y + h//2 - HALF_RESOLUTION_HEIGHT
		

		

		
		
		perceived_distance = QRCODE_SIDE_LENGTH*PERCEIVED_FOCAL_LENGTH/h #<-- height is more robust, given in cm
		perceived_direction = np.arctan2(c_x,PERCEIVED_FOCAL_LENGTH) * RAD_TO_DEG#<-- given in degree

		#uncomment these
		# print('barcode {} detected at ({} px ,{} px) with width={} px,height={} px'.format(barcodeData,c_x,c_y,w,h))
		if is_image_shown:
			#TODO:comment these
			cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)#<--draw rectangle
			cv2.circle(frame, (c_x+HALF_RESOLUTION_WIDTH, c_y+HALF_RESOLUTION_HEIGHT), 7, (255, 255, 255), -1) #<--marks the center of the rectangle
			
			
			
			#TODO:comment these
			text = "{}".format(barcodeData)
			cv2.putText(frame, text, (x, y - 10),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)#<--draw the barcode data and barcode type on the image
		elif is_printed:
			#uncomment these
			# print('barcode {} detected at ({} px ,{} px) with width={} px,height={} px'.format(barcodeData,c_x,c_y,w,h))
			print('barcode {} detected at ({} cm ,{} deg) with width={} px,height={} px'.format(barcodeData,perceived_distance,perceived_direction,w,h))
		#write CSV
		csv.write("{},{},{},{},{},{},{},{}\n".format(timestamp,barcodeData,c_x,c_y,w,h,perceived_distance,perceived_direction))
		csv.flush()
		# found.add(barcodeData)
	
	#TODO:comment these
	if is_image_shown:
		cv2.imshow("Barcode Scanner", frame)#<--show the output frame
	
	key = cv2.waitKey(1) & 0xFF

	# if the ESC key was pressed, break from the loop
	if key == 27:
		break
 
# close the output CSV file do a bit of cleanup
print("[INFO] cleaning up...")
csv.close()
cv2.destroyAllWindows()
vs.stop()