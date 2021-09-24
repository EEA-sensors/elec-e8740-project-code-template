# import the necessary packages
from pyzbar import pyzbar
import argparse, numpy as np
import cv2
 
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True,
	help="path to input image")
args = vars(ap.parse_args())

# load the input image
image = cv2.imread(args["image"])
# image = cv2.imread('WIN_20191007_20_18_19_Pro.jpg')

qrDecoder = cv2.QRCodeDetector()

# Display barcode and QR code location
def display(im, bbox):
    n = len(bbox)
    for j in range(n):
        cv2.line(im, tuple(bbox[j][0]), tuple(bbox[ (j+1) % n][0]), (255,0,0), 3)
 
    # Display results
    cv2.imshow("Results", im)
     
# Detect and decode the qrcode
data,bbox,rectifiedImage = qrDecoder.detectAndDecode(image)
if len(data)>0:
    print("Decoded Data : {}".format(data))
    display(image, bbox)
    rectifiedImage = np.uint8(rectifiedImage)
    cv2.imshow("Rectified QRCode", rectifiedImage)
else:
    print("QR Code not detected")
    cv2.imshow("Results", image)
 
cv2.waitKey(0)
cv2.destroyAllWindows()