import cv2
import sys

# Get user supplied values
imagePath = sys.argv[1]
cascPath = "haarcascade_frontalface_default.xml"

#number of ramp frames to toss while adjusting to light
ramp_frames = 15

#init camera
cam = cv2.VideoCapture(0)

#function to take a picture
def get_image():
	retval, im = cam.read()
	return im

#ramp camera
for i in xrange(ramp_frames):
	temp = get_image()


#take picture
print("Taking picture")
image = get_image()

# Create the haar cascade
faceCascade = cv2.CascadeClassifier(cascPath)

# Read the image
if(sys.argv[1]):
    image = cv2.imread(imagePath)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect faces in the image
faces = faceCascade.detectMultiScale(
    gray,
    scaleFactor=1.1,
    minNeighbors=5,
    minSize=(30, 30)
    #flags = cv2.CV_HAAR_SCALE_IMAGE
)

print("Found {0} faces!".format(len(faces)))

print faces

# Draw a rectangle around the faces
for (x, y, w, h) in faces:
    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
    cv2.circle(image, (x+w/2, y+h/2), 50, (0, 255, 0), 2)
    print "x: ",x,"y: ",y,"w: ",w,"h: ",h

cv2.imshow("Faces found", image)
cv2.imwrite("./faces.png", image)

del(cam)
cv2.waitKey(0)
