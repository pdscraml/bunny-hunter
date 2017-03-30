
# import the necessary packages
import numpy as np
import argparse
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image")
args = vars(ap.parse_args())

# define window
cv2.namedWindow("results", cv2.WINDOW_NORMAL)

# load the image
image = cv2.imread(args["image"])

output = []

# define the list of boundaries
boundaries = [([45, 215, 240],[75, 230, 255]), # yellow
              ([165, 90, 15], [200, 110, 30]), # blue
              ([65, 125, 25], [125, 205, 120]),# green
              ([35, 45, 225], [80, 100, 255]), # orange
              ([75, 10, 160], [105, 45, 210])] # red

# loop over the boundaries
for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    # find the colors within the specified boundaries and apply the mask
    mask = cv2.inRange(image, lower, upper)
    # output = cv2.bitwise_and(image, image, mask = mask)
    output.append(cv2.bitwise_and(image, image, mask = mask))
    # cv2.imshow("results", output[2])
    # cv2.imshow("results", np.hstack([image, output]))
    # cv2.waitKey(0)

# show the images
img_arr1 = np.hstack([image, output[0], output[1]])
img_arr2 = np.hstack([output[2], output[3], output[4]])

cv2.imshow("results", np.vstack([img_arr1, img_arr2]))
cv2.waitKey(0)
