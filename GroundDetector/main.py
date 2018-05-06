from cv2 import *
import numpy as np

cap = VideoCapture('output.mp4')
print cap

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()




    # Display the resulting frame
    imshow('frame', frame)
    if waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
destroyAllWindows()

