import cv2
import numpy as np

MAX_FRAME = 1000
MIN_NUM_FEAT = 2000


def featureTracking(img1, img2, points1, points2, status):
    error = []
    winSize = (21, 21)
    termcrit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
    cv2.calcOpticalFlowPyrLK(img1, img2, points1, points2, status, error,
                             winSize, 3, termcrit, 0, 0.001)

    indexCorrection = 0
    for i in range(0, status.size()):
        pt = points2[i - indexCorrection]
        if (status[i] == 0) or (pt.x < 0) or (pt.y < 0):
            if (pt.x < 0 or pt.y < 0):
                status[i] = 0
                del points1[i - indexCorrection]
                del points2[i - indexCorrection]
                indexCorrection += 1


def featureDetection(img1):
    fast = cv2.FastFeatureDetector_create()
    fast.setBool('nonmaxSuppression', True)
    fast.setInt('threshold', 20)
    keypoints1 = fast.detect(img1, cv2.NONE)
    return np.float([keypoints1[idx].pt for idx in len(keypoints1)]).reshape(
        -1, 1, 2)
    #convert keypoints to points


def getScale():
    #TODO: return from encoder speed
    return 1.0


scale = 1.0

vid = cv2.VideoCapture("vid")
while not vid.isOpened():
    vid = cv2.VideoCapture("vid")
    cv2.waitKey(1000)
    print("Wait for the header")

f1, img1c = vid.read()
f2, img2c = vid.read()
if (not f1 or not f2):
    print("error reading video")

img1 = cv2.cvtColor(img1c, cv2.COLOR_BGR2GRAY)
img2 = cv2.cvtColor(img2c, cv2.COLOR_BGR2GRAY)
points2 = []
points1 = featureDetection(img1)
status = []
featureTracking(img1, img2, points1, points2, status)

E, mask = cv2.findEssentialMat(points2, points1, 1.0)
cv2.recoverPose(E, points2, points1)

prevImage = img2
prevFeatures = points2

cv2.namedWindow("camera")
cv2.namedWindow("trajectory")
traj = np.zeroes((600, 600), cv2.CV_8UC3)

pos_frame = vid.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
while True:
    flag, frame = vid.read()
    if flag:
        # The frame is ready and already captured
        pos_frame = vid.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
        print(str(pos_frame) + " frames")
        currImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        status = []
        currFeatures = []
        featureTracking(prevImage, currImage, prevFeatures, currFeatures,
                        status)

        E, mask = cv2.findEssentialMat(currFeatures, prevFeatures, 1.0)
        pointsa, R, t, mask = cv2.recoverPose(E, currFeatures, prevFeatures)
        prevImage = img2
        prevFeatures = points2
        R_f = np.copy(R)
        t_f = np.copy(t)

        # prevPts = np.zeroes((len(prevFeatures), 2), cv2.CV_64F)
        # currPts = np.zeroes((len(currFeatures), 2), cv2.CV_64F)

        if (scale > 0.1 and t[2] > t[0] and t[2] > t[1]):
            t_f = t_f + scale * (R_f * t)
            R_f = R * R_f

        prevImage = np.copy(currImage)
        prevFeatures = currFeatures
        x = t_f[0] + 300
        y = t_f[2] + 100
        cv2.circle(traj, (x, y), 1, cv2.CV_RGB(255, 0, 0), 2)
        cv2.rectangle(traj, (10, 30), (550, 50), cv2.CV_RGB(0, 0, 0),
                      cv2.CV_FILLED)
        print('Coordinates: x = {:f}m y = {:f}m z = {:f}m'.format(
            t_f[0], t_f[1], t_f[2]))

        cv2.imshow("Road facing camera", frame)
        cv2.imshow("Trajectory", traj)
        cv2.waitKey(1)
    else:
        # The next frame is not ready, so we try to read it again
        vid.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, pos_frame - 1)
        print("frame is not ready")
        # It is better to wait for a while for the next frame to be ready
        cv2.waitKey(1000)

    if cv2.waitKey(10) == 27:
        break
    if vid.get(cv2.cv.CV_CAP_PROP_POS_FRAMES) == vid.get(
            cv2.cv.CV_CAP_PROP_FRAME_COUNT):
        # If the number of captured frames is equal to the total number of frames,
        # we stop
        break
