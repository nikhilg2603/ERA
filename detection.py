#!/usr/bin/env python
import cv2 as cv
from cv2 import aruco
import numpy as np
import rospy
from std_msgs.msg import Int8, Int32MultiArray

list1 = [4, 5]  # upper
list2 = []  # right
list3 = []  # bottom
list4 = []  # left
id_to_cordinate = {
    1: [1, 2],
    2: [3, 4],
    4: [0, 80],
    5: [0, 160],
    3: [1, 2],
    6: [1, 2],
    7: [1, 2],
    8: [1, 2],
    9: [1, 2],
    0: [1, 2],
}


def dist_vect(v1, v2):
    return np.sqrt((v1[0]-v2[0])**2 + (v1[1]-v2[1])**2)


def locate_bot(id_, distance, theta):
    a = id_to_cordinate[id_]
    x1 = a[0]
    y1 = a[1]
    xf, yf = 0, 0
    if id_ in list1:
        xf = x1+distance*np.sin(theta)
        yf = y1+distance*np.cos(theta)  # here y1 is 0 identiacally

    if id_ in list2:
        xf = x1 - distance*np.cos(theta)
        yf = y1 - distance*np.sin(theta)

    if id_ in list3:
        xf = x1-distance*np.sin(theta)
        yf = y1-distance*np.cos(theta)

    if id_ in list4:
        xf = x1+distance*np.cos(theta)  # here x1 = 0 identically
        yf = y1 - distance*np.sin(theta)

    return xf, yf


calib_data_path = "../calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 9.5  # centimeters

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(marker_dict, params)

cap = cv.VideoCapture(0)


def checkFrame():
    pub = rospy.Publishe0r('t1', Int32MultiArray, queue_size=10)
     
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random    
    # numbers are added to the end of the name.
    rospy.init_node('coordinates', anonymous=True)
     
    # Go through the loop 10 times per second
    rate = rospy.Rate(10) # 10hz
    ret, frame = cap.read()
    while not rospy.is_shutdown():

        if not ret:
            print("ret issue")
            return "Done"

        # Processing the image
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(
            gray_frame)

        if markerCorners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                markerCorners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, markerIds.size)
            for ids, corners, i in zip(markerIds, markerCorners, total_markers):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0,
                                                            255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
                # so I have rectified that mistake, I have test that out it increase the accuracy overall.
                # Calculating the distance
                distance = 1.32*np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
                theta_rod = np.sqrt(
                    rVec[i][0][2] ** 2 + rVec[i][0][0] ** 2 + rVec[i][0][1] ** 2
                )
                v = [rVec[i][0][0]/theta_rod, rVec[i][0]
                    [1]/theta_rod, rVec[i][0][2]/theta_rod]
                theta_x = 180/np.pi*(np.arctan(2*rVec[i][0][0]/v[0]))
                theta_y = 180/np.pi*(np.arctan(2*rVec[i][0][1]/v[1]))
                theta_z = 90 - 180/np.pi*(np.arctan(2*rVec[i][0][2]/v[2]))

                location = locate_bot(ids[0], distance, -theta_z)

                print(location, distance, theta_z)

                # Draw the pose of the marker
                point = cv.drawFrameAxes(
                    frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                cv.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
                cv.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )

        cv.imshow("frame", frame)
        msg = Int32MultiArray(data=[location[0], location[1], distance, ids])
        pub.publish(msg)
        rate.sleep()
        key = cv.waitKey(1)
        if key == ord('q'):
            print("q received")
            return "Done"


while True:
    if checkFrame() == "Done":
        break


cap.release()
cv.destroyAllWindows()
