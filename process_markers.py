import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math


class Localizer:

    DICTIONARYID = 0
    MARKERLENGTH = 0.053  # in meters
    XML_FILENAME = "calibration_data1.xml"

    def is_rotation_matrix(self, R):
        Rt = np.transpose(R)
        should_be_identity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - should_be_identity)
        return n < 1e-6

    # Calculates rotation matrix to euler angles x y z
    def rotation_matrix_to_euler_angles(self, R):
        assert (self.is_rotation_matrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def __init__(self):
        self.cam = cv.VideoCapture(1)
        cv.namedWindow("HD Pro Webcam C920")
        # read camera calibration data
        fs = cv.FileStorage(self.XML_FILENAME, cv.FILE_STORAGE_READ)
        if not fs.isOpened():
            print("Invalid camera file")
            exit(-1)
        self.camMatrix = fs.getNode("camera_matrix").mat()
        self.distCoeffs = fs.getNode("distortion_coefficients").mat()

    def pose_from_camera(self):
        _, input_image = self.cam.read()
        if input_image is not None:
            # detect markers from the input image
            dictionary = aruco.Dictionary_get(self.DICTIONARYID)
            parameters = aruco.DetectorParameters_create()
            marker_corners, marker_ids, _ = aruco.detectMarkers(input_image, dictionary, parameters=parameters)

            if marker_ids is not None:
                # find index of center marker
                index = 0
                index1 = 0
                for i in range(len(marker_ids)):
                    if marker_ids[i] == 0:
                        index = i
                    elif marker_ids[i] == 1:
                        index1 = i

                # pose estimation
                if len(marker_ids) > 1:
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        marker_corners, self.MARKERLENGTH, self.camMatrix, self.distCoeffs)

                    # check if tvecs is none
                    if tvecs is not None:
                        # translate tvecs from relation to camera to a marker
                        tvecs[index][0] -= tvecs[index1][0]

                        # get angle from rotational matrix
                        # convert rotational vector rvecs to rotational matrix
                        # convert euler in relation to a marker
                        rmat = np.empty([3, 3])
                        cv.Rodrigues(rvecs[index][0], rmat)
                        rmat_0 = np.empty([3, 3])
                        cv.Rodrigues(rvecs[index1][0], rmat_0)
                        euler_angle = self.rotation_matrix_to_euler_angles(rmat)
                        euler_angle1 = self.rotation_matrix_to_euler_angles(rmat_0)
                        euler_angle = euler_angle - euler_angle1

                        # display annotations (IDs and pose)
                        image_copy = input_image.copy()
                        cv.putText(image_copy, "Cozmo Pose", (10, 20), cv.FONT_HERSHEY_PLAIN, 1, (255, 0, 0, 0))
                        msg = "X(m): " + str(tvecs[index][0][0])
                        cv.putText(image_copy, msg, (10, 45), cv.FONT_HERSHEY_PLAIN, 1, (255, 0, 0, 0))
                        msg = "Y(m): " + str(tvecs[index][0][1])
                        cv.putText(image_copy, msg, (10, 70), cv.FONT_HERSHEY_PLAIN, 1, (255, 0, 0, 0))
                        msg = "Angle(rad): " + str(euler_angle[2])
                        cv.putText(image_copy, msg, (10, 95), cv.FONT_HERSHEY_PLAIN, 1, (255, 0, 0, 0))
                        aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
                        cv.imshow("Detect Markers", image_copy)
                        cv.waitKey(100)

                        # return the euler x, y, z coordinates and euler angles
                        return tvecs[index][0], euler_angle
                    else:
                        return None, None
                else:
                    return None, None
            else:
                return None, None
        else:
            return None, None
