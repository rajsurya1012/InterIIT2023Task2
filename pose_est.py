import numpy as np
import cv2
import sys
import time

class camera_pose():
    def __init__(self):
        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }

        self.aruco_type = "DICT_4X4_250"

        self.arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.aruco_type])

        self.arucoParams = cv2.aruco.DetectorParameters_create()

        # self.intrinsic_camera = np.array(
        #     [[1.68271154e+03, 0.00000000e+00, 8.49231882e+02],
        #     [0.00000000e+00, 1.67214169e+03, 5.28968275e+02],
        #     [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        # self.distortion = np.array(
        #     [-0.39499519,  0.18129408,  0.00185536,  0.00307922, -0.03505981])

        self.intrinsic_camera = np.array(
            [[1.74266348e+03, 0.00000000e+00, 8.55750077e+02],
            [0.00000000e+00, 1.73815941e+03, 5.54921581e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.distortion = np.array(
            [-0.50980866,  1.00789444, -0.00915618,  0.00429768, -1.2860852])

            


        # self.cap = cv2.VideoCapture(0)
        self.cap = cv2.VideoCapture(0)
        self.flag = 0

        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        frame_width = 1920
        frame_height = 1080
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        self.tvec = np.array([])
        
        # Scaling
        # 60cm on ground = 34cm on camera in x
        # self.xScale = 1.765
        self.xScale = 1
        # 60cm on ground = 33cm on camera in y
        # self.yScale = 1.765
        self.yScale = 1
        # 68 cm on ground = 43cm on camera in z
        # self.zScale = 1.582
        self.zScale = 1

        #videowriter
        
        self.frames = cv2.VideoWriter('drone_vid.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 15, (frame_width,frame_height))


    def aruco_display(self, corners, ids, rejected, image):

        if len(corners) > 0:

            ids = ids.flatten()

            for (markerCorner, markerID) in zip(corners, ids):

                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

                cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)
                print("[Inference] ArUco marker ID: {}".format(markerID))

        return image


    def pose_estimation(self,frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
            gray, cv2.aruco_dict, parameters=parameters)

        if len(corners) > 0:
            for i in range(0, len(ids)):

                rvec, self.tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], 0.02, matrix_coefficients, distortion_coefficients)
                cv2.aruco.drawDetectedMarkers(frame, corners)
                # print(self.tvec)

                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, self.tvec, 0.01)

        return frame

    def getPose(self):
    


        # while self.cap.isOpened():

        #     ret, img = self.cap.read()
        #     time.sleep(0.1)

        #     output = self.pose_estimation(
        #         img, self.ARUCO_DICT[self.aruco_type], self.intrinsic_camera, self.distortion)

        #     cv2.imshow('Estimated Pose', output)
        #     check = np.shape(self.tvec)
        #     # cv2.imshow('Estimated Pose', output)
        #     # cv2.waitKey(2)

        #     if  check[0] != 0:
        #         self.out = self.tvec[0,0,:]
        #         self.out[0] *= self.xScale
        #         self.out[1] *= self.yScale
        #         self.out[2] *= self.zScale
        #         print(self.out)
        #         self.flag = 1
                
        #         self.frames.write(output)
                
        #     else:
        #         print('Marker not detected')

        #     key = cv2.waitKey(1) & 0xFF
        #     if key == ord('q'):
        #         break

        # self.cap.release()
        # cv2.destroyAllWindows()

        if self.cap.isOpened():

            self.flag = 0

            while self.flag == 0:


                ret, img = self.cap.read()

                output = self.pose_estimation(
                    img, self.ARUCO_DICT[self.aruco_type], self.intrinsic_camera, self.distortion)
                
                check = np.shape(self.tvec)
                # cv2.imshow('Estimated Pose', output)
                # cv2.waitKey(2)

                if  check[0] != 0:
                    self.out = self.tvec[0,0,:]
                    self.out[0] *= self.xScale
                    self.out[1] *= self.yScale
                    self.out[2] *= self.zScale
                    # print(self.out)
                    self.flag = 1
                    
                    self.frames.write(output)
                    return self.out
                    
                else:
                    print('Marker not detected')
            

        else:
            print('ERROR: Check your camera and port')

if __name__ == '__main__':
    pose = camera_pose()
    pose.getPose()
