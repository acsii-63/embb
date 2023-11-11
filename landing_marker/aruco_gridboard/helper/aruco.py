import rospy
import os
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose,Vector3,Quaternion
from cv_bridge import CvBridge
import cv2
import yaml
import cv2.aruco as aruco
from tf.transformations import quaternion_from_matrix, translation_matrix, quaternion_matrix, euler_from_matrix
import numpy as np

bridge = CvBridge()

# Variables to store camera information
camera_info = None
# camera_matrix = np.array([1790.195479, 0, 705.965607, 0, 1786.073073, 509.041732, 0, 0, 1]).reshape(3,3)
# dist_coeffs = np.array([-0.221296,0.197511,-0.001090,-0.000714,0.000000])
camera_matrix = np.array([1800.3422192258558, 0.0, 677.3559856376415, 0.0, 1794.5150697044257, 520.4012438448146, 0.0, 0.0, 1.0]).reshape(3,3)
dist_coeffs = np.array([-0.23439610909332634, 0.21701958114249134, 9.828036369030322e-05, -0.002553866090183632, 0])
def camera_info_callback(msg):
    global camera_info, camera_matrix, dist_coeffs
    camera_info = msg
    camera_matrix = np.array(camera_info.K).reshape((3, 3))
    dist_coeffs = np.array(camera_info.D)

def rotation_matrix_z(angle):
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    rotation_matrix = np.array([
        [cos_theta, -sin_theta, 0],
        [sin_theta, cos_theta, 0],
        [0, 0, 1]
    ])
    return rotation_matrix

def image_callback(msg):
    # Convert the ROS Image message to OpenCV image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Process the image
    process_image(cv_image)
def first_step (x1, y1 , z1 , p1 , x2 , y2, z2 , p2):
    p21 = p1 + p2
    if abs(p21) > abs(p21-2*np.pi):
        p21 = p21-2*np.pi
    if abs(p21) > abs(p21+2*np.pi):
        p21 = p21+2*np.pi
    x21 = x1 + np.cos(p1)* x2 - np.sin(p1) * y2
    y21 = y1 + np.sin(p1)* x2 + np.cos(p1) * y2
    z21 = z1 + z2 
    return x21 , y21 , z21 , p21
def get_path (n ,i, j):
    remainders = []
    for num in range(n):
        if num != i and num != j:
            remainders.append(num)
    return remainders

def zRm (angle):
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle), 0],
                                [np.sin(angle),  np.cos(angle), 0],
                                [0       ,                   0, 1]])
    return rotation_matrix

def aruco_offset (angle , center ,  length):
    corner =  np.array([[-1, 1, 0.0], [ 1, 1 ,0],  [ 1,-1, 0],  [-1,-1, 0]],  dtype=np.float32)
    rot = zRm(angle)
    rotated_corner = []
    new_corner = []
    for c in corner:
        rotated_corner.append(length*np.dot(rot,c))
    for rc in rotated_corner:
        rc += center
        new_corner.append(rc)
    return new_corner

def process_image(cv_image,filename):
    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # gray = cv2.equalizeHist(gray)

    # Define the ArUco dictionary
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

    # Define the ArUco parameters
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect the markers in the image using camera calibration
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters,
                                              cameraMatrix=camera_matrix, distCoeff=dist_coeffs)
    processed_image_path = "/home/analys/aruco/"
    # Draw detected markers
    if ids is not None:
      cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

      if len(ids) > 1:
        for i in ids:
            for j in ids :
                if j <= i:
                    continue
                marker_index1 = np.where(ids == i)[0][0]
                marker_index2 = np.where(ids == j)[0][0]
                # Estimate pose for the marker
                rvec1, tvec1, _ = cv2.aruco.estimatePoseSingleMarkers(corners[marker_index1], 0.75, camera_matrix, dist_coeffs)
                rvec2, tvec2, _ = cv2.aruco.estimatePoseSingleMarkers(corners[marker_index2], 0.75, camera_matrix, dist_coeffs)

                cv2.aruco.drawAxis(cv_image, camera_matrix, dist_coeffs, rvec1, tvec1, 2.0)
                cv2.aruco.drawAxis(cv_image, camera_matrix, dist_coeffs, rvec2, tvec2, 2.0)

                cvec12 = tvec2 - tvec1
                cvec21 = tvec1 - tvec2

                R1, _ = cv2.Rodrigues(rvec1)
                R2, _ = cv2.Rodrigues(rvec2)
                # Compute the inverse of the rotation matrix to get the transpose
                R1_inv = np.linalg.inv(R1)
                R2_inv = np.linalg.inv(R2)
                # Compute primary unit vector
                X1C = R1[0, :] / np.linalg.norm(R1[0, :])
                Z1C = R1[2, :] / np.linalg.norm(R1[2, :])
                X2C = R2[0, :] / np.linalg.norm(R2[0, :])
                Z2C = R2[2, :] / np.linalg.norm(R2[2, :])
                # Compute the translation vector from marker1 to marker2 in the marker1 frame
                cvecM1 = np.dot(R1_inv, cvec12.reshape(3, 1))
                cvecM2 = np.dot(R2_inv, cvec21.reshape(3, 1))
                tvecM1 = np.dot(R1_inv, tvec1.reshape(3, 1))
                tvecM2 = np.dot(R2_inv, tvec2.reshape(3, 1))

                # Compute uX in opposite marker frame
                OPXM1 = np.dot(R1_inv,X2C)
                OPXM2 = np.dot(R2_inv,X1C)
                # Compute Opp yaw in marker frame
                yawM1 = np.arctan2(OPXM1[1],OPXM1[0])
                yawM2 = np.arctan2(OPXM2[1],OPXM2[0])
                # Parallel confidence
                PC = 0.8 + (Z1C.dot(Z2C))**2*0.2 - abs(1e-04*(tvecM1[2]+tvecM2[2])**2)*0.2
                PC1 = PC - abs(cvecM1[2])/8.0
                PC2 = PC - abs(cvecM2[2])/8.0
                # print(f"{i[0]},{j[0]},{np.squeeze(cvec_marker1_frame[0])},{np.s**queeze(cvec_marker1_frame[1])},{filename}",rvec2,rvec1)
                with open(processed_image_path+"aruco.txt", 'a') as file:
                    file.writelines(f"{i[0]},{j[0]},{np.squeeze(cvecM1[0])},{np.squeeze(cvecM1[1])},{0.5*np.squeeze(cvecM1[2])},{yawM1},{np.squeeze(PC1)},{filename}\n")
                    file.writelines(f"{j[0]},{i[0]},{np.squeeze(cvecM2[0])},{np.squeeze(cvecM2[1])},{0.5*np.squeeze(cvecM2[2])},{yawM2},{np.squeeze(PC2)},{filename}\n")
        cv2.imwrite(processed_image_path+filename, cv_image)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('aruco_detector')

    # with open('/home/analys/aruco/aruco.txt', 'w') as file:
    #     file.truncate(0)

    # # Subscribe to the camera image topic
    # folder_path = "/home/analys/Documents/full"

    # for filename in os.listdir(folder_path):
    #     if filename.endswith('.jpg') or filename.endswith('.png'):
    #         image_path = os.path.join(folder_path, filename)
    #         cv_image = cv2.imread(image_path)
    #         process_image(cv_image,os.path.basename(filename))

    numberMarker = 4

    value = np.zeros((numberMarker, numberMarker , 5))
    with open('/home/analys/aruco/aruco.txt', 'r') as file:
        lines = file.readlines()
        for line in lines:
            
            data = line.strip().split(',')
            value[int(data[0]),int(data[1]), 0]+= float(data[2])*float(data[6])
            value[int(data[0]),int(data[1]), 1]+= float(data[3])*float(data[6])
            value[int(data[0]),int(data[1]), 2]+= float(data[4])*float(data[6])
            angle = 0.0
            if(float(data[5])<0) :
                angle = 2 * np.pi + float(data[5])
            else :
                angle = float(data[5])
            value[int(data[0]),int(data[1]), 3]+= angle*float(data[6])
            value[int(data[0]),int(data[1]), 4]+= float(data[6])
            
    
    # print (value)

    for i in range(numberMarker):
        for j in range(numberMarker) :
            for k in range(4) :
                if j == i :
                    continue
                else :
                    if value[i,j,4] != 0 :
                        value[i,j,k] /= value[i,j,4]
                    else :
                        value[i,j,k] = -9999
            if value[i,j,3] > np.pi :
                value[i,j,3] -= 2*np.pi
    
    for i in range(numberMarker):
        for j in range(numberMarker) :
            if value[i,j,0] == -9999 :
                S = np.zeros(5)
                p = get_path(numberMarker,i,j)
                for k in p :
                    Xt , Yt , Zt , Pt = first_step (value[i,k,0],value[i,k,1],value[i,k,2],value[i,k,3],value[k,j,0],value[k,j,1],value[k,j,2],value[k,j,3])
                    CoCo = value[i,k,4]*value[k,j,4]/(value[i,k,4]+value[k,j,4])
                    S[4]+=CoCo
                    S[0]+=Xt*CoCo
                    S[1]+=Yt*CoCo
                    S[2]+=Zt*CoCo
                    S[3]+=Pt*CoCo
                    # print(Xt,Yt,Pt,i,j,CoCo)
                S[0]/=S[4]
                S[1]/=S[4]
                S[2]/=S[4]
                S[3]/=S[4]
                value[i,j,:]=S
    value[:, 2] /= 1 #0.975
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True, nanstr='NaN')
    print (value*1) #0.975
    print ("\n")
    for i in range(4):
        arr = aruco_offset(value[0,i,3],np.asarray([value[0,i,0],value[0,i,1],value[0,i,2]]),0.375)
        print("[", end="")
        for i, a in enumerate(arr):
            formatted_values = [f"{val:.3f}" for val in a]
            print(f"[{', '.join(formatted_values)}]", end="")
            if i != len(arr) - 1:
                print(",", end="")
        print("]")
        print ("\n")
    # print(value)
    # for i in range(4):
    #     translation_matrix = np.eye(4)
    #     translation_matrix[0][0] = np.cos(value[0,i,3])
    #     translation_matrix[0][1] = -np.sin(value[0,i,3])
    #     translation_matrix[1][0] = np.sin(value[0,i,3])
    #     translation_matrix[1][1] = np.cos(value[0,i,3])
    #     translation_matrix[0][3] = value[0,i,0]
    #     translation_matrix[1][3] = value[0,i,1]
    #     translation_matrix[2][3] = value[0,i,2]
    #     for row in translation_matrix:
    #         print('[', end=' ')
    #         print(', '.join(f'{x:.3f}' for x in row), end=' ')
    #         print('],')
    #     print("\n")

    # fx = camera_matrix[0, 0]
    # fy = camera_matrix[1, 1]
    # cx = camera_matrix[0, 2]
    # cy = camera_matrix[1, 2]
    # k1 = dist_coeffs[0]
    # k2 = dist_coeffs[1]
    # p1 = dist_coeffs[2]
    # p2 = dist_coeffs[3]

    # Printing the parameters
    # print(f"{fx},{fy},{cx},{cy},{k1},{k2},{p1},{p2}")
    # Start the main ROS loop
    data = {
    "camera_matrix": camera_matrix.tolist(),
    "distortion_coefficients": dist_coeffs.tolist()
    }

    # Write the dictionary to a YAML file
    with open("camera.yaml", "w") as f:
        yaml.dump(data, f)
    rospy.signal_shutdown()