# -*- coding: utf-8 -*-
"""
Python script to create a new session and folder structure with it

./Sessions is the folder where all the sessions are saved

./Session/{SESSION_ID} is the folder of a particular session

./Session/{SESSION_ID}/readMe.txt is needed to annotate eventual comments to the session

./Session/{SESSION_ID}/ZED contains all file relative to the zed:
    ./Session/{SESSION_ID}/ZED/checkerBoard.PNG file needed to calibrate the zed with opencv
    ./Session/{SESSION_ID}/ZED/tcpMessage.txt contains the tcp message encoded for unity to unwrapp the required values
   ./Session/{SESSION_ID}/ZED/sunDir/ contains the sun direction in view space and clip space
    
./Session/{SESSION_ID}/iPhone contains all file relative to the iPhone and ARKit:
    ./Session/{SESSION_ID}/iPhone/checkerBoard.PNG file needed to calibrate the iPhone with opencv
    ./Session/{SESSION_ID}/iPhone/ARKitCam_2_ZEDCam.txt contains the matrix value
    ./Session/{SESSION_ID}/iPhone/ARKitWorld_2_ARKitCam.txt contains the matrix value
    ./Session/{SESSION_ID}/iPhone/GPS_coords.txt contains the latitude, longitud, altitude, timestamp
    ./Session/{SESSION_ID}/iPhone/Zenith_Azimuth.txt contains the latitude, longitud, altitude, timestamp
    
./Session/{SESSION_ID}/images constains all the images recored from the zed
    ./Session/{SESSION_ID}/images/color/ 
    ./Session/{SESSION_ID}/images/normals/
    ./Session/{SESSION_ID}/images/depth/
    ./Session/{SESSION_ID}/images/shadows/

./Session/{SESSION_ID}/{SESSION_ID}.svo the svo file recorded
./Session/{SESSION_ID}/{SESSION_ID}/meshes/mesh1.obj object obtained with spatial mapping



WORKFLOW:
    0) connect PC and iPhone7 to iPhone5 hotspot
    1) newSession.py is launched
    2) iOS application is launched
    3) button to send position relative to UP-NORD-EST
    4) button to send checkerboard from iOS
    5) launch zed application
    6) button to send checkerboard from ZED
    7) record .svo
    8) set the correct sessionID

"""

import os
import socket
import numpy as np
import cv2
import sys
sys.path.insert(0, './sun-position/')

from sunposition import sunpos
from datetime import datetime



# zed intrinsic parameters
cameraMatrix_ZED = np.zeros((3,3), np.float64)
cameraMatrix_ZED[0,0] = 605.516
cameraMatrix_ZED[0,2] = 674.885
cameraMatrix_ZED[1,1] = 603.069
cameraMatrix_ZED[1,2] = 350.466
cameraMatrix_ZED[2,2] = 1

distCoeff_ZED = np.zeros((1,5), np.float64)
distCoeff_ZED[0,0] = 0.0468465
distCoeff_ZED[0,1] = -0.140856
distCoeff_ZED[0,2] = 0.00864799
distCoeff_ZED[0,3] = 0.00208945
distCoeff_ZED[0,4] =  0.0933369

# iphone intrinsic parameters
cameraMatrix_iPhone = np.zeros((3,3), np.float64)
cameraMatrix_iPhone[0,0] = 1055.26
cameraMatrix_iPhone[0,2] = 364.98
cameraMatrix_iPhone[1,1] = 1055.63
cameraMatrix_iPhone[1,2] = 644.366
cameraMatrix_iPhone[2,2] = 1

distCoeff_iPhone = np.zeros((1,5), np.float64)
distCoeff_iPhone[0,0] = 0.256208
distCoeff_iPhone[0,1] = -1.79925	
distCoeff_iPhone[0,2] = -0.00473381
distCoeff_iPhone[0,3] = -0.00554958
distCoeff_iPhone[0,4] = 3.35488

			



ZEDCam_2_Chkb = np.zeros((4,4), np.float)
iPhoneCam_2_Chkb = np.zeros((4,4), np.float)
Chkb_2_ZEDCam = np.zeros((4,4), np.float)
Chkb_2_iPhoneCam = np.zeros((4,4), np.float)


def getCameraExtrinsic(images, mtx, dist):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
    objp = np.multiply(objp, 0.0246)
    
   
   
    tvecs = []
    rvecs = [] 
    
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
    
        # If found, add object points, image points (after refining them)
        if ret == True:
            
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            
            _, rvec, tvec, inliers = cv2.solvePnPRansac(objp,corners2, mtx, dist)
            rvecs.append(rvec)
            tvecs.append(tvec)
            
            cv2.namedWindow("img", cv2.WINDOW_NORMAL )        # Create window with freedom of dimensions
            cv2.resizeWindow("img", 2400, 1200)              # Resize window to specified dimensions
    
            # Draw and display the corners
            cv2.drawChessboardCorners(img, (9,6), corners,ret)
            cv2.imshow('img',img)
            cv2.waitKey(10)
    
    cv2.destroyAllWindows()
    
    
    return (rvecs, tvecs)

def getCameraMatrix(rvec, tvec):
    Cam_2_Chkb = np.zeros((4,4), np.float)
    Chkb_2_Cam = np.zeros((4,4), np.float)
    
    
    rotM_Chkb_2_Cam = cv2.Rodrigues(rvec)[0]
    
    Chkb_2_Cam[0:3, 0:3] = rotM_Chkb_2_Cam
    Chkb_2_Cam[0:3, 3] = tvec.ravel()
    Chkb_2_Cam[3,3] = 1
    
    Cam_2_Chkb = np.linalg.inv(Chkb_2_Cam)
    
    
    return Cam_2_Chkb, Chkb_2_Cam
    
    
    
def writeMatrix(matrix):
    
    string:str = ""
    for i in range(0,matrix.shape[0]):
        for j in range(0,matrix.shape[1]):
            string += "{:.9f}".format(matrix[i,j])
            if(j != 3):
                string += "="
        if(i != 3):
                string += "!"
    
    return string

def ZEDServerSendDataReceiveCheckerBoard(folderZED, folderARKit, ip, port, sendTCP):
    
    s = socket.socket()
    s.bind((ip,port))
    s.listen(1)
    print("listening")
    c, addr = s.accept()
    print ("Connection from: " + str(addr))
    
    receivedData = c.recv(16777216)
    
    
    #receivedData =  np.asarray(bytearray(receivedData), dtype=np.uint8)
    x = np.frombuffer(receivedData, dtype='uint8')
    
    #decode the array into an image
    img = cv2.imdecode(x, 1)
    
    cv2.imwrite(folderZED + 'checkerBoard.png', img)
#    if sendTCP:
#        sendDataEncoded = tcpString.encode('utf-8')
#        c.send(sendDataEncoded)
#        
    print("shut down")
    s.close()
    
    image_ZED = [folderZED + "checkerBoard.png"]
    image_iPhone = [folderARKit + "checkerBoard.png"]

    # get extrinsics iPhone and  ZED
    rvecs_ZED, tvecs_ZED = getCameraExtrinsic(image_ZED, cameraMatrix_ZED, distCoeff_ZED)
    rvecs_iPhone, tvecs_iPhone = getCameraExtrinsic(image_iPhone, cameraMatrix_iPhone, distCoeff_iPhone)
    
    
    # get single camera2checkerboard matrices
    ZEDCam_2_Chkb[:,:], Chkb_2_ZEDCam[:,:] = getCameraMatrix(rvecs_ZED[0], tvecs_ZED[0])
    iPhoneCam_2_Chkb[:,:], Chkb_2_iPhoneCam[:,:] = getCameraMatrix(rvecs_iPhone[0], tvecs_iPhone[0])
    
    # get matrix from iPhone to Zed
    iPhoneCam_2_ZEDCam = np.matmul(Chkb_2_ZEDCam[:,:], iPhoneCam_2_Chkb[:,:])

    # encode the matrix in string
    iPhoneCam_2_ZEDCam_matrix_string = writeMatrix(iPhoneCam_2_ZEDCam)
   
    
    # read the other strings from file:
    #    1) ARKitWorld_2_ARKitCam
    #    1) azimuth and zenith angles
    arkitMatrixFile = open(folderARKit + "ARKitWorld_2_ARKitCam.txt", "r")
    arkitMatrixString = arkitMatrixFile.readlines()
    arkitMatrixFile.close()
    
    
    anglesFile = open(folderARKit + "Zenith_Azimuth.txt", "r")
    anglesString = anglesFile.readlines()
    anglesFile.close()
    
    
    
    tcpString = iPhoneCam_2_ZEDCam_matrix_string + "?" + arkitMatrixString[0] +"?" + anglesString[0]
    
    
    
    fileTxt = open(folderZED + "tcpMessage.txt", "a+")
    fileTxt.write(tcpString)
    fileTxt.close()
    
    

def ARKitCheckerBoardReceive(folderCheckerBoarARKit, ip, port):
    
    s = socket.socket()
    s.bind((ip,port))
    s.listen(1)
    print("listening")
    c, addr = s.accept()
    print ("Connection from: " + str(addr))
    
    while 1:
        receivedData = c.recv(16777216)
        x = np.frombuffer(receivedData, dtype='uint8')
        
        #decode the array into an image
        img = cv2.imdecode(x, 1)
        imgTransposed = np.zeros((img.shape[1], img.shape[0]), np.uint8)
        
        imgTransposed = cv2.transpose(img);
        imgTransposed = cv2.flip(imgTransposed, 0);
        cv2.imwrite(folderCheckerBoarARKit + 'checkerBoard.png', imgTransposed)
        break
    
    print("shut down")
    s.close()
    

def ARKitDataReceive(folderARKit, ip, port):
    s = socket.socket()
    s.bind((ip,port))
    s.listen(1)
    print("listening")
    c, addr = s.accept()
    print ("Connection from: " + str(addr))
    
    receivedData = c.recv(1024)
    
    receivedDataStr = receivedData.decode('utf-8')
    matrixStr, gpsStr = receivedDataStr.split('?')
    
    fileTxt = open(folderARKit + "ARKitWorld_2_ARKitCam.txt", "a+")
    fileTxt.write(matrixStr)
    fileTxt.close()
    
    lat, lon, alt, horAcc, timeStamp = gpsStr.split("!")
    time = datetime.utcnow()
    az, zen, ra, dec, h = sunpos(time,float(lat),float(lon),float(alt))
    
    sunPosStr = str(az) + "!" + str(zen)
    fileTxt = open(folderARKit + "Zenith_Azimuth.txt", "a+")
    fileTxt.write(sunPosStr)
    fileTxt.close()
    
    fileTxt = open(folderARKit + "GPS_Coord.txt", "a+")
    fileTxt.write("latitude: " + lat + "\nlongitude: " + lon +  "\naltitude: " + alt + "\nhorizontal accuracy: " + horAcc + "\ntime stamp: " + timeStamp)
    fileTxt.close()
    
    s.close()
    print("shut down")



sessionID = input("Enter session ID: ")
sessionPath = "./Sessions/" + sessionID + "/"
if os.path.exists(sessionPath):
    print("Directory with session ID already exist!")
else:
    os.makedirs(sessionPath)
    
    zedFolder = sessionPath + "ZED/"
    iPhoneFolder = sessionPath + "iPhone/"
    imagesFolder = sessionPath + "images/"
    meshFolder = sessionPath + "meshes/"
    singleFramesFolder = sessionPath + "singleFrames/"
    colorPath = imagesFolder + "color/"
    depthPath = imagesFolder + "depth/"
    normalPath = imagesFolder + "normals/"
    shadowPath = imagesFolder + "shadows/"
    sunDirPath = sessionPath + "sunDirection/"
    
    
    os.makedirs(zedFolder)
    os.makedirs(iPhoneFolder)
    os.makedirs(imagesFolder)
    os.makedirs(meshFolder)
    
    os.makedirs(singleFramesFolder)
    os.makedirs(sunDirPath)
    os.makedirs(shadowPath)
    os.makedirs(colorPath)
    os.makedirs(depthPath)
    os.makedirs(normalPath)
    
    
    ARKitDataReceive(iPhoneFolder, "172.20.10.2", 5004)
    ARKitCheckerBoardReceive(iPhoneFolder, "172.20.10.2", 5005)
    ZEDServerSendDataReceiveCheckerBoard(zedFolder, iPhoneFolder, "127.0.0.1", 5001, 0)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    