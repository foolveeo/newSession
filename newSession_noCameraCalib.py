# -*- coding: utf-8 -*-
"""
Created on Wed May 30 15:45:31 2018

@author: Fulvio Bertolini
"""

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
import sys
sys.path.insert(0, './sun-position/')

from sunposition import sunpos
from datetime import datetime


    
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
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    