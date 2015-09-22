"""
Example demonstrating how to communicate with Microsoft Robotic Developer
Studio 4 via the Lokarria http interface. 

Author: Erik Billing (billing@cs.umu.se)

Updated by Ola Ringdahl 204-09-11
"""

MRDS_URL = 'localhost:50000'

import httplib, json, time
from math import sin,cos,pi,atan2,sqrt

HEADERS = {"Content-type": "application/json", "Accept": "text/json"}

with open('Path-around-table-and-back.json') as path_file:
    path = json.load(path_file)

class UnexpectedResponse(Exception): pass

def postSpeed(angularSpeed,linearSpeed):
    """Sends a speed command to the MRDS server"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    params = json.dumps({'TargetAngularSpeed':angularSpeed,'TargetLinearSpeed':linearSpeed})
    mrds.request('POST','/lokarria/differentialdrive',params,HEADERS)
    response = mrds.getresponse()
    status = response.status
    #response.close()
    if status == 204:
        return response
    else:
        raise UnexpectedResponse(response)

def getLaser():
    """Requests the current laser scan from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/echoes')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        return json.loads(laserData)
    else:
        return response
    
def getLaserAngles():
    """Requests the current laser properties from the MRDS server and parses it into a dict"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/laser/properties')
    response = mrds.getresponse()
    if (response.status == 200):
        laserData = response.read()
        response.close()
        properties = json.loads(laserData)
        beamCount = int((properties['EndAngle']-properties['StartAngle'])/properties['AngleIncrement'])
        a = properties['StartAngle']#+properties['AngleIncrement']
        angles = []
        while a <= properties['EndAngle']:
            angles.append(a)
            a+=pi/180 #properties['AngleIncrement']
        #angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
        return angles
    else:
        raise UnexpectedResponse(response)

def getPose():
    """Reads the current position and orientation from the MRDS"""
    mrds = httplib.HTTPConnection(MRDS_URL)
    mrds.request('GET','/lokarria/localization')
    response = mrds.getresponse()
    if (response.status == 200):
        poseData = response.read()
        response.close()
        return json.loads(poseData)
    else:
        return UnexpectedResponse(response)

def bearing(q):
    return rotate(q,{'X':1.0,'Y':0.0,"Z":0.0})

def rotate(q,v):
    return vector(qmult(qmult(q,quaternion(v)),conjugate(q)))

def quaternion(v):
    q=v.copy()
    q['W']=0.0;
    return q

def vector(q):
    v={}
    v["X"]=q["X"]
    v["Y"]=q["Y"]
    v["Z"]=q["Z"]
    return v

def conjugate(q):
    qc=q.copy()
    qc["X"]=-q["X"]
    qc["Y"]=-q["Y"]
    qc["Z"]=-q["Z"]
    return qc

def qmult(q1,q2):
    q={}
    q["W"]=q1["W"]*q2["W"]-q1["X"]*q2["X"]-q1["Y"]*q2["Y"]-q1["Z"]*q2["Z"]
    q["X"]=q1["W"]*q2["X"]+q1["X"]*q2["W"]+q1["Y"]*q2["Z"]-q1["Z"]*q2["Y"]
    q["Y"]=q1["W"]*q2["Y"]-q1["X"]*q2["Z"]+q1["Y"]*q2["W"]+q1["Z"]*q2["X"]
    q["Z"]=q1["W"]*q2["Z"]+q1["X"]*q2["Y"]-q1["Y"]*q2["X"]+q1["Z"]*q2["W"]
    return q
    
def getBearing():
    """Returns the XY Orientation as a bearing unit vector"""
    return bearing(getPose()['Pose']['Orientation'])

def getAngle(pr, pt):
    #return the angle between Heading vector and target position
    xr = pr['Pose']['Position']['X']
    yr = pr['Pose']['Position']['Y']
    xt = pt['Pose']['Position']['X']
    yt = pt['Pose']['Position']['Y']
    rAngle = atan2(getBearing()['Y'], getBearing()['X'])
    tAngle = atan2((yt - yr), (xt - xr))
    return tAngle - rAngle

def getDistance(pr, pt):
    #return distance between current position and target position
    xr = pr['Pose']['Position']['X']
    yr = pr['Pose']['Position']['Y']
    xt = pt['Pose']['Position']['X']
    yt = pt['Pose']['Position']['Y']
    distance = sqrt((yt-yr)**2 + (xt-xr)**2)
    return distance



    
if __name__ == '__main__':
    print 'Sending commands to MRDS server', MRDS_URL
    
    try:
        print 'Robot Start Running'
        #initial state
        L = 0.2
        targetIdx = 0
        currentPose = getPose()
        targetPose = path[targetIdx]
        linearSpeed = 0
        turningSpeed = 0
        robotRun = 1
        while robotRun == 1:
            #Find Target position
            while getDistance(getPose(), path[targetIdx]) < L :
                currentPose = getPose()
                targetPose = path[targetIdx]
                #if abs(getAngle(currentPose, targetPose)) > 0.3:    
                   #break
                targetIdx += 1
                if targetIdx >= len(path):
                    targetIdx = len(path)-1
                    break
            #Set speed due to positions
            print 'distance',getDistance(currentPose, targetPose)
            print 'angle',getAngle(currentPose, targetPose)
            if getAngle(currentPose, targetPose) == 0:
                linearSpeed = getDistance(currentPose, targetPose)/ 2
                respone = postSpeed(0, linearSpeed)
            elif abs(getAngle(currentPose, targetPose)) > 0.3:
                turningSpeed = getAngle(currentPose, targetPose)
                linearSpeed = 0
                respone = postSpeed(turningSpeed, linearSpeed)
            elif getAngle(currentPose, targetPose) != 0:
                turningSpeed = getAngle(currentPose, targetPose)
                linearSpeed = getDistance(currentPose, targetPose)
                respone = postSpeed(turningSpeed, linearSpeed)

            #check position to stop
            if targetIdx == len(path)-1:
                if getDistance(getPose(), path[targetIdx]) < 0.05:
                    respone = postSpeed(0, 0)
                    robotRun = 0
                    print 'Robot Stopped'
            #prevent crash using laser        
            if min(getLaser()['Echoes']) < 0.1:
                respone = postSpeed(0, 0)
                print 'Will crash'
                robotRun = 0
            time.sleep(0.5)
            
    except UnexpectedResponse, ex:
        print 'Unexpected response from server when reading position:', ex
    

    
