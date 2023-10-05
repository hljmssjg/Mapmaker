"""
Example demonstrating how to communicate with Microsoft Robotic Developer
Studio 4 via the Lokarria http interface. 

Author: Erik Billing (billing@cs.umu.se)

Updated by Ola Ringdahl 2014-09-11
Updated by Lennart Jern 2016-09-06 (converted to Python 3)
Updated by Filip Allberg and Daniel Harr 2017-08-30 (actually converted to Python 3)
Updated by Thomas Johansson 2019-08-22 from Lokarriaexample.py to a class implementation
190904 thomasj fixed some errors in getHeading, getPosition. getHeading now returns and angle.
Updated by Ola Ringdahl 2019-11-21 Fixed some stuff for AI2
"""



import http.client, json, time
from math import sin, cos, pi, atan2
import quaternion
import sys

HEADERS = {"Content-type": "application/json", "Accept": "text/json"}


class UnexpectedResponse(Exception): pass


class Robot:
    def __init__(self, url="http://localhost:50000"):
        # HTTPConnection does not want to have http:// in the address apparently, so let's remove it:
        mrds_url = url[len("http://"):]
        self.url = mrds_url

    def getHeading(self):
        """Returns the heading angle, in radians, counterclockwise from the x-axis
        Note that the sign changes at pi radians, i.e. the heading goes from 0
        to pi, then from -pi back to 0 for a complete circuit."""
        pose = self._getPose()['Pose']['Orientation']
        heading = quaternion.heading(pose)
        return atan2(heading["Y"], heading["X"])

    def getPosition(self):
        """Returns the XY position as a two-element list"""
        pose = self._getPose()
        return pose['Pose']['Position']

    def setMotion(self, linearSpeed, turnrate):
        """ 
        Sends a speed and turn rate 
        command to the MRDS server
        speed is given in m/s, turn rate in radians/s
        """
        mrds = http.client.HTTPConnection(self.url, timeout=1)
        params = json.dumps({'TargetLinearSpeed': linearSpeed, 'TargetAngularSpeed': turnrate})
        try:
            mrds.request('POST', '/lokarria/differentialdrive', params, HEADERS)            
        except:
            print("Connection refused")
            sys.exit(-1)
        response = mrds.getresponse()
        status = response.status
        # response.close()
        if status == 204:
            return response
        else:
            raise UnexpectedResponse(response)
        pass

    def getLaser(self):
        """Requests the current laser scan from the MRDS server and parses it into a dict"""
        mrds = http.client.HTTPConnection(self.url, timeout=1)
        try:
            mrds.request('GET', '/lokarria/laser/echoes')
        except:
            print("Connection refused")
            sys.exit(-1)        
        response = mrds.getresponse()
        if (response.status == 200):
            laserData = response.read()
            response.close()
            return json.loads(laserData.decode())
        else:
            return response
        pass

    def getLaserAngles(self):
        """Requests the current laser properties from the MRDS server and parses it into a dict"""
        mrds = http.client.HTTPConnection(self.url, timeout=1)
        try:
            mrds.request('GET', '/lokarria/laser/properties')
        except:
            print("Connection refused")
            sys.exit(-1)      
        response = mrds.getresponse()
        if (response.status == 200):
            laserData = response.read()
            response.close()
            properties = json.loads(laserData.decode())
            beamCount = int((properties['EndAngle'] - properties['StartAngle']) / properties['AngleIncrement'])
            a = properties['StartAngle']  # +properties['AngleIncrement']
            angles = []
            while a <= properties['EndAngle']:
                angles.append(a)
                a += pi / 180  # properties['AngleIncrement']
            # angles.append(properties['EndAngle']-properties['AngleIncrement']/2)
            return angles
        else:
            raise UnexpectedResponse(response)

    # Local methods, not usually used outside of this class    
    def _getPose(self):
        """Reads the current position and orientation from the MRDS"""
        mrds = http.client.HTTPConnection(self.url, timeout=1)
        try:
            mrds.request('GET', '/lokarria/localization')
        except:
            print("Connection refused")
            sys.exit(-1) 
        response = mrds.getresponse()
        if (response.status == 200):
            poseData = response.read()
            response.close()
            return json.loads(poseData.decode())
        else:
            return UnexpectedResponse(response)
        pass
