#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import urllib2
import json

def decodeGMapPolylineEncoding(asciiEncodedString):
#    print "\nExtrapolating WKT For:"
#    print asciiEncodedString

    strLen = len(asciiEncodedString)

    index = 0
    lat = 0
    lng = 0
    coordPairString = ""
    latString = []
    lonString = []
    countOfLatLonPairs = 0
    firstLatLonPair = ""
    gotFirstPair = False

    while index < strLen:
        shift = 0
        result = 0
        if countOfLatLonPairs >= 1:
            coordPairString += ","
			
        stayInLoop = True
        while stayInLoop:                                                # GET THE LATITUDE
            b = ord(asciiEncodedString[index]) - 63
            result |= (b & 0x1f) << shift
            shift += 5
            index += 1

            if not b >= 0x20:
                stayInLoop = False

        # Python ternary instruction..
        dlat = ~(result >> 1) if (result & 1) else (result >> 1)
        lat += dlat

        shift = 0
        result = 0

        stayInLoop = True
        while stayInLoop:                                                # GET THE LONGITUDE
            b = ord(asciiEncodedString[index]) - 63
            result |= (b & 0x1f) << shift
            shift += 5
            index += 1

            if not b >= 0x20:
                stayInLoop = False

        # Python ternary instruction..
        dlng = ~(result >> 1) if (result & 1) else (result >> 1)
        lng += dlng

        lonNum = lng * 1e-5
        latNum = lat * 1e-5
        latString.append(latNum)
        lonString.append(lonNum)
		
        if gotFirstPair == False:
            gotFirstPair = True
            firstLatLonPair = str(lonNum) + " " + str(latNum)

        countOfLatLonPairs += 1
	wkt = zip(latString,lonString)
    return wkt
    
orgLat = "19.133798";			#hard-coded for now, later to be replaced by GPS input for origin
orgLon = "72.918324";
destLat = "19.130508";
destLon = "72.918962";
orgCo = orgLat+","+orgLon;
destCo = destLat+","+destLon;
str_origin = "origin="+orgCo;
str_dest = "destination="+destCo;
sensor = "sensor=false";
mode = "mode=driving";
alter = "alternatives=true";
parameters = str_origin+"&"+str_dest+"&"+sensor+"&"+mode+"&"+alter;
output = "json";
url2 = "https://maps.googleapis.com/maps/api/directions/"+output+"?"+parameters;
        
content = urllib2.urlopen(url2).read() #this is a string
cont = json.loads(content)				#this is a dictionary
rues = cont['routes']
total = 0 
for tmp_object in rues:
	total = total +1 
minIndex = 0
minDist = 99999
for i in range(total):
	dist = cont['routes'][i]['legs'][0]['distance']['value']
 	if int(dist) < int(minDist):
		minDist = dist
		minIndex = i

ssum = cont['routes'][int(minIndex)]['overview_polyline']['points'];
coors = decodeGMapPolylineEncoding(ssum)

def talker():
    pub = rospy.Publisher('gps_pnts', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        rospy.loginfo(str(coors))
        pub.publish(str(coors))
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
