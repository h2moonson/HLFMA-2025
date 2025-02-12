#!/usr/bin/env python3

import rospy
from datetime import datetime

from rtcm_msgs.msg import Message
from base64 import b64encode
from threading import Thread
import http.client

# IncompleteRead 오류 수정
def patch_http_response_read(func):
    def inner(*args):
        try:
            return func(*args)
        except http.client.IncompleteRead as e:
            return e.partial
    return inner
http.client.HTTPResponse.read = patch_http_response_read(http.client.HTTPResponse.read)

class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def run(self):
        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode((self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass)).encode()).decode()
        }
        connection = http.client.HTTPConnection(self.ntc.ntrip_server)
        connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga, headers)
        response = connection.getresponse()
        if response.status != 200: raise Exception("blah")
        buf = ""
        rmsg = Message()
        restart_count = 0
        while not self.stop:
            data = response.read(1)
            if len(data) != 0:
                if ord(data[0]) == 211:
                    buf += data
                    data = response.read(2)
                    buf += data
                    cnt = ord(data[0]) * 256 + ord(data[1])
                    data = response.read(2)
                    buf += data
                    typ = (ord(data[0]) * 256 + ord(data[1])) / 16
                    print(str(datetime.now()), cnt, typ)
                    cnt = cnt + 1
                    for x in range(cnt):
                        data = response.read(1)
                        buf += data
                    rmsg.message = buf
                    rmsg.header.seq += 1
                    rmsg.header.stamp = rospy.get_rostime()
                    self.ntc.pub.publish(rmsg)
                    buf = ""
                else: 
                    print(data)
            else:
                restart_count += 1
                print("Zero length ", restart_count)
                connection.close()
                connection = http.client.HTTPConnection(self.ntc.ntrip_server)
                connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga, headers)
                response = connection.getresponse()
                if response.status != 200: raise Exception("blah")
                buf = ""
        connection.close()

class ntripclient:
    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param('~rtcm_topic', 'rtcm')
        self.nmea_topic = rospy.get_param('~nmea_topic', 'nmea')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')
        self.nmea_gga = rospy.get_param('~nmea_gga')

        self.pub = rospy.Publisher(self.rtcm_topic, Message, queue_size=10)

        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True

if __name__ == '__main__':
    c = ntripclient()
    c.run()
