# -*- coding: utf-8 -*-
import base64
import io
import copy
import socket
import struct
import threading
from PID import *
from Face import *
import numpy as np
from Thread import *
from PIL import Image
from Command import COMMAND as cmd

class Client:
    def __init__(self):
        self.face = Face()
        self.pid=Incremental_PID(1,0,0.0025)
        self.udp_flag=False
        self.video_flag=True
        self.ball_flag=False
        self.face_flag=False
        self.face_id = False
        self.image=''
    def turn_on_client(self,ip):
        self.client_socket1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)       # UDP client
        self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 65538)
        print (ip)
    def turn_off_client(self):
        try:
            self.client_socket.shutdown(2)
            self.client_socket1.shutdown(2)
            self.client_socket.close()
            self.client_socket1.close()
        except Exception as e:
            print(e)
    def is_valid_image_4_bytes(self,buf): 
        bValid = True
        if buf[6:10] in (b'JFIF', b'Exif'):     
            if not buf.rstrip(b'\0\r\n').endswith(b'\xff\xd9'):
                bValid = False
        else:        
            try:  
                Image.open(io.BytesIO(buf)).verify() 
            except:  
                bValid = False
        return bValid
    def Looking_for_the_ball(self):
        MIN_RADIUS=10
        #red
        THRESHOLD_LOW = (0, 200, 200)
        THRESHOLD_HIGH = (5,255,255)

        img_filter = cv2.GaussianBlur(self.image.copy(), (3, 3), 0)
        img_filter = cv2.cvtColor(img_filter, cv2.COLOR_BGR2HSV)
        img_binary = cv2.inRange(img_filter.copy(), THRESHOLD_LOW, THRESHOLD_HIGH)
        img_binary = cv2.dilate(img_binary, None, iterations = 1)
        contours = cv2.findContours(img_binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        radius = 0
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] > 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius < MIN_RADIUS:
                    center = None
        if center != None:
            cv2.circle(self.image, center, int(radius), (0, 255, 0))
            D=round(2700/(2*radius))  #CM
            x=self.pid.PID_compute(center[0])
            d=self.pid.PID_compute(D)
            if radius>15:
                if d < 20:
                        command=cmd.CMD_MOVE_BACKWARD+"#"+self.move_speed+'\n'
                        self.send_data(command)
                        #print (command)
                elif d > 30:
                        command=cmd.CMD_MOVE_FORWARD+"#"+self.move_speed+'\n'
                        self.send_data(command)
                        #print (command)
                else:
                    if x < 70:
                        command=cmd.CMD_TURN_LEFT+"#"+self.move_speed+'\n'
                        self.send_data(command)
                        #print (command)
                    elif x>270:
                        command=cmd.CMD_TURN_RIGHT+"#"+self.move_speed+'\n'
                        self.send_data(command)
                        #print (command)
                    else:
                        command=cmd.CMD_MOVE_STOP+"#"+self.move_speed+'\n'
                        self.send_data(command)
                        #print (command)
        else:
            command=cmd.CMD_MOVE_STOP+"#"+self.move_speed+'\n'
            self.send_data(command)
            #print (command)
    def receiving_video(self,ip):
        stream_bytes = b' '
        try:
            self.client_socket.connect((ip, 8001))
            self.connection = self.client_socket.makefile('rb')
        except:
            #print ("command port connect failed")
            pass

        fps, st, frames_to_count, cnt = (0, 0, 20, 0)
        while True:
            packet, _ = self.client_socket.recvfrom(65536)
            data = base64.b64decode(packet, ' /')
            npdata = np.fromstring(data, dtype=np.uint8)
            frame = cv2.imdecode(npdata, 1)
            frame = cv2.putText(frame, 'FPS: ' + str(fps), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("RECEIVING VIDEO", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.client_socket.close()
                break
            if cnt == frames_to_count:
                try:
                    fps = round(frames_to_count / (time.time() - st))
                    st = time.time()
                    cnt = 0
                except:
                    pass
            cnt += 1
        '''
        while True:
            try:
                stream_bytes= self.connection.read(4)
                leng=struct.unpack('<L', stream_bytes[:4])
                jpg=self.connection.read(leng[0])
                if self.is_valid_image_4_bytes(jpg):
                    if self.video_flag:
                        self.image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if self.ball_flag and self.face_id==False:
                           self.Looking_for_the_ball()
                        elif self.face_flag and self.face_id==False:
                            self.face.face_detect(self.image)
                        self.video_flag=False
            except BaseException as e:
                print (e)
                break
        '''
    def send_data(self,data):
        if self.udp_flag:
            try:
                self.client_socket1.send(data.encode('utf-8'))
            except Exception as e:
                print(e)
    def receive_data(self):
        data=""
        data=self.client_socket1.recv(1024).decode('utf-8')
        return data
 
if __name__ == '__main__':
    pass
