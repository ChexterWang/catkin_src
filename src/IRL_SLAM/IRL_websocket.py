#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import rospy
import json
import websockets
from uuid import uuid4
import asyncio
import numpy as np
import cv2
import base64
from MegaDepth.sample import Depth
# from GLPDepth.code.sample import Depth

from line_lbd.msg import final_pose
from line_lbd.msg import combined_boxes
from line_lbd.msg import My_image
from line_lbd.msg import updateServer
# from ORB_SLAM2.msg import Tracking_status
# from ORB_SLAM2.Examples.ROS.ORB_SLAM2.build.devel.lib.python3.dist-packages.ORB_SLAM2.msg import Tracking_status
from sys import path
path.insert(0, "/home/brian/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/devel/lib/python3/dist-packages/ORB_SLAM2")
from msg import Tracking_status
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import Virtual_Angle as VA
from Util import Util
from pprint import pformat

util = Util(verbose=False, log=False)
br = CvBridge()

class Client_Server:
    def __init__(self, topic):
        self.sub = rospy.Subscriber("/final_pose", final_pose, self.return_callback)
        self.update_sub = rospy.Subscriber("/update_pose_python", updateServer, self.update_callback)
        self.tracking_status_sub = rospy.Subscriber("/slam_track", Tracking_status, self.tracking_status_callback)
        self.net = Depth(640,480)
        self.topic = topic
        self.has_pose = False
        self.has_update = False
        self.has_host = False
        self.update_now = ""
        self.pose_now = ""
        self.save_ids = []
        self.file = "/home/brian/catkin_ws/src/IRL_SLAM/raw_data_seq.csv"
        self.experiment = ""
        self.host_frame_id = 0
        self.save_id = 0
        self.VO = VA.Virtual_Angle(True,False)
        self.set_host_done = False
        self.has_viewer = {}
        self.set_viewer_done = {}
        self.viewer_pose_msg = {}
        self.pose = {}
        self.tracking_status = {}
        self.has_initialize = False
        self.has_initialize_send = False
        self.relocalize = False
        self.assigned_id_now = 0
        self.pixel = True
        with open("/home/brian/catkin_ws/src/IRL_SLAM/raw_data_seq.csv", 'w') as f:
            f.write(f'frame_no,device_no,msg,enc_img,raw_kalib\n')
        f.close()

    def tracking_status_callback(self, data):
        device_id = data.id
        if(hasattr(self, 'tracking_status')):
            self.tracking_status[device_id] = data.tracking_status

    def update_callback(self, data):
        # print("get update")
        update_id = data.id
        pose_rot = data.rot
        pose_trans = data.trans
        total_data = "UP,{}".format(update_id)
        # for i in range(size):
        update_rot = self.parse_to_string(pose_rot)
        update_trans = self.parse_to_string(pose_trans)
        total_data+=",{},{}".format(update_rot,update_trans)
        self.update_now = total_data
        # print("update data: ",self.update_now)
        self.has_update = True

    def return_callback(self,data):
        device_id = data.id
        # self.relocalize = (id_=="reloc")
        Rot_tmp = data.rot
        Trans_tmp = data.trans
        count = data.count
        boxes = data.boxes
        # boxes_type = data.types
        # boxes_2d = data.boxes_2d
        # boxes_3d = data.boxes_3d 
        Rot = self.parse_to_string(Rot_tmp)
        Trans = self.parse_to_string(Trans_tmp)
        total_data = "BB_{}_[{}]_[{}]_{}".format(device_id,Rot,Trans,count)
        total_data_tmp = "BB_{}_[{}]_[{}]_{}".format(device_id,Rot,Trans,count)
        
        if(count==0):
            total_data+="_"
            total_data_tmp+="_"

        for i in range(count):
            box_type = self.parse_to_string(boxes[i].type_)
            box_2d = self.parse_to_string(boxes[i].BoundingBox2D)
            box_3d = self.parse_to_string(boxes[i].BoundingBox3D)
            box_class = boxes[i].class_name
            total_data+="_[{}]_[{}]_[{}]".format(box_type,box_2d,box_3d)
            total_data_tmp+="_{}_[{}]_[{}]_[{}]".format(box_class,box_type,box_2d,box_3d)

        cv_image = br.imgmsg_to_cv2(data.image_now, "rgb8")

        if(self.has_host):
            print(f"[set_host] start (id:{device_id})")
            self.VO.set_host(total_data_tmp,(320,240),1.0,cv_image,device_id)
            cv_image = br.imgmsg_to_cv2(data.image_now, "rgb8")
            depth_image = self.net.depth_estimate(cv_image)
            host_num = len(self.VO.host_classes)
            self.host_depth = depth_image.copy()
            # print(depth_image.shape)
            if(host_num>1):
                # util.log(self.file, f"host num: {host_num}, depth: 1.0\n")
                self.has_host = False
                self.set_host_done = True
        
        if(device_id in self.has_viewer and self.has_viewer[device_id]):
            # assertion: device_id in self.tracking_status
            if(device_id in self.tracking_status):
                self.pose[device_id] = self.VO.scene_check(total_data_tmp, device_id, self.tracking_status[device_id])
            else:
                util.print(f"[scene_check] device_id not in tracking_status, use 'Lost' instead")
                self.pose[device_id] = self.VO.scene_check(total_data_tmp, device_id, '3')
            
            if(self.pose[device_id]['done']):
                print(f"[scene_check] viewer (id:{device_id}) done")
                pixel = self.pose[device_id]['pixel']
                pos = self.pose[device_id]['pos']

                # TODO: add depth to return msg
                ''' depth
                # cv_image = br.imgmsg_to_cv2(data.image_now, "rgb8")
                # depth_image = self.net.depth_estimate(cv_image)
                # # cv2.imwrite('/home/brian/Documents/experiment/host_depth.png', self.host_depth*255)
                # # cv2.imwrite('/home/brian/Documents/experiment/view_depth.png', depth_image*255)
                # assume in host, the object was placed in air and the distance/depth of the object to camera is d1.
                # assume in host, the depth of mid point in frame to camera is d2.
                # assume in viewer, the depth of mid point in frame to camera is d3.
                # the retrieve depth of the object is then d1 * d3/d2.
                # depth_h = self.host_depth[self.VO.select_depth_center_host[0],self.VO.select_depth_center_host[1]]
                # # self.viewer_depth = 1*(depth_image[self.VO.select_depth_center_viewer[0],self.VO.select_depth_center_viewer[1]]/depth_h)
                # self.viewer_depth = depth_image[self.VO.select_depth_center_viewer[0],self.VO.select_depth_center_viewer[1]]
                # # self.viewer_depth = "input depth"*(depth_image[self.VO.select_depth_center_viewer[0],self.VO.select_depth_center_viewer[1]]/depth_h)
                # print(f"host depth: {depth_h:.4f}, viewer depth:{self.viewer_depth:.4f}")
                # # if(self.viewer_depth<=1):
                # #     self.viewer_depth = 1.2
                # # self.viewer_pose_msg = f"viewer_done,{pose[0]:.4f},{pose[1]:.4f},{self.viewer_depth:.4f}"
                # util.log(self.file, f"pose: {pose[0]:.4f}, {pose[1]:.4f}, depth ratio: {self.viewer_depth:.4f}, angle: {self.VO.sm_angle}\n")
                '''
                
                pixel_msg = f"viewer_done,{pixel[0]},{pixel[1]},{self.VO.sm_angle:.0f},{device_id}"
                pos_msg = f"viewer_done,{pos[0]},{pixel[1]},{self.VO.sm_angle:.0f},{device_id}"
                self.viewer_pose_msg[device_id] = pixel_msg if self.pixel else pos_msg
                self.set_viewer_done[device_id] = True
                self.has_viewer[device_id] = False

        if(self.has_initialize_send==False):
            self.has_initialize = True

        self.pose_now = total_data
        self.has_pose = True        
    
    def parse_to_string(self,data):
        return_data = ""
        for i in range(len(data)-1):
            return_data+=str(data[i])
            return_data+=","
        return_data+=str(data[len(data)-1])
        return return_data

    def set_host_frame_id(self,id_):
        self.host_frame_id = id_
        self.has_host = True

async def main_loop():
    async with websockets.connect('ws://localhost:9090') as websocket:
        rospy.init_node("test_publiser", anonymous=True)
        pub = rospy.Publisher('/camera/rgb/image_raw', Image,queue_size=10)
        my_pub = rospy.Publisher('/camera/rgb/My_image', My_image,queue_size=10)

        save_image = False
        topic = "/IRL_SLAM"
        server = Client_Server(topic)

        await util.wsSend(websocket, {
            'op':'advertise',
            'topic':topic,
            'type':'std_msgs/String'
        })

        await util.wsSend(websocket, {
            'op':'subscribe',
            'id': str(uuid4()),
            'topic':'/chatter',
            'type':'std_msgs/String',
            'queue_length':5
        })

        print(f"[System] Ready")

        while True:
            try:
                if(server.has_initialize):
                    print(f"[initialize] SLAM initialized")
                    await util.wsPub(websocket, topic, "initialize")
                    server.has_initialize_send = True
                    server.has_initialize = False
                
                if(server.has_update):
                    server.has_update = False
                    await util.wsPub(websocket, topic, server.update_now)
                
                json_message = await websocket.recv()
                recv_message = json.loads(json_message)
                raw_data = recv_message['msg']['data']
                
                if(len(raw_data)!=0):
                    data = util.parse(raw_data)
                    device_no = data['device_no']
                    frame_no = data['frame_no']
                    rcv_msg = data['msg']
                    frame = data['frame']
                    Kalib = data['kalib']
                    raw_kalib = data['raw_kalib']
                    enc_img = data['enc_img']

                    data_verbose = dict(data)
                    data_verbose['enc_img'] = data_verbose['enc_img'][:10]
                    data_verbose.pop('frame')
                    data_verbose.pop('kalib')
                    util.print(f'[parse] [debug] data saved to csv:\n{pformat(data_verbose)}')
                    util.log(server.file, f'{frame_no},{device_no},{rcv_msg},{enc_img},{raw_kalib},\n')

                    if(rcv_msg == "getId" and int(device_no) < 0):
                        # assign id to device
                        server.VO.Kalib[str(server.assigned_id_now)] = Kalib
                        util.print(f'[getId] [debug] kalib received {raw_kalib}')
                        print(f"[getId] assign id {server.assigned_id_now} to device")
                        await util.wsPub(websocket, topic, f"id,{server.assigned_id_now}")
                        server.assigned_id_now += 1
                        
                    if(rcv_msg == "viewer"):
                        if(not server.relocalize):
                            print(f"[retrieve] no host, send relocalize msg")
                            await util.wsPub(websocket, topic, f"relocalize")
                        else:
                            if(int(device_no) > 0):
                                server.has_viewer[device_no] = True
                                print(f"[scene_check] start for id {device_no}")

                    if(device_no in server.set_viewer_done and server.set_viewer_done[device_no]):
                        print(f"[scene_check] {server.viewer_pose_msg[device_no]}")
                        server.set_viewer_done[device_no] = False
                        await util.wsPub(websocket, topic, server.viewer_pose_msg[device_no])
                       
                    if(rcv_msg == "host"):
                        server.has_host = True

                    if(server.set_host_done):
                        await util.wsPub(websocket, topic, "host_set")
                        server.set_host_done = False
                        server.relocalize = True
                        
                    msg = br.cv2_to_imgmsg(frame, "rgb8")
                    msg.header.frame_id = frame_no 
                    pub.publish(msg)

                    My_msg = My_image()
                    My_msg.id = device_no
                    My_msg.image_now = msg
                    my_pub.publish(My_msg)

            except websockets.ConnectionClosed as e:
                print(f"[error] connection closed, {e}")
                return
            
            except TypeError as e:
                print(f"[error] type error, {e}")
                return
            
            except KeyError as e:
                print(f"[error] index error, {e}")
                return
            
            except Exception as e:
                print("[error] connected loss", e)
                return

if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main_loop())