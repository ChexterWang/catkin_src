import json
import cv2
import numpy as np
import base64
from pprint import pformat

class Util:

    def __init__(self, verbose=False, log=False) -> None:
        self.verbose = verbose
        self.save_log = log
        self.limitInterval = lambda u, a, b: b if u > b else a if u < a else u
        self.wsSend = lambda ws, msg: ws.send(json.dumps(msg))
        self.wsPub = lambda ws, topic, data: self.wsSend(ws, {
            'op': 'publish',
            'topic': topic,
            'msg':{
                'data': data,
            },
        })
        self.orb_tracking_status = {
            '-1': 'SYSTEM_NOT_READY',
            '0': 'NO_IMAGE_YET',
            '1': 'NOT_INITIALIZED',
            '2': 'OK',
            '3': 'LOST',
        }

    def log(self, filepath, str):
        if(self.save_log):
            with open(filepath, 'a') as f:
                f.write(str)
            f.close()

    def print(self, str):
        if(self.verbose):
            print(str)

    def parse(self, data):
        '''
        data: raw data
        format: <device_no>_<frame_no>_<msg>_<enc_img>_[kalib]_
        return: dictionary
        {
            'device_no': device_no,
            'frame_no': frame_no,
            'msg': msg,
            'enc_img': enc_img,
            'frame': img decoded from enc_img,
            'raw_kalib': raw kalib string,
            'kalib': numpy matrix of kalib,
        }
        '''
        split_data = data.split("_")
        has_kalib = len(split_data) == 6
        [device_no, frame_no, msg, enc_img] = split_data[:4]
        raw_kalib = split_data[4] if has_kalib else ''
        
        frame = bytes(enc_img,'utf-8')
        frame = base64.decodebytes(frame)
        frame = np.frombuffer(frame, np.int8)
        frame = cv2.imdecode(frame, -1)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        Kalib = np.zeros((3, 3))
        if(has_kalib):
            data = raw_kalib.split('+')
            if(len(data) == 4):
                '''
                        | fx 0  cx |
                kalib = | 0  fy cy |
                        | 0  0  1  |
                '''
                [fx, fy, cx, cy] = data
                fx, fy, cx, cy = float(fx), float(fy), float(cx), float(cy)
                Kalib[0,0] = fx
                Kalib[0,2] = cx
                Kalib[1,1] = fy
                Kalib[1,2] = cy
                Kalib[2,2] = 1
        ret = dict(
            device_no = device_no,
            frame_no = frame_no,
            msg = msg,
            enc_img = enc_img,
            frame = frame,
            raw_kalib = raw_kalib,
            kalib = Kalib,
        )
        ret_verbose = dict(ret)
        ret_verbose['enc_img'] = ret_verbose['enc_img'][:10]
        ret_verbose['frame'] = ret_verbose['frame'][0]
        self.print(f'[parse] [debug] return data:\n{pformat(ret_verbose)}')
        return ret