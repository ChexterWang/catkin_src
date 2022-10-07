'''
Doyeon Kim, 2022
'''
import sys
sys.path.append('/home/brian/catkin_ws/src/IRL_SLAM/GLPDepth/code/')
import os
import cv2
import numpy as np
from collections import OrderedDict

import torch
import torch.backends.cudnn as cudnn
from torch.autograd import Variable

from models.model import GLPDepth
from skimage.transform import resize

class Depth():
    def __init__(self,height,width):
    
        device = torch.device('cuda')
        cudnn.benchmark = True

        model = GLPDepth(max_depth=80, is_train=False).to(device)
        model_weight = torch.load("/home/brian/catkin_ws/src/IRL_SLAM/GLPDepth/best_model_nyu.ckpt")
        if 'module' in next(iter(model_weight.items()))[0]:
            model_weight = OrderedDict((k[7:], v) for k, v in model_weight.items())
        model.load_state_dict(model_weight)
        model.eval()

        self.model = model
        self.input_height = height
        self.input_width  = width

    def depth_estimate(self,image):
        img = np.float32(image)/255.0
        # img = np.float32(io.imread(img_path))/255.0
        # print(img.shape)
        img = resize(img, (self.input_height, self.input_width), order = 1)
        input_img =  torch.from_numpy( np.transpose(img, (2,0,1)) ).contiguous().float()
        input_img = input_img.unsqueeze(0)

        input_images = Variable(input_img.cuda())

        with torch.no_grad():
            pred = self.model(input_images)
        pred = pred['pred_d']
        pred = torch.exp(pred)
        pred = 1/pred

        pred = pred.squeeze().cpu().numpy()
        pred = pred/np.amax(pred)

        return pred

