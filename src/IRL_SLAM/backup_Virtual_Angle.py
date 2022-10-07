import numpy as np
from numpy.linalg import inv
import cv2
import math

limitInterval = lambda u, a, b: b if u > b else a if u < a else u

class Virtual_Angle:
    def __init__(self,save_image,save_log):
        fx = 737.037
        fy = 699.167
        cx = 340.565
        cy = 218.486

        self.Kalib = np.zeros((3,3))
        self.Kalib[0,0] = fx
        self.Kalib[0,2] = cx
        self.Kalib[1,1] = fy
        self.Kalib[1,2] = cy
        self.Kalib[2,2] = 1

        self.colors = [(0, 0, 255),(0, 255, 0),(255, 0, 0),(255, 0, 255),(0, 255, 255),(255, 255, 255),(255,255,0),(0,0,0)]
        self.save_image = save_image
        self.save_log = save_log
        # self.angles = [40, 35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15, -20, -25, -30, -35, -40]
        self.angles = [40, 30, 20, 10, 0, -10, -20, -30, -40]
        self.angle_dict = {}
        self.host_numbers = 0
        self.center_world = np.zeros((3))
        self.center_world_x = np.zeros((3))
        self.center_world_y = np.zeros((3))
        self.center_world_z = np.zeros((3))
        self.host_classes = []
        self.depth = 0
        self.sm_angle = 0
        self.viewer_pixel = [340,240]
        self.select_depth_center_host = [340,240]
        self.select_depth_center_viewer = [340,240]
    def parse(self,data):
        parsed_data_tmp = []
    
        for i in range(5):
            tmp = data.find("_")
            parsed_data_tmp.append(data[0:tmp])
            data = data[tmp+1:]
        nums = int(parsed_data_tmp[-1])
        if(nums>0):
            for i in range(nums):
                for j in range(4):
                    tmp = data.find("_")
                    parsed_data_tmp.append(data[0:tmp])
                    # if(j==1):
                    #   print(data[0:tmp])
                    data = data[tmp+1:]

        self.Tcw = {}
        self.Tcw["type"] = parsed_data_tmp[0]
        self.Tcw["id"] = int(parsed_data_tmp[1])
        self.Tcw["Rot"] = np.array(parsed_data_tmp[2][1:-1].split(','),dtype = np.float64).reshape((3,3))
        self.Tcw["Trans"] = np.array(parsed_data_tmp[3][1:-1].split(','),dtype = np.float64)
        self.Tcw["nums"] = nums
        if(nums>0):
            start = 5
            for i in range(nums):
                tmp = {}
                tmp["class"] = parsed_data_tmp[start]
                tmp["3D_type"] = np.array(parsed_data_tmp[start+1][1:-1].split(','),dtype = np.int32)
                # 共8個點, 每個點是(u,v,1)的形式
                points_2D = np.ones((8,3))
                tmp_2D = np.array(parsed_data_tmp[start+2][1:-1].split(','),dtype = np.float64)
                for point in range(8):
                    points_2D[point][0] = tmp_2D[point]
                    points_2D[point][1] = tmp_2D[point+8]
                    # 測試有沒有對上
                    # p = (int(tmp_2D[point]),int(tmp_2D[point+8]))
                    # cv2.circle(image,p, 5,colors[i] , 3)

                tmp["position"] = points_2D

                tmp["pose"] = np.array(parsed_data_tmp[start+3][1:-1].split(','),dtype = np.float64)
                start+=4
                self.Tcw[i] = tmp

    def DrawAxis_host(self,image,center,depth):
        # draw x
        length = 80/depth
        image =  cv2.line(image,center,(int(center[0]-length),center[1]),(0,0,255),3)
        # draw y
        image =  cv2.line(image,center,(center[0],int(center[1]-length)),(255,0,0),3)
        # draw z
        image =  cv2.line(image,center,(center[0],center[1]),(0,255,0),3)
        return image
    
    def DrawAxis(self,image,center,x,y,z):
        
        length = ((center[0]-z[0])**2+(center[1]-z[1])**2)**0.5
        # draw x
        image =  cv2.line(image,center,x,(0,0,255),3)
        # draw y
        image =  cv2.line(image,center,(center[0],int(center[1]-length)),(255,0,0),3)
        # draw z
        image =  cv2.line(image,center,z,(0,255,0),3)
        return image


        
    def Pixel2World(self,point, Rot, Trans):
        # 從pixel的位置推回在world coordinate的座標
        # P_pixel = Kalib(Rot*Pw+Trans) => Pw = inv(K*Rot) (P_{pixel}-K*Trans)
        KR_inv = inv(np.matmul(self.Kalib,Rot))
        KT = np.matmul(self.Kalib,Trans)
        return np.matmul(KR_inv, point - KT)

    def World2Pixel(self,point, Rot, Trans):
        tmp = np.matmul(Rot,point)+Trans
        return np.matmul(self.Kalib, tmp)

    def DrawCircles(self,img, points, color, size):
        for point in range(8):
            # p = tmp[i]["position"][point][0:2]
            u = points[point][0]
            v = points[point][1]
            p = (int(u),int(v))
            cv2.circle(img, p, size ,color, 3)

    def setAnchor(self,u,v):
        pixel = np.array([u,v,1])
        Pc = np.matmul(inv(self.Kalib),pixel)
        Rot = np.eye(3, dtype=np.float64)
        return Rot, -Pc


    def T_Anchor2VC(self,theta,R_AC,T_AC):
        R_x = np.eye(3, dtype=np.float64)
        radius = (theta)/180*math.pi
        R_x[0,0] = math.cos(radius)
        R_x[0,2] = math.sin(radius)
        R_x[2,0] = -math.sin(radius)
        R_x[2,2] = math.cos(radius)


        R_Anchor2VC = inv(np.matmul(R_x,R_AC))
        t_Anchor2VC = -(np.matmul(R_x,T_AC))
        return R_Anchor2VC, np.matmul(R_Anchor2VC,t_Anchor2VC)

    def T_VC2Anchor(self,theta,R_AC,T_AC): #求出來是T_AC'
        R_A2C, T_A2C = self.T_Anchor2VC(theta, R_AC, T_AC)
        R_C2A = inv(R_A2C)
        return R_C2A, np.matmul(R_C2A, -T_A2C)
        

    #       ^ y                     ^ x
    #       |                       |                 t_C'A = - (R_x * t_AC)  [這裡的算法是 R(p+t)] => 改成 R*p + t => t' = R_C'A*t_C'A
    # ------|---- > x       y <-----|-----            R_C'A = inv(R_x * R_AC)  
    #       | anchor                |   camera'       
    #                                                 t_AC' = -t' (是R(p+t)), 改成 => R_AC'*t_AC'
    #                                                 R_AC' = inv(R_C'A)                                  
    #       ^ y    
    #       |
    # ------|---- > x           R_x : 轉theta度
    #       | camera 


    def Rot2Euler(self,R):
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([x, y, z])


    def sizeOfCuboid(self,cuboid):
        base = cuboid[0]
        p1 = cuboid[1]
        p2 = cuboid[3]
        p3 = cuboid[4]
        tmp = p1-base
        l1 = (tmp[0]**2 + tmp[1]**2)**0.5
        tmp = p2-base
        l2 = (tmp[0]**2 + tmp[1]**2)**0.5
        tmp = p3-base
        l3 = (tmp[0]**2 + tmp[1]**2)**0.5
        return l1*l2*l3

    def anchor_object(self,Points,center):
        small_tmp = np.inf
        closet = ""
        for key,value in Points.items():
            tmp = 0
            for p in value:
                tmp+=((p[0]-center[0])**2 + (p[1]-center[1])**2)**0.5
                # tmp+=abs(p[1]-center[1])
            if(tmp<small_tmp):
                # print(small_tmp)
                # print(tmp)
                # print(key)
                # print("-------")
                small_tmp = tmp
                closet = key
        return closet

    def Transfer_matrix(self,anchor,target):
        
        # print(anchor.shape)
        # print(target.shape)
        B = np.zeros((2,8))

        for i in range(8):
            B[0:2,i] = anchor[i,0:2]
        # A = anchor[0:2,0:8]
        A = np.zeros((2,8))

        for i in range(8):
            A[0:2,i] = target[i,0:2]
    
        # B = target[0:8]

        pseudo = inv(np.matmul(B,np.transpose(B)))
        pseudo = np.matmul(np.transpose(B),pseudo)
        return np.matmul(A,pseudo)

    def CalcScore(self,GT,CP):
        u_score = 0
        v_score = 0
        match = 0 
        for key, value in GT.items():
            if(key in CP.keys()):
                match += 1
                # print(key)
                diff = abs(CP[key]-value)
                u_score += np.sum(diff[0])
                v_score += np.sum(diff[1])

        u_score/=match
        v_score/=match
        # print(u_score,v_score)
        return u_score,v_score

    '''# def CalcScore_experiment(self,GT,CP):
    #     u_score = 0
    #     v_score = 0
    #     match = 0 
    #     for key, value in GT.items():
    #         if(key in CP.keys()):
    #             match += 1
    #             # print(key)
    #             diff = abs(CP[key]-value)
    #             u_score += np.sum(diff[0])
    #             v_score += np.sum(diff[1])

    #     u_score/=match
    #     v_score/=match
    #     # print(u_score,v_score)
    #     return u_score,v_score'''

    def PlaceVO(self,u_score,v_score):
        return u_score<=70 and v_score<=70

    def VO_position(self,position,T_W2VC,transfer):
        tmp = self.World2Pixel(position,T_W2VC[0:3,0:3],T_W2VC[0:3,3])
        return np.matmul(transfer,np.array([tmp[0]/tmp[2],tmp[1]/tmp[2]]))
        

    def set_host(self,data,center,depth,image):
        print(f"[virtual_angle/set_host] start")
        self.parse(data)
    
        self.world_anchor = self.World2Pixel(np.array([0,0,1]),self.Tcw["Rot"],self.Tcw["Trans"])
        VO_initial_place = np.array([center[0],center[1],1])
        self.VO_world_place = self.Pixel2World(VO_initial_place, self.Tcw["Rot"], self.Tcw["Trans"])
        self.VO_world_place[2] = depth
        self.depth = depth
        print("VO position: ", self.VO_world_place)
        '''
        if(self.save_image):
            # image = self.draw_axis(image,self.Tcw["Rot"],self.Tcw["Trans"])
            # image = self.DrawAxis_host(image,VO_initial_place[0:2],depth)
            cv2.circle(image,VO_initial_place[0:2], 5, (0,0,255) , 3)
            cv2.imshow('final', image)
            # cv2.imwrite("host.png",image)
            # for i in range(self.Tcw["nums"]):
            #     p = self.Tcw[i]["position"]
            #     for point in range(8):
            #         print(p[point,0:2])
            #         u = int(p[point,0])
            #         v = int(p[point,1])
            #         cv2.circle(image,(int(u),int(v)), 5, (0,0,255) , 3)
                    
            # cv2.imwrite("results/VO_host_20220113_green_{}_{}_dots.png".format(id_,self.depth),image)
            # cv2.imwrite("results/VO_host_20220113_green_{}_{}_raw.png".format(id_,self.depth),image)
            # cv2.imwrite("results/VO_host_20220114_1.png",image)
            cv2.waitKey(0)
        '''
        self.center_world = self.VO_world_place
        print("center",self.center_world)
        '''# VO_offset = np.array([center[0]-100/depth,center[1],1])
        # self.center_world_x = self.Pixel2World(VO_offset, self.Tcw["Rot"], self.Tcw["Trans"])
        # self.center_world_x[2] = depth
        # print("x",self.center_world_x)

        # VO_offset = np.array([center[0],center[1]-100/depth,1])
        # self.center_world_y = self.Pixel2World(VO_offset, self.Tcw["Rot"], self.Tcw["Trans"])
        # self.center_world_y[2] = depth
        # print("y",self.center_world_y)
        
        # self.center_world_z = np.array([self.VO_world_place[0],self.VO_world_place[1],self.VO_world_place[2]+0.2])
        # print("z",self.center_world_z)
        # if(self.save_log):
            # save some data'''

        self.T_AW = np.eye(4,dtype=np.float64)
        self.T_A2VC = np.eye(4,dtype=np.float64)
        self.Pw = {} #key: object class; value: 3D bounding box corners in world coordinate
        
        self.numbers = self.Tcw["nums"]
        for i in range(self.numbers):
            Pw_point = []
            name = self.Tcw[i]["class"]
            self.host_classes.append(name)
            for point in range(8):
                point_3D = self.Pixel2World(self.Tcw[i]["position"][point], self.Tcw["Rot"], self.Tcw["Trans"])
                Pw_point.append(point_3D)
            self.Pw[self.Tcw[i]["class"]] = Pw_point
        
        R_A, T_A = self.setAnchor(center[0],center[1])
       
         
        T_C2A = np.eye(4,dtype=np.float64)
        T_C2A[0:3,0:3] = R_A
        T_C2A[0:3,3] = T_A

        T_World2Camera = np.eye(4,dtype=np.float64)
        T_World2Camera[0:3,0:3] = self.Tcw["Rot"]
        T_World2Camera[0:3,3] = self.Tcw["Trans"]

        self.T_AW = np.matmul(T_C2A,T_World2Camera)
        self.host_numbers = self.Tcw["nums"]
        print(self.host_classes)
        #construct angle dictionary

        for Set_Angle in self.angles:
            P_pixel = {}
            dict_tmp = {}
            T_W2VC = np.eye(4,dtype=np.float64)
            R, T = self.T_Anchor2VC(Set_Angle,R_A,T_A)
            self.T_A2VC[0:3,0:3] = R
            self.T_A2VC[0:3,3] = T
            T_W2VC = np.matmul(self.T_A2VC,self.T_AW)

            # calculate 3D bounding box corners in Virtual camera's pixel
            for i in range(self.numbers):
                name = self.Tcw[i]["class"]
                points_2D = np.ones((8,3))
                Pp_point = []
                for point in range(8):
                    point_pixel = self.World2Pixel(self.Pw[name][point], T_W2VC[0:3,0:3], T_W2VC[0:3,3])
                    Pp_point.append(point_pixel/point_pixel[2])
                P_pixel[name] = np.array(Pp_point)

            # find the object closet to the center (used for linear mapping of pixels)
            anchor_name = self.anchor_object(P_pixel,center)
            dict_tmp["position"] = P_pixel
            dict_tmp["anchor"] = anchor_name
            dict_tmp["W2VC"] = T_W2VC

            for i in range(self.Tcw["nums"]):
                name = self.Tcw[i]["class"]
                if(name == anchor_name):
                    c_u = 0
                    c_v = 0
                    for point in range(8):
                        c_u += self.Tcw[i]["position"][point][0]
                        c_v += self.Tcw[i]["position"][point][1]
                    c_u = int(c_u/8)
                    c_v = int(c_v/8)                 
            dict_tmp["depth_center"] = [c_u,c_v]
            self.angle_dict[Set_Angle] = dict_tmp 
        #===============================================================================
        # structure of angle dict:

        # angle_dict-> key: angle, value: dictionary of the angle
        #     dictionary of the angle -> key: position, value: dictionary of bounding boxes position
        #         dictionary of bounding boxes position -> key: class name, value: 8(corners)*3(u,v,1)
        #     dictionary of the angle -> key: anchor, value: object closetest to the center
        #===============================================================================

    def check_view(self):
        match_class = 0
        for i in range(self.Tcw["nums"]):
            if(self.Tcw[i]["class"] in self.host_classes):
                match_class+=1
        if(match_class<2):
            print(f"[virtual_angle/check_view] failed")
            return False
        else:
            return True

    def check_view_exact(self):
        if(len(self.host_classes)!=self.Tcw["nums"]):
            return False
        for i in range(self.Tcw["nums"]):
            if(self.Tcw[i]["class"] not in self.host_classes):
                return False
        return True

    def scene_check(self,data):
        # print(f"[virtual_angle/scene_check] start")
        self.parse(data)
        if(not self.check_view()): 
            return []

        select_angle = 0
        smallest = np.inf
        select_T = np.eye(4,dtype=np.float64)
        select_transer = np.eye(2,dtype=np.float64)
        select_u_score = np.inf
        select_v_score = np.inf
        self.select_depth_center_host = [340,240]
        self.select_depth_center_viewer = [340,240]

        for angle, angle_value in self.angle_dict.items():
            anchor_name = angle_value["anchor"]
            for i in range(self.Tcw["nums"]):
                match = False
                name = self.Tcw[i]["class"]
                if(name == anchor_name):
                    match = True
                    target = self.Tcw[i]["position"]
                if(match):
                    transfer = self.Transfer_matrix(angle_value["position"][anchor_name],target)

                    target_final = {} #有對應到name的point位置
                    for i in range(self.Tcw["nums"]):
                        name = self.Tcw[i]["class"]
                        if(name in self.host_classes):
                            target_final[name] = self.Tcw[i]["position"][:,0:2]
                    
                    result_pose = {}
            
                    for i in range(self.Tcw["nums"]):
                        name = self.Tcw[i]["class"]
                        if(name in self.host_classes):
                        # if(Tcw[i]["class"]=="mouse"):
                            points = np.zeros((8,2))
                            for point in range(8):
                                # p = tmp[i]["position"][point][0:2]
                                # if(abs(Set_Angle)<40):
                                u = angle_value["position"][name][point][0]
                                v = angle_value["position"][name][point][1]
                                tmp = np.matmul(transfer,np.array([u,v]))
                                u = tmp[0]
                                v = tmp[1]
                                if(name!=anchor_name and abs(angle)>10): 
                                    u += angle
                                u = limitInterval(u, 0, 640)
                                v = limitInterval(v, 0, 480)
                                '''if(u<0): u = 0
                                if(u>640): u= 640
                                if(v<0): v=0
                                if(v>480): v= 480'''
                                points[point] = np.array([u,v])
                        
                            # cv2.circle(image,p, 5,colors[i] , 3)
                            result_pose[name] = points
                        # cv2.putText(image, "{}".format(Tcw[i]["class"]), text_point,cv2.FONT_HERSHEY_DUPLEX, 1,colors[i] , 1, cv2.LINE_AA)
                        # text_point[1]+=30
                    
                    u_score, v_score = self.CalcScore(target_final,result_pose)
                    '''# if(self.PlaceVO(u_score,v_score)):
                    #     return True
                    # if(u_score+v_score<):
                    #     return True
                        # smallest = u_score+v_score
                        # select_angle = angle
                        # select_u_score = u_score
                        # select_v_score = v_score
                        # select_T = angle_value["W2VC"]
                        # select_transer = transfer'''
                    if(u_score+v_score<smallest):
                        smallest = u_score+v_score
                        select_angle = angle
                        select_u_score = u_score
                        select_v_score = v_score
                        select_T = angle_value["W2VC"]
                        select_transer = transfer
                        self.select_depth_center_host = angle_value["depth_center"]
                        d_u = 0
                        d_v = 0
                        for point in range(8):
                            d_u += target[point,0]
                            d_v += target[point,1]
                        d_u = int(d_u/8)
                        d_v = int(d_v/8)
                        self.select_depth_center_viewer = [d_u,d_v]
                        # print("host depth center:",self.select_depth_center_host)
                        # print("viewer depth center:",self.select_depth_center_viewer)

        print("smallest angle:",select_angle)

        if(self.PlaceVO(select_u_score,select_v_score)):
            
            new_position = self.VO_position(self.VO_world_place,select_T,select_transer)
            u = new_position[0]
            v = new_position[1]
            p = (int(u),int(v))
            pixel_uv = np.array([int(u),int(v),1])
            self.viewer_pixel[0] = int(u)
            self.viewer_pixel[1] = int(v)

            world_p = np.matmul(inv(self.Kalib),pixel_uv)
            x_th = 0.3
            if(world_p[1]>0.1):
                world_p[1] = 0.1
            if(world_p[0]>x_th):
                world_p[0] = x_th
            if(world_p[0]<-x_th):
                world_p[0] = -x_th
            print("pixel in world",world_p)
            slam_pixel = self.SLAM_viewer(self.Tcw["Rot"],self.Tcw["Trans"])
            print("algo position: ",p, "SLAM position: ",slam_pixel)
            self.sm_angle = select_angle
            return world_p
            
            '''# center = p
            # x = self.VO_position(self.center_world_x,select_T,select_transer)
            # y = self.VO_position(self.center_world_y,select_T,select_transer)
            # z = self.VO_position(self.center_world_z,select_T,select_transer)



            # if(self.save_image):
            #     if(image.shape[0]>0 and image.shape[1]>0):
            #         text_point = [0,400]
            #         slam_image = image
            #         # image = self.DrawAxis(image,center,(int(x[0]),int(x[1])),(int(y[0]),int(y[1])),(int(z[0]),int(z[1])))
            #         cv2.circle(image,p, 5,(0,0,255), 3)
            #         # cv2.putText(image, "angle: {}".format(select_angle), text_point,cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255) , 1, cv2.LINE_AA)
            #         cv2.imshow('final', image)
            #         # cv2.imwrite("results/VO_test_{}_20220113_green_{}_{}_raw.png".format(select_angle,id_,self.depth),image)
            #         cv2.waitKey(0)


            #         cv2.circle(slam_image,slam_pixel, 5,(255,255,255), 3)
            #         # cv2.putText(image, "angle: {}".format(select_angle), text_point,cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255) , 1, cv2.LINE_AA)
            #         cv2.imshow('final', slam_image)
            #         # cv2.imwrite("results/VO_test_slam_20220113_green_{}_{}_raw.png".format(id_,self.depth),slam_image)
            #         # cv2.imwrite("results/VO_0113_40_20220114_{}_{}_u_{}_v_{}.png".format(select_angle,self.depth,abs(p[0]-slam_pixel[0]),abs(p[1]-slam_pixel[1])),slam_image)
            #         cv2.waitKey(0)'''
        else:
            print(f"[virtual_angle/scene_check] failed")
            return []
        # return False
        # print("smallest angle:",select_angle)


    def viewer_check(self,data,image,id_):

        host_Tcw = self.Tcw
        self.parse(data)
        if(self.check_view_exact()==False): return
        ratio = []
        '''# for i in range(self.host_numbers):
        #     name = host_Tcw[i]["class"]
        #     for j in range(self.Tcw["nums"]):
        #         if(name==self.Tcw[j]["class"]):
        #             ratio.append(self.sizeOfCuboid(self.Tcw[j]["position"])/self.sizeOfCuboid(host_Tcw[i]["position"]))
        #             print(name,", ",ratio[-1])
        # print("ratios: ",ratio)
        # print("average: ",sum(ratio)/len(ratio))
'''
        select_angle = 0
        smallest = np.inf
        select_T = np.eye(4,dtype=np.float64)
        select_transer = np.eye(2,dtype=np.float64)
        select_u_score = np.inf
        select_v_score = np.inf

        for angle, angle_value in self.angle_dict.items():
            anchor_name = angle_value["anchor"]
            match = False
            for i in range(self.Tcw["nums"]):
                name = self.Tcw[i]["class"]
                if(name == anchor_name):
                    match = True
                    target = self.Tcw[i]["position"]
                
            if(match):
                transfer = self.Transfer_matrix(angle_value["position"][anchor_name],target)

                target_final = {}
                for i in range(self.Tcw["nums"]):
                    name = self.Tcw[i]["class"]
                    target_final[name] = self.Tcw[i]["position"][:,0:2]
                
                points_final = []
                result_pose = {}
        
                for i in range(self.host_numbers):
                    name = self.Tcw[i]["class"]
                    # if(Tcw[i]["class"]=="mouse"):
                    points = np.zeros((8,2))
                    for point in range(8):
                        # p = tmp[i]["position"][point][0:2]
                        # if(abs(Set_Angle)<40):
                        u = angle_value["position"][name][point][0]
                        v = angle_value["position"][name][point][1]

                        tmp = np.matmul(transfer,np.array([u,v]))
                        u = tmp[0]
                        v = tmp[1]
                    
                        if(name!=anchor_name and abs(angle)>10): 
                            u += angle
                           

                        if(u<0): u = 0
                        if(u>640): u= 640
                    
                        
                        if(v<0): v=0
                        if(v>480): v= 480
                        points[point] = np.array([u,v])
                    
                    
                        # cv2.circle(image,p, 5,colors[i] , 3)
                    result_pose[name] = points
                    # cv2.putText(image, "{}".format(Tcw[i]["class"]), text_point,cv2.FONT_HERSHEY_DUPLEX, 1,colors[i] , 1, cv2.LINE_AA)
                    # text_point[1]+=30
                
                u_score, v_score = self.CalcScore(target_final,result_pose)

                if(u_score+v_score<smallest):
                    smallest = u_score+v_score
                    select_angle = angle
                    select_u_score = u_score
                    select_v_score = v_score
                    select_T = angle_value["W2VC"]
                    select_transer = transfer

        print("smallest angle:",select_angle)
        
        if(self.PlaceVO(select_u_score,select_v_score)):
            
            new_position = self.VO_position(self.VO_world_place,select_T,select_transer)
            u = new_position[0]
            v = new_position[1]
            p = (int(u),int(v))
            pixel_uv = np.array([int(u),int(v),1])

            world_p = np.matmul(inv(self.Kalib),pixel_uv)

            print("pixel in world",world_p)
            slam_pixel = self.SLAM_viewer(self.Tcw["Rot"],self.Tcw["Trans"])
            print("algo position: ",p, "SLAM position: ",slam_pixel)
            
            '''# center = p
            # x = self.VO_position(self.center_world_x,select_T,select_transer)
            # y = self.VO_position(self.center_world_y,select_T,select_transer)
            # z = self.VO_position(self.center_world_z,select_T,select_transer)
'''


            if(self.save_image):
                if(image.shape[0]>0 and image.shape[1]>0):
                    text_point = [0,400]
                    slam_image = image
                    # image = self.DrawAxis(image,center,(int(x[0]),int(x[1])),(int(y[0]),int(y[1])),(int(z[0]),int(z[1])))
                    cv2.circle(image,p, 5,(0,0,255), 3)
                    # cv2.putText(image, "angle: {}".format(select_angle), text_point,cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255) , 1, cv2.LINE_AA)
                    cv2.imshow('final', image)
                    # cv2.imwrite("results/VO_test_{}_20220113_green_{}_{}_raw.png".format(select_angle,id_,self.depth),image)
                    cv2.waitKey(0)


                    cv2.circle(slam_image,slam_pixel, 5,(255,255,255), 3)
                    # cv2.putText(image, "angle: {}".format(select_angle), text_point,cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255) , 1, cv2.LINE_AA)
                    cv2.imshow('final', slam_image)
                    # cv2.imwrite("results/VO_test_slam_20220113_green_{}_{}_raw.png".format(id_,self.depth),slam_image)
                    # cv2.imwrite("results/VO_0113_40_20220114_{}_{}_u_{}_v_{}.png".format(select_angle,self.depth,abs(p[0]-slam_pixel[0]),abs(p[1]-slam_pixel[1])),slam_image)
                    cv2.waitKey(0)
                

    def SLAM_viewer(self,R,t):
        new_place = np.ones((4,1))
        new_place[0:3,0] = self.VO_world_place
        T = np.eye(4,dtype=np.float64)
        T[0:3,0:3] = R
        T[0:3,3] = t
        tmp = np.matmul(T,new_place) 
        pixel_raw = np.matmul(self.Kalib,tmp[0:3,0])
        return (int(pixel_raw[0]/pixel_raw[2]),int(pixel_raw[1]/pixel_raw[2]))
