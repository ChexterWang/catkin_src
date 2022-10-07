from MegaDepth.sample import Depth
import cv2
import matplotlib.pyplot as plt
net = Depth(480,640)
img_path = '/home/brian/Documents/5045.png'
image = cv2.imread(img_path)

image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
test = net.depth_estimate(image)
plt.imshow(test)
plt.show()