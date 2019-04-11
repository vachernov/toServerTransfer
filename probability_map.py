import numpy as np
import cv2
import rospy

from damage_detection.msg import road_damage, road_damage_list
from nav_msgs.msg import OccupancyGrid, Odometry

WIDTH = 800
HEIGHT = 800

D2P_TRANSFORM = 0.1

K = 0.2

X = 0.0
Y = 0.0
Theta = 0.0

dX = 0.0
dY = 0.0
dTheta = 0.0

def createMap(w, h):
    return np.zeros((w, h))


def distance2pixels(a):
    return a * D2P_TRANSFORM


def intersectMaps(old, mask): # make intersection of view areas after given dX, dY, dTheta
    global dX
    global dY
    global dTheta

    for x_new in range(WIDTH):
        for y_new in range(HEIGHT):
            # do reverse transform as direct on (-dTheta)* (-dx, -dy)
            x_old =  np.cos(-dTheta)*x_new + np.sin(-dTheta)*y_new + distance2pixels(-dX)
            y_old = -np.sin(-dTheta)*x_new + np.cos(-dTheta)*y_new + distance2pixels(-dY)

            # if point is our view, we save probability in it
            if (x_old < WIDTH) and (x_old > 0) and (y_old < WIDTH) and (y_old > 0):
                mask[x_new] = old[0]
                mask[y_new] = old[0]

    return mask


def updateMap(new, mask):
    h = createMap(WIDTH, HEIGHT)

    for n in range(WIDTH):
        for m in range(HEIGHT):
            h[n][m] = new[n][m]*(1 - K) + mask[n][m]*K

    return h


def cordinates_to_image(hole_list, image_shape):
    image = np.zeros(image_shape, dtype=np.uint8)
    print(hole_list)
    for hole in hole_list.damages:
        image = cv2.rectangle(image, (int(hole.top_left.x), int(hole.top_left.y)),\
                             (int(hole.bottom_right.x), int(hole.bottom_right.y)), int(hole.probability*255), -1)
    return image


def bird_eye_transform(original_image):
    src = np.float32([[310, 390], [475, 390], [-800, 430 + 130], [800 + 635, 450 + 130]])
    dst = np.float32([[0, 0], [800, 0], [0, 800], [800, 800]])
    m = cv2.getPerspectiveTransform(src, dst)
    image = cv2.warpPerspective(original_image, m, (800, 800))
    return image


class image_converter:
    def __init__(self):
        self.m = createMap(WIDTH, HEIGHT)
        self.p = createMap(WIDTH, HEIGHT)

        self.map_msg = OccupancyGrid()
        self.map_msg.info.width = WIDTH
        self.map_msg.info.height = HEIGHT
        self.map_msg.info.resolution = 0.2
        self.map_msg.data = range(WIDTH*HEIGHT)
        self.image_sub = rospy.Subscriber("boxes_topic", road_damage_list, self.callback)
        self.carpos_sub = rospy.Subscriber("base_pose_ground_truth", Odometry, self.get_position)
        self.prob_pub = rospy.Publisher("probability_topic", OccupancyGrid, queue_size=16)
        print('Probability Init complited.')

    def callback(self, data):
        image = cordinates_to_image(data, (WIDTH, HEIGHT))
        bird_image = bird_eye_transform(image)

        self.p = intersectMaps(self.m, self.p)
        self.m = updateMap(bird_image, self.p)

        self.map_msg.header.stamp = rospy.Time.now()

        for i in range(WIDTH*HEIGHT):
            self.map_msg.data[i] = self.m.reshape(-1)[i]
        self.prob_pub.publish(self.map_msg)

    def get_position(self, data):
        global X
        global Y
        global Theta
        global dX
        global dY
        global dTheta

        dX = X - data.pose.pose.position.x
        dY = Y - data.pose.pose.position.y
        dTheta = Theta - data.pose.pose.position.z

        X = data.pose.pose.position.x
        Y = data.pose.pose.position.y
        Theta = data.pose.pose.position.z
        #print(X, Y, Theta, dX, dY, dTheta)


if __name__ == '__main__':
    rospy.init_node('image_probability_sub', anonymous=True)
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
