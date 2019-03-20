#!/usr/bin/env python
#! coding:utf-8

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import rospy
import tf
from nav_msgs.msg import Odometry
import message_filters
from geometry_msgs.msg import Quaternion

from landmark_slam_msgs.msg import Landmark, LandmarkArray

np.set_printoptions(linewidth=200)

# EKF state covariance
Q = np.diag([0.5, 0.5, np.deg2rad(30.0)])**2

# Simulation parameter
observation_noise = np.diag([0.2, np.deg2rad(1.0)])**2 # landmark observation noise
Rsim = np.diag([1.0, np.deg2rad(10.0)])**2 # input noise

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]
MAX_RANGE = 20.0  # maximum observation range
M_DIST_TH = 2.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

CHI_2 = 9.21934 # X^2, 99%

show_animation = True

class ObjectDetectionBasedLandmarkSLAM:
    def __init__(self):
        self.Q = np.diag([0.5, 0.5, np.radians(30.0)])**2
        self.OBSERVATION_NOISE = np.diag([0.2, np.radians(10.0)])**2 # landmark observation noise
        self.R = np.diag([1.0, np.radians(10.0)])**2 # input noise
        self.MAX_RANGE = 10.0
        self.LIMIT_MAHALANOBIS_DISTANCE = 2.0
        self.ROBOT_STATE_SIZE = 3 # x, y, yaw
        self.LANDMARK_STATE_SIZE = 2 # x, y

        rospy.init_node('object_detection_based_landmark_slam')

        #self.odom_sub = rospy.Subscriber('/odometry', Odometry, self.odom_callback, queue_size=1)
        self.odom_sub = message_filters.Subscriber('/odom/sim/noise', Odometry)
        self.lm_sub = message_filters.Subscriber('/observation/sim', LandmarkArray)
        mf = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.lm_sub], 1, 0.1)
        mf.registerCallback(self.callback)
        self.estimated_pose_pub = rospy.Publisher('/estimated_pose', Odometry, queue_size=1)

        self.last_time = rospy.get_time()
        self.current_pose = None
        self.last_pose = None
        self.first_flag = True
        self.x_est = np.zeros((self.ROBOT_STATE_SIZE, 1))
        self.p_est = np.eye(self.ROBOT_STATE_SIZE)
        self.estimated_pose = Odometry()
        print 'object detection based landmark SLAM'

        self.send_transform(self.x_est)

        #rospy.spin()

    def move(self, x, u, dt):
        F = np.array([[1.0, 0, 0],
                      [0, 1.0, 0],
                      [0, 0, 1.0]])

        B = np.array([[dt * math.cos(x[2, 0]), 0],
                      [dt * math.sin(x[2, 0]), 0],
                      [0.0, dt]])

        x = np.dot(F, x) + np.dot(B, u)
        return x

    def ekf_slam(self, x_est, p_est, u, z, dt):
        print "start ekf_slam"
        # Predict
        S_ = self.ROBOT_STATE_SIZE
        #print x_est, p_est, u, z, dt
        print x_est[0:S_]
        print p_est[0:S_, 0:S_]
        '''
        print u
        print z
        print dt
        '''
        x_est[0:S_] = self.move(x_est[0:S_], u, dt)
        jf = self.get_jacobian_f(x_est[0:S_], u, dt)
        p_est[0:S_, 0:S_] = np.dot(np.dot(jf, p_est[0:S_, 0:S_]), jf.T) + self.Q
        initP = np.eye(2)

        # Update
        print(str(len(z)) + " landmarks detected")
        for iz in range(len(z)):  # for each observation
            minid = self.get_correspond_landmark_index(x_est, p_est, z[iz, 0:2])

            nLM = self.calculate_landmark_num(x_est)
            if minid == nLM:
                print("New LM")
                # Extend state and covariance matrix
                xAug = np.vstack((x_est, self.calculate_landmark_position(x_est, z[iz, :])))
                PAug = np.vstack((np.hstack((p_est, np.zeros((len(x_est), LM_SIZE)))),
                                  np.hstack((np.zeros((self.LANDMARK_STATE_SIZE, len(x_est))), initP))))
                x_est = xAug
                p_est = PAug
            lm = self.get_estimated_landmark_position(x_est, minid)
            e, S, H = self.calculate_innovation(lm, x_est, p_est, z[iz, 0:2], minid)

            K = np.dot(np.dot(p_est, H.T), np.linalg.inv(S))
            x_est = x_est + np.dot(K, e)
            p_est = np.dot((np.eye(len(x_est)) - np.dot(K, H)), p_est)

        x_est[2] = self.pi_2_pi(x_est[2])

        return x_est, p_est

    def get_jacobian_f(self, x, u, dt):
        Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
            (STATE_SIZE, LM_SIZE * self.calculate_landmark_num(x)))))

        jF = np.array([[0.0, 0.0, -dt * u[0] * math.sin(x[2, 0])],
                       [0.0, 0.0, dt * u[0] * math.cos(x[2, 0])],
                       [0.0, 0.0, 0.0]])

        #return Fx
        return jF

    def get_jacobian_h(self, delta, i, n):
        d = math.sqrt(delta[0, 0]**2 + delta[1, 0]**2)
        H = np.array([[-d * delta[0, 0], - d * delta[1, 0], 0, d * delta[0, 0], d * delta[1, 0]],
                      [delta[1, 0], - delta[0, 0], - 1.0, - delta[1, 0], delta[0, 0]]])

        H = H / d**2
        H1 = np.hstack((np.eye(3), np.zeros((3, 2 * n))))
        H2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                        np.eye(2), np.zeros((2, 2 * n - 2 * i))))

        _H = np.vstack((H1, H2))

        H = np.dot(H, _H)

        return H

    def calculate_landmark_num(self, x):
        n = int((len(x) - STATE_SIZE) / LM_SIZE)
        return n

    def get_estimated_landmark_position(self, x, index):
        lm = x[self.ROBOT_STATE_SIZE + self.LANDMARK_STATE_SIZE * index: self.ROBOT_STATE_SIZE + self.LANDMARK_STATE_SIZE * (index + 1), :]
        return lm

    def calculate_landmark_position(self, x, z):
        zp = np.zeros((2, 1))

        zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
        zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

        return zp

    def get_correspond_landmark_index(self, x_est, p_est, zi):
        nLM = self.calculate_landmark_num(x_est)
        mahalanobis_distance = []

        for i in range(nLM):
            lm = self.get_estimated_landmark_position(x_est, i)
            e, S, H = self.calculate_innovation(lm, x_est, p_est, zi, i)
            mahalanobis_distance.append(np.dot(np.dot(e.T, np.linalg.inv(S)), e))

        mahalanobis_distance.append(self.LIMIT_MAHALANOBIS_DISTANCE)  # new landmark
        minid = mahalanobis_distance.index(min(mahalanobis_distance))
        return minid

    def calculate_innovation(self, lm, x_est, p_est, z, index):
        # dx, dy
        delta = lm - x_est[0:2]
        # distance^2
        d2 = np.dot(delta.T, delta)[0, 0]
        # angle in robot frame
        zangle = math.atan2(delta[1, 0], delta[0, 0]) - x_est[2, 0]
        # polar coordinates
        zp = np.array([[math.sqrt(d2), self.pi_2_pi(zangle)]])
        # error in polar coordinates (estimated pose - observed pose)
        e = (z - zp).T
        e[1] = self.pi_2_pi(e[1])
        H = self.get_jacobian_h(delta, index + 1, self.calculate_landmark_num(x_est))
        S = np.dot(np.dot(H, p_est), H.T) + self.Q[0:2, 0:2]
        return e, S, H

    def calculate_error_ellipse(self, P):
        _lambda, _v = np.linalg.eig(P)
        max_index = np.argmax(_lambda)
        min_index = np.argmin(_lambda)
        a = math.sqrt(CHI_2 * _lambda[max_index])
        b = math.sqrt(CHI_2 * _lambda[min_index])
        ellipse_angle = math.atan2(_v[max_index, 1], _v[max_index, 0])
        return a, b, ellipse_angle

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def callback(self, odom, lm):
        current_time = rospy.get_time()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.last_pose = self.current_pose
        self.current_pose = odom.pose.pose
        if(self.last_pose is not None):
            dx = self.current_pose.position.x - self.last_pose.position.x
            dy = self.current_pose.position.y - self.last_pose.position.y
            d = math.sqrt(dx * dx + dy * dy)
            v = d / dt
            _, _, current_yaw = tf.transformations.euler_from_quaternion((self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w))
            _, _, last_yaw = tf.transformations.euler_from_quaternion((self.last_pose.orientation.x, self.last_pose.orientation.y, self.last_pose.orientation.z, self.last_pose.orientation.w))
            omega = (current_yaw - last_yaw) / dt
            u = np.array([[v, omega]]).T
            z = self.get_observation_from_landmark_msg(lm)
            self.x_est, self.p_est = self.ekf_slam(self.x_est, self.p_est, u, z, dt)
            self.estimated_pose.header.frame_id = "world"
            self.estimated_pose.header.stamp = rospy.get_rostime()
            self.estimated_pose.child_frame_id = "base_link"
            self.estimated_pose.pose.pose.position.x = self.x_est[0]
            self.estimated_pose.pose.pose.position.y = self.x_est[1]
            q = self.get_quaternion_from_yaw(self.x_est[2])
            self.estimated_pose.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.estimated_pose_pub.publish(self.estimated_pose);

    def get_observation_from_landmark_msg(self, landmark):
        z = []
        for lm in landmark.landmarks:
            _z = [math.sqrt(lm.pose.pose.position.x * lm.pose.pose.position.x + lm.pose.pose.position.y * lm.pose.pose.position.y), math.atan2(lm.pose.pose.position.y, lm.pose.pose.position.x)]
            z.append(_z)
        z = np.array(z)
        return z

    def send_transform(self, x):
        br = tf.TransformBroadcaster()
        br.sendTransform((x[0], x[1], 0),
                         self.get_quaternion_from_yaw(x[2]),
                         rospy.Time.now(),
                         "base_link",
                         "world"
                         )
    def process(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.send_transform(self.x_est)
            r.sleep()

    def get_quaternion_from_yaw(self, yaw):
        q = tf.transformations.quaternion_from_euler(0, 0, yaw[0])
        return q


'''
def ekf_slam(xEst, PEst, u, z):

    # Predict
    S = STATE_SIZE
    xEst[0:S] = motion_model(xEst[0:S], u)
    G, Fx = jacob_motion(xEst[0:S], u)
    #PEst[0:S, 0:S] = G.T * PEst[0:S, 0:S] * G + Fx.T * Q * Fx
    #PEst[0:S, 0:S] = np.dot(np.dot(Fx, PEst[0:S, 0:S]), Fx.T) + np.dot(np.dot(G, Q), G.T)
    PEst[0:S, 0:S] = np.dot(np.dot(Fx, PEst[0:S, 0:S]), Fx.T) + Q# + np.dot(np.dot(G, Q), G.T)
    initP = np.eye(2)

    # Update
    print(str(len(z)) + " landmarks detected")
    for iz in range(len(z[:, 0])):  # for each observation
        minid = search_correspond_LM_ID(xEst, PEst, z[iz, 0:2])

        nLM = calc_n_LM(xEst)
        if minid == nLM:
            print("New LM")
            # Extend state and covariance matrix
            xAug = np.vstack((xEst, calc_LM_Pos(xEst, z[iz, :])))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            xEst = xAug
            PEst = PAug
        lm = get_LM_Pos_from_state(xEst, minid)
        e, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], minid)

        K = np.dot(np.dot(PEst, H.T), np.linalg.inv(S))
        xEst = xEst + np.dot(K, e)
        PEst = np.dot((np.eye(len(xEst)) - np.dot(K, H)), PEst)

    xEst[2] = pi_2_pi(xEst[2])

    return xEst, PEst


def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u


def observation(xTrue, xd, u, RFID):

    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 3))

    for i in range(len(RFID[:, 0])):

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.sqrt(dx**2 + dy**2)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
        # observable landmark
        if d <= MAX_RANGE:
            # recognition probablirity
            if np.random.rand(1) > 0.70:
                dn = d + np.random.randn() * observation_noise[0, 0]  # add noise
                anglen = angle + np.random.randn() * observation_noise[1, 1]  # add noise
                zi = np.array([dn, anglen, i])
                z = np.vstack((z, zi))

    # add noise to input
    ud = np.array([[
        u[0, 0] + np.random.randn() * Rsim[0, 0],
        u[1, 0] + np.random.randn() * Rsim[1, 1]]]).T

    xd = motion_model(xd, ud)
    return xTrue, z, xd, ud


def motion_model(x, u):

    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = np.dot(F, x) + np.dot(B, u)
    return x


def calc_n_LM(x):
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def jacob_motion(x, u):

    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_LM(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]])

    G = np.eye(STATE_SIZE) + np.dot(np.dot(Fx.T, jF), Fx)

    return G, Fx,


def calc_LM_Pos(x, z):
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

    return zp


def get_LM_Pos_from_state(x, ind):

    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def search_correspond_LM_ID(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance
    """

    nLM = calc_n_LM(xAug)

    mdist = []

    for i in range(nLM):
        lm = get_LM_Pos_from_state(xAug, i)
        e, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        mdist.append(np.dot(np.dot(e.T, np.linalg.inv(S)), e))

    mdist.append(M_DIST_TH)  # new landmark

    minid = mdist.index(min(mdist))

    return minid


def calc_innovation(lm, xEst, PEst, z, LMid):
    # dx, dy
    delta = lm - xEst[0:2]
    # distance^2
    q = np.dot(delta.T, delta)[0, 0]
    # angle in robot frame
    zangle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    # polar coordinates
    zp = np.array([[math.sqrt(q), pi_2_pi(zangle)]])
    # error in polar coordinates (estimated pose - observed pose)
    e = (z - zp).T
    e[1] = pi_2_pi(e[1])
    H = jacobH(q, delta, xEst, LMid + 1)
    S = np.dot(np.dot(H, PEst), H.T) + Q[0:2, 0:2]

    return e, S, H


def jacobH(q, delta, x, i):
    # distance
    sq = math.sqrt(q)
    # -d*dx, -d*dy, 0, d*dx, d*dy
    # dy, -dx, -1, -dy, dx
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - 1.0, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_LM(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))

    H = np.dot(G, F)

    return H


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def calculate_error_ellipse(P):
    _lambda, _v = np.linalg.eig(P)
    max_index = np.argmax(_lambda)
    min_index = np.argmin(_lambda)
    a = math.sqrt(CHI_2 * _lambda[max_index])
    b = math.sqrt(CHI_2 * _lambda[min_index])
    ellipse_angle = math.atan2(_v[max_index, 1], _v[max_index, 0])
    return a, b, ellipse_angle

def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    RFID = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [3.0, 15.0],
                     [-5.0, 20.0]])

    # State Vector [x y yaw v]'
    xEst = np.zeros((STATE_SIZE, 1))
    xTrue = np.zeros((STATE_SIZE, 1))
    PEst = np.eye(STATE_SIZE)

    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    while SIM_TIME >= time:
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        xEst, PEst = ekf_slam(xEst, PEst, ud, z)

        # robot error ellipse
        a, b, ellipse_angle = calculate_error_ellipse(PEst[0:2, 0:2])

        x_state = xEst[0:STATE_SIZE]

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:  # pragma: no cover
            plt.cla()
            ax = plt.gca()

            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(xEst[0], xEst[1], ".r")

            p = patches.Ellipse(xy = (xEst[0], xEst[1]), width = a, height = b, alpha = 1, angle = math.degrees(ellipse_angle), color = "cyan")
            ax.add_patch(p)
            ax.annotate('', xy=[xEst[0]+math.cos(xEst[2]), xEst[1]+math.sin(xEst[2])], xytext=[xEst[0], xEst[1]],
                        arrowprops=dict(shrink=0, width=1, headwidth=8,
                                        headlength=10, connectionstyle='arc3',
                                        facecolor='gray', edgecolor='gray')
            )

            # plot landmark
            for i in range(calc_n_LM(xEst)):
                plt.plot(xEst[STATE_SIZE + i * 2],
                         xEst[STATE_SIZE + i * 2 + 1], "xg")
                a, b, ellipse_angle = calculate_error_ellipse(PEst[(STATE_SIZE + i * 2):(STATE_SIZE + i * 2) + 2, (STATE_SIZE + i * 2):(STATE_SIZE + i * 2) + 2])
                p = patches.Ellipse(xy = (xEst[STATE_SIZE + i * 2], xEst[STATE_SIZE + i * 2 + 1]), width = a, height = b, alpha = 1, angle = math.degrees(ellipse_angle), color = "Magenta")
                ax.add_patch(p)

            plt.plot(hxTrue[0, :],
                     hxTrue[1, :], "-b", label="Ground Truth")
            plt.plot(hxDR[0, :],
                     hxDR[1, :], "-k", label="Dead Reckoning")
            plt.plot(hxEst[0, :],
                     hxEst[1, :], "-r", label="Estimated Pose")

            plt.legend()

            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


'''
if __name__ == '__main__':
    #main()
    odlm_slam = ObjectDetectionBasedLandmarkSLAM()
    odlm_slam.process()
