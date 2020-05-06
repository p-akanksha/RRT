import os
import cv2
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev, splrep

plt.ion()

class RRTStar:
    class node:
        def __init__(self, x, y, theta = 0, t=0, cost = 0):
            self.x = x
            self.y = y
            self.t = t
            self.theta = theta
            self.parent = None
            self.x_path = None
            self.y_path = None
            self.theta_path = None
            self.cost = cost

        def pretty_print(self):
            print("x: " + str(self.x))
            print("y: " + str(self.y))
            print("x_path: " + str(self.x_path))
            print("y_path: " + str(self.y_path))

    def create_world(self):
        im = cv2.imread('map.png')

        imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(imgray, 200, 255, 0)

        kernel = np.ones((3, 3),np.uint8)
        world = cv2.erode(thresh, kernel,iterations = 1)

        return world

    def __init__(self, start, goal ):
        self.start_node = self.node(start[0], start[1])
        self.goal_node = self.node(goal[0], goal[1])
        self.nodes = [self.start_node]
        self.lower_lim_x = 0
        self.lower_lim_y = 0
        self.upper_lim_x = 100
        self.upper_lim_y = 100 
        self.neigh_dist = 4
        self.vel = 2    # robot speed 
        self.r = 0.4826 # robot radius
        self.c = 1      # robot clearance
        self.thresh = self.c + self.r
        self.world = self.create_world()

    def check_collision(self, node):
        ret = False

        map_idx_x = int(10*node.x)
        map_idx_y = 1000-int(10*node.y)

        if node.x - self.thresh < self.lower_lim_x:
            ret = True
        elif node.x + self.thresh > self.upper_lim_x:
            ret = True
        elif node.y - self.thresh < self.lower_lim_y:
            ret = True
        elif node.y + self.thresh > self.upper_lim_y:
            ret = True

        if map_idx_x < 0 or map_idx_x >=1000 or map_idx_y < 0 or map_idx_y >= 1000:
            print(map_idx_x)
            print(map_idx_y)
            print("Array index out of bound")
            return True

        if self.world[map_idx_y][map_idx_x] == 0:
            # print("Collision Detected!!!!!")
            ret = True

        return ret

    def goal_check(self, node):
        if self.get_dist(node, self.goal_node) < 5:
            return True

        return False

    def get_random_node(self):
        x = random.randint(1, 100)
        y = random.randint(1, 100)

        new_node = self.node(x, y)

        if self.check_collision(new_node):
            return None

        return self.node(x, y)

    def get_dist(self, node1, node2):
        return math.sqrt((node1.x -node2.x)**2 + (node1.y - node2.y)**2)

    def get_nearest_node(self, rand_node, nodes):
        nearest_node_idx = 0
        min_dist = float('inf')

        for i, node in enumerate(nodes):
            dist = self.get_dist(rand_node, node)
            if dist < min_dist:
                nearest_node_idx = i
                min_dist = dist

        return nearest_node_idx

    def step_ahead(self, parent, dest_node):
        par_x = parent.x
        par_y = parent.y

        dest_x = dest_node.x
        dest_y = dest_node.y

        theta = np.arctan2((dest_y-par_y), (dest_x-par_x))

        count = 0
        x_path = [par_x]
        y_path = [par_y]
        x = par_x
        y = par_y
        dist = 0

        while(count < 10): 
            dx = 0.1 * math.cos(theta) * self.vel
            dy = 0.1 * math.sin(theta) * self.vel
            x = x + dx
            y = y + dy

            dist = dist + np.sqrt(dx**2 + dy**2)
            
            if self.check_collision(self.node(x, y)):
                return None

            x_path.append(x)
            y_path.append(y)
            count = count + 1

        new_node = self.node(x, y, theta, t=parent.t+10)
        new_node.parent = parent
        new_node.x_path = x_path
        new_node.y_path = y_path
        new_node.theta = [theta] * len(x_path)
        new_node.cost = parent.cost + dist

        return new_node

    def get_path(self, parent, child):
        dist = self.get_dist(parent, child)

        if (dist*10) % self.vel == 0:
            max_count = (dist*10)/self.vel
        else:
            max_count = (dist*10)/self.vel + 1

        par_x = parent.x
        par_y = parent.y

        child_x = child.x
        child_y = child.y

        theta = np.arctan2((child_y-par_y), (child_x-par_x))

        count = 0
        x_path = [par_x]
        y_path = [par_y]
        x = par_x
        y = par_y
        dist = 0

        while(count < max_count): 
            dx = 0.1 * math.cos(theta) * self.vel
            dy = 0.1 * math.sin(theta) * self.vel
            x = x + dx
            y = y + dy

            dist = dist + np.sqrt(dx**2 + dy**2)
            
            if self.check_collision(self.node(x, y)):
                return None, None, None

            x_path.append(x)
            y_path.append(y)
            count = count + 1

        return x_path, y_path, [theta] * len(x_path)

    def get_neighbours(self, new_node):
        ngh_indx = []

        for i, node in enumerate(self.nodes):
            dist = self.get_dist(new_node, node)
            if dist <= 4:
                ngh_indx.append(i)

        return ngh_indx

    def set_parent(self, new_node, ngh_indx):
        for i in ngh_indx:
            dist = self.get_dist(new_node, self.nodes[i])
            if self.nodes[i].cost + dist < new_node.cost:
                x_path, y_path, theta_path = self.get_path(self.nodes[i], new_node)
                if x_path == None:
                    continue
                new_node.t = self.nodes[i].t + len(x_path) - 1
                new_node.x_path = x_path
                new_node.y_path = y_path
                new_node.theta = theta_path[0]
                new_node.theta_path = theta_path
                new_node.cost = self.nodes[i].cost + dist
                new_node.parent = self.nodes[i]
                # self.propagate_cost_to_leaves(new_node)

    def propagate_cost_to_leaves(self, parent):
        for node in self.nodes:
            if node.parent == parent:
                dist = self.get_dist(parent, node)
                node.cost = parent.cost + dist
                self.propagate_cost_to_leaves(self.node)

    def rewire(self, new_node, ngh_indx):
        new_path_x = []
        new_path_y = []

        for i in ngh_indx:
            dist = self.get_dist(new_node, self.nodes[i])
            if new_node.cost + dist < self.nodes[i].cost:
                x_path, y_path, theta_path = self.get_path(new_node, self.nodes[i])
                if x_path == None:
                    continue
                new_node.t = self.nodes[i].t + len(x_path) - 1
                self.nodes[i].x_path = x_path
                self.nodes[i].y_path = y_path
                self.nodes[i].theta = theta_path[0]
                self.nodes[i].theta_path = theta_path
                self.nodes[i].cost = new_node.cost + dist
                self.nodes[i].parent = new_node
                self.propagate_cost_to_leaves(self.nodes[i])
                new_path_x.append(x_path)
                new_path_y.append(y_path)

        return new_path_x, new_path_y


    def plan(self):
        if self.check_collision(self.start_node):
            print("Start node inside obstacle")
            exit()

        if self.check_collision(self.goal_node):
            print("Goal node inside obstacle")
            exit()

        fig = plt.figure()
        ax = fig.gca()
        # ax.axis('equal')
        ax.set_xlim([0,100])
        ax.set_ylim([0,100])

        cir_start = plt.Circle((self.start_node.x, self.start_node.y), 1, fill=True, color = 'b')
        cir_goal = plt.Circle((self.goal_node.x, self.goal_node.y), 1, fill=True, color = 'b')

        ax.add_patch(cir_start)
        ax.add_patch(cir_goal)

        count = 0
        while (True):
            rand_node = self.get_random_node()
            if rand_node == None:
                continue

            nearest_node_idx = self.get_nearest_node(rand_node, self.nodes)
            new_node = self.step_ahead(self.nodes[nearest_node_idx], rand_node)

            # if collision detected, continue
            if new_node == None:
                continue

            self.nodes.append(new_node)
            ngh_indx = self.get_neighbours(new_node)
            self.set_parent(new_node, ngh_indx)
            new_path_x, new_path_y = self.rewire(new_node, ngh_indx)

            cir_node = plt.Circle((new_node.x, new_node.y), 0.2, fill=True, color = 'r')
            ax.add_patch(cir_node)
            plt.plot(new_node.x_path, new_node.y_path, color = 'g', linewidth = 1)

            for i in range(len(new_path_x)):
                plt.plot(new_path_x[i], new_path_y[i], color = 'g', linewidth = 1)

            plt.pause(0.01)

            if self.goal_check(new_node):
                print("Goal reached")
                break

            count = count + 1
            # if count == 5:
            # 	break

        # plt.show()

        return new_node

    def backtrace(self, cur_node):
        if(cur_node.parent == None):
            return np.asarray([]), np.asarray([]), np.asarray([])

        x, y, t = self.backtrace(cur_node.parent)

        x_s = np.hstack((x, cur_node.x))
        y_s = np.hstack((y, cur_node.y))
        t_s = np.hstack((t, cur_node.t))

        return x_s, y_s, t

def main():
    # starting position (x, y)
    start = [30, 30]
    # start_node = node(start[0], start[1])

    # goal point (x, y)
    goal = [70, 70]
    # goal_node = node(goal[0], goal[1])

    rrt_star = RRTStar(start, goal)
    res = rrt_star.plan()

    t = res.t

    # backtrace path 
    x_s, y_s, t_s = rrt_star.backtrace(res)

    step = float(1/float(t))
    # print(step)

    # Path smoothing
    tck, u = splprep([x_s, y_s], s=1)
    u_s = np.arange(0, 1.01, step)
    new_points = splev(u_s, tck)
    new_points = np.asarray(new_points)
    print(new_points.shape)

    # Plot both trajectories
    plt.plot(x_s, y_s, color = 'r', linewidth = 1.5)
    plt.plot(new_points[0], new_points[1], label="S", color = 'c', linewidth = 1.5)

    # save data in txt file
    out = new_points.T
    if os.path.exists("out.txt"):
        os.remove("out.txt")
    f1 = open("out.txt", "a")

    for i in range(len(out)):
        np.savetxt(f1, out[i], fmt="%s", newline=' ')
        f1.write("\n")

    plt.show()
    plt.pause(15)
    plt.close()

if __name__ == "__main__":
    main()

