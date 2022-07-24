import math 
import matplotlib.pyplot as plt 
from PIL import Image
import cv2 

visulation = 1

class AstarPath:
    #initialising the class
    def __init__(self, bot_radius, grid, x_obs, y_obs):
        self.bot_radius = bot_radius 
        self.grid_size = grid
        self.create_obstacle_map(x_obs, y_obs)
        self.path = self.get_path()

    class Block:
        def __init__(self, x, y, cost, path):
            self.x = x
            self.y = y 
            self.cost = cost 
            self.path = path 
        
        def __str__(self):
            return str(self.x) + '+' + str(self.y) + ',' + str(self.cost) + ',' + str(self.path) 
    @staticmethod
    def get_path():
        #all possible paths/direction the robot can go from current position
        path = [[1,0,1], [0,1,1], [-1,0,1], [0,-1,1], [-1,-1, math.sqrt(2)], [-1,1,math.sqrt(2)], [1,-1,math.sqrt(2)], [1,1,math.sqrt(2)]]
        return path 
    @staticmethod
    # A* finds a path from start to goal.
    # h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    def heuristicCalculation(n1, n2):
        return 1.0*math.sqrt((n1.x - n2.x)**2 + (n1.y-n2.y)**2)

    #calculates the grid position
    def gridPosition(self, idx, offset):
        return idx*self.grid_size + offset 

    #distance to block
    def calc_xy(self, position, min_position): #distance to block
        return round((position-min_position) / self.grid_size)

    #blocks to distance
    def gridIndexCalculation(self, block):
        return (block.y - self.y_min) * self.x_width + (block.x - self.x_min)

    #checks if the curr location is valid or not
    def isValid(self, block):
        x_pos = self.gridPosition(block.x, self.x_min)
        y_pos = self.gridPosition(block.y, self.y_min)

        if x_pos < self.x_min:
            return 0
        elif y_pos < self.y_min: 
            return 0 
        elif x_pos >= self.x_max:
            return 0 
        elif y_pos >= self.y_max:
            return 0
        
        if self.obs_pos[block.x][block.y]:
            return 0 
        return 1
    
    #it will generate the maze to be solved
    def create_obstacle_map(self, x_obs, y_obs):
        self.x_min = round(min(x_obs))
        self.y_min = round(min(y_obs))
        self.x_max = round(max(x_obs))
        self.y_max = round(max(y_obs))
        #width with block size of grid size
        self.x_width = round((self.x_max - self.x_min)/self.grid_size)
        self.y_width = round((self.y_max - self.y_min)/self.grid_size)
        self.obs_pos = [[False for i in range(self.y_width)] for i in range(self.x_width)]
        for i in range(self.x_width):
            #calculating current position from start point with grid size as block
            x = self.gridPosition(i, self.x_min)
            for j in range(self.y_width):
                y = self.gridPosition(j, self.y_min)
                for idx_x_obs, idx_y_obs in zip(x_obs, y_obs):
                    d = math.sqrt((idx_x_obs-x)**2 + (idx_y_obs-y)**2)
                    if d<=self.bot_radius:
                        self.obs_pos[i][j] = 1
                        break 
    
    #after the path is found this function will be used to traverse and show user the ouput path as reslult
    def finalPathCal(self, end_block, record_closed):
        x_outPath, y_outPath = [self.gridPosition(end_block.x, self.x_min)], [self.gridPosition(end_block.y, self.y_min)]
        path = end_block.path 
        while path != -1:
            n = record_closed[path]
            x_outPath.append(self.gridPosition(n.x, self.x_min))
            y_outPath.append(self.gridPosition(n.y, self.y_min))
            path = n.path 

        return x_outPath, y_outPath

    def aStarSearch(self, start_x, start_y, end_x, end_y):
        #x_min is min(obstacle_x) and start_x is the start point we give as input
        start_block = self.Block(self.calc_xy(start_x, self.x_min), self.calc_xy(start_y, self.y_min), 0.0, -1) #x_min is min(obstacle_x) and start_x is the start point we give as input
        end_block = self.Block(self.calc_xy(end_x, self.x_min), self.calc_xy(end_y, self.y_min), 0.0, -1)

        record_open, record_closed = dict(), dict()
        record_open[self.gridIndexCalculation(start_block)] = start_block 
        while 1:
            if len(record_open) == 0:
                print("Invalid inputs")
                break 

            #tarverse through all elements of record open and then return that cost which is minimum
            total_cost = min(record_open, key=lambda x: record_open[x].cost + self.heuristicCalculation(end_block, record_open[x]))
            collected_cost = record_open[total_cost]

            if visulation:
                plt.plot(self.gridPosition(collected_cost.x, self.x_min), self.gridPosition(collected_cost.y, self.y_min), "xy")
                if len(record_closed.keys())%20 == 0:
                    plt.pause(0.000001)

            #if current block position is same as end block then this if executes
            if collected_cost.x == end_block.x and collected_cost.y == end_block.y:
                print("Finished!!!")
                end_block.path = collected_cost.path
                end_block.cost = collected_cost.cost
                break 
            
            #remove from record_open and add in record_closed
            del record_open[total_cost]
            record_closed[total_cost] = collected_cost

            for i in range(len(self.path)):
                block = self.Block(collected_cost.x + self.path[i][0], collected_cost.y + self.path[i][1], collected_cost.cost + self.path[i][2], total_cost)
                idx_block = self.gridIndexCalculation(block)

                if not self.isValid(block):
                    continue 
                if idx_block in record_closed:
                    continue
                if idx_block not in record_open:
                    record_open[idx_block] = block
                else:
                    if record_open[idx_block].cost > block.cost:
                        record_open[idx_block] = block

        x_outPath, y_outPath = self.finalPathCal(end_block, record_closed)
        return x_outPath, y_outPath 


def main():
    grid_size =3.0
    bot_radius = 2.0

    image = cv2.imread('maze.jpg')
    image = cv2.resize(image, (200,200))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (width, length) = gray.shape
    x_obs, y_obs = [], []
    for i in range(width):
        for j in range(length):
            if gray[i][j] <= 150:
                y_obs.append(i)
                x_obs.append(j)

    plt.plot(x_obs, y_obs, ".k")
    plt.show()

    print("enter starting index")
    start_x = int(input())
    start_y = int(input())

    print("enter ending index")
    end_x = int(input())
    end_y = int(input())

    if visulation:
        plt.plot(x_obs, y_obs, ".r")
        plt.plot(start_x, start_y, "og")
        plt.plot(end_x, end_y, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AstarPath(bot_radius, grid_size, x_obs, y_obs)
    x_outPath, y_outPath = a_star.aStarSearch(start_x, start_y, end_x, end_y)

    if visulation:
        plt.plot(x_outPath, y_outPath, "r")
        plt.show()

if __name__ == '__main__':
    main()