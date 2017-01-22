import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from field import field
import numpy as np

class arrPoint(object):
    x = 0
    y = 0
    def __init__(self,x,y):
        self.x = x
        self.y = y

class obstacle(object):
    def __init__(self,c,k,x,y):
        self.c = c
        self.k = k
        self.x = x
        self.y = y

class plot(object):
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    obstacles = []
    goals = []
    x = []
    y = []
    z = []
    gx = 0
    gy = 0
    gz = 0
    gu = 0
    gv = 0
    gw = 0
    def __init__(self,goal):
        #print("New Plot")
        self.goals.append(goal)
    def distance(self,x1,y1,x2,y2):
        return ((x2-x1)**2 + (y2-y1)**2)**(0.5)
    def singleCost(self,x,y,c,k,a,b):
        zinf = 10
        if(self.distance(a,b,x,y)**c == 0):
            if(k<0):
                s = -1
                return zinf*s
            else:
                return zinf
        else:
            return k/(self.distance(a,b,x,y)**c)
    def cost(self,x,y):
        c = self.singleCost(x,y,1.1,-10,self.goals[0].x,self.goals[0].y)
        #c=0
        for obstacle in self.obstacles:
            c = c + self.singleCost(x,y,obstacle.c,obstacle.k,obstacle.x,obstacle.y)
        return c
    def makePoints(self):
        for i in range(-100,100):
            for j in range(-100,100):
                self.x.append(i/10)
                self.y.append(j/10)
                self.z.append(self.cost(self.x[len(self.x)-1],self.y[len(self.y)-1]))
        #self.x = np.arange(0,10,0.25)
        #self.y = np.arange(-3,3,0.25)
        #self.x, self.y = np.meshgrid(self.x,self.y)
        #z = self.cost(self.x,self.y)
    def makeVectors(self,x,y):
        z = self.cost(x,y)
        bstep = .000001
        bg = [(self.cost(x+bstep,y)-self.cost(x,y))/bstep,(self.cost(x,y+bstep)-self.cost(x,y))/bstep]
        self.gx = x
        self.gy = y
        self.gz = z + 5
        #print "from [" + str(x) + ', ' + str(y) + ', ' + str(z) + ']'
        grad = field(arrPoint(x,y))
        #print "robot" + str(grad.robotPos.x) + ", " + str(grad.robotPos.y)
        grad.goals.append(arrPoint(self.goals[0].x,self.goals[0].y))
        for obstacle in self.obstacles:
            grad.obstacles.append(obstacle)
        g = grad.findGradientVector()
        self.gu = (g[0])
        self.gv = (g[1])
        self.gw = g[0] + g[1]
        #g.append(self.gw)
        #print "Computed" + str(g)
        #print "Brute   " + str(bg)
        self.ax.quiver(self.gx,self.gy,self.gz,self.gu,self.gv,self.gw,length=2,pivot='tail')
    def plot(self):
        #self.ax.plot_trisurf(self.x,self.y,self.z,color='red')
        plt.show()

def main():
    pylar1 = obstacle(1,3,9,9)
    pylar2 = obstacle(1,4,0,5)

    surf = plot(arrPoint(9,9))

    #surf.obstacles.append(pylar1)
    surf.obstacles.append(pylar2)

    surf.makePoints()
    for i in range (-10,10):
        for j in range(-10,10):
            surf.makeVectors(i,j)
    #surf.makeVectors(1,2)
    #print(surf.z)
    surf.plot()

if __name__ == "__main__":
    main()

