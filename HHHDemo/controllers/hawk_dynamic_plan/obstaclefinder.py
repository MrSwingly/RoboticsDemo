##anku255
from tkinter import Image


import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
from scipy import signal as s
from scipy import ndimage as sim
from scipy import misc as m
from PIL import Image as im
from mpl_toolkits.mplot3d import Axes3D

class imgToObs():


    def __init__(self,imagepath = "C:/Users/bobbe/PycharmProjects/ECE183DA/lab3/maze.bmp"):
        self.image = cv2.imread(imagepath)

    def showimage(self):
        plt.imshow( self.image)
        plt.show()
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def obsfind(self,scale, expand = 6):
        #Grayscale the image and convert to a boolean array (true for obstacle, false for free space)
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        imagearray = np.transpose(np.array(gray))
        imagearray = imagearray < 255

        templine = []
        temparray = []
        #Make a copy of the image to expand
        obsexparray = imagearray.copy()

        # Scale Obstacles to account for robot radius
        expand = math.ceil(expand)
        linei = 0
        pixi = 0
        for line in imagearray:
            for pixel in line:
                if pixel == True:
                    for miniline in range(linei - expand - 1, linei + expand + 1):
                        for minipix in range(pixi - expand - 1, pixi + expand + 1):
                            if 0 < miniline < obsexparray.shape[0] - 1 and 0 < minipix < obsexparray.shape[1] - 1:
                                obsexparray[miniline][minipix] = True
                pixi = pixi + 1
            templine = []
            pixi = 0
            linei = linei + 1

        #For actual obstacle array
        #Scale up array via duplication
        biggerimageOarray = []
        biggerimgOarrayline = []
        for line in obsexparray.tolist():
            for p in line:
                for i in range(0,scale):
                    biggerimgOarrayline.append(p)
            for i in range(0, scale):
                biggerimageOarray.append(biggerimgOarrayline)
            biggerimgOarrayline = []
        #return Larger true/false array
        obarray = np.array(biggerimageOarray)

        #For display array (original, needs to be scaled the same for plot to work)
        #Scale up array via duplication
        biggerimagearray = []
        biggerimgarrayline = []
        for line in imagearray.tolist():
            for p in line:
                for i in range(0,scale):
                    biggerimgarrayline.append(p)
            for i in range(0, scale):
                biggerimagearray.append(biggerimgarrayline)
            biggerimgarrayline = []
        #return Larger true/false array
        array = np.array(biggerimagearray)


        print("ARRAY PROCESSING DONE")
        return [array,obarray]

    def padToMatch(self, input, toMatch):
        [xmatch, ymatch] = np.shape(toMatch)
        [xin, yin] = np.shape(input)
        output = np.pad(input, ((0, xmatch - xin), (0, ymatch - yin)), 'edge')
        return output

    def toBin(self,input):
        return np.logical_not(np.logical_not(input))

    def obsSpaceGen(self,robotSpace,obsSpace,scaledown,debug = False):
        obs = np.array(im.fromarray(obsSpace).resize(size=(round(np.shape(obsSpace)[1]/scaledown),round(np.shape(obsSpace)[0]/scaledown))))
        [x,y] = robotSpace
        robot = np.ones((y, x))
        robotRe = np.array(im.fromarray(robot).resize(size=(round(np.shape(robot)[1]/scaledown),round(np.shape(robot)[0]/scaledown))))
        configSpace = []
        # Create array of robot angle shifts
        for i in range(0, 180, 5):
            r = sim.rotate(robotRe,i,reshape = True)
            [xT,yT] = np.shape(r)
            xT = round(xT / 2)
            yT = round(yT / 2)
            temp = self.toBin(s.convolve2d(obs, r))[xT:-xT,yT:-yT]
            configSpace.append(np.array(im.fromarray(temp).resize(resample = im.BICUBIC,size = (np.shape(obsSpace)[1],np.shape(obsSpace)[0]))))
            print(i)
        if debug:
            counter1 = 0
            counter2 = 0
            for c in configSpace:
                if counter1 % 9 == 0 or counter2 % 6 == 0:
                    plt.imshow(np.flip(np.add(c.astype(int),obsSpace.astype(int)),0))
                    plt.show()
                counter1 = counter1 + 1
                counter2 = counter2 + 1
        return np.array(configSpace)




if __name__ == "__main__":
    o = imgToObs(imagepath = "C:/Users/Cooper/PycharmProjects/183Final/controllers/hawk_control2/testmaze.png")
    gray = cv2.cvtColor(o.image, cv2.COLOR_BGR2GRAY)
    plt.imshow(np.array(gray))
    plt.show()
    conSpace = o.obsSpaceGen([22,22],np.logical_not(np.array(gray)))
    gray = np.logical_not(np.array(gray))
    for obs in conSpace:
        plt.imshow(np.add(gray.astype(int),obs.astype(int)))
        plt.show()
