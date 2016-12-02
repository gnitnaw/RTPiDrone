from scipy.stats import norm
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def fitFunc(t, a, b, c):
    return a*t**0.5 + b*t**0.25 + c

start = 1820
for index in range(4) :
    power = []
    mag0 = []
    mag1 = []
    mag2 = []
    mag0err = []
    mag1err = []
    mag2err = []
    mag = [[], [], []]
    magErr = [[], [], []]

    offset = [0.0,0.0,0.0]

    strs = "HMC5883L_PWM_"+str(index)+".log"

    f = open(strs, 'r')

    while True :
        i = f.readline()
        if i=='': break
        tA = float((i.split('\t'))[0])
        if (tA >= 1650 and tA < 1750):
            for k in range(3):
                offset[k] += float((i.split('\t'))[k*2+1])
        if (tA == 1800):
            for k in range(3):
                offset[k] /= 100
        if (tA>start):
            power.append(tA)
            for k in range(3):
                mag[k].append(float((i.split('\t'))[1+k*2])-offset[k])
                magErr[k].append(float((i.split('\t'))[2+k*2]))

    f.close()

    po = np.asarray(power)

    print('{\t')
    for k in range(3):
        fitParams, fitCovariances = curve_fit(fitFunc, power, mag[k], sigma=magErr[k])
        sigma = [fitCovariances[0,0], fitCovariances[1,1], fitCovariances[2,2]]

        plt.ylabel('Mag'+str(k), fontsize = 16) 
        plt.xlabel('Power'+str(index), fontsize = 16) 
        plt.xlim(start,3280)
        # plot the data as red circles with vertical errorbars
        plt.errorbar(power, mag[k], fmt = 'ro', yerr = magErr[k])

        plt.plot(power, fitFunc(po, fitParams[0], fitParams[1], fitParams[2]))   

        strs = "Motor"+str(index)+"_Meg"+str(k)+".png"
        plt.savefig(strs, bbox_inches=0)

        print('{'+str(fitParams[0])+',\t'+str(fitParams[1])+',\t'+str(fitParams[2])+'}')
        #print(fitCovariances)
        plt.close()

    print('}')
