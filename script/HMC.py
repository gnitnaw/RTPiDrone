from scipy.stats import norm
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def fitFunc(t, a, b, c):
    return a*t**0.5 + b*t + c

power = []
mag0 = []
mag1 = []
mag2 = []
mag0err = []
mag1err = []
mag2err = []

offset0 = 0
offset1 = 0
offset2 = 0

f = open('HMC5883L_PWM_3.log', 'r')
while True :
    i = f.readline()
    if i=='': break
    tA = float((i.split('\t'))[0])
    if (tA == 1800):
        offset0 = float((i.split('\t'))[1])
        offset1 = float((i.split('\t'))[3])
        offset2 = float((i.split('\t'))[5])
    if (tA>=1800):
        power.append(tA)
        mag0.append(float((i.split('\t'))[1])-offset0)
        mag1.append(float((i.split('\t'))[3])-offset1)
        mag2.append(float((i.split('\t'))[5])-offset2)
        mag0err.append(float((i.split('\t'))[2]))
        mag1err.append(float((i.split('\t'))[4]))
        mag2err.append(float((i.split('\t'))[6]))
f.close()

po = np.asarray(power)

fitParams0, fitCovariances0 = curve_fit(fitFunc, power, mag0, sigma=mag0err)
sigma0 = [fitCovariances0[0,0], fitCovariances0[1,1], fitCovariances0[2,2] ]

fitParams1, fitCovariances1 = curve_fit(fitFunc, power, mag1, sigma=mag1err)
sigma1 = [fitCovariances1[0,0], fitCovariances1[1,1], fitCovariances1[2,2] ]

fitParams2, fitCovariances2 = curve_fit(fitFunc, power, mag2, sigma=mag2err)
sigma2 = [fitCovariances2[0,0], fitCovariances2[1,1], fitCovariances2[2,2] ]

plt.ylabel('Mag', fontsize = 16) 
plt.xlabel('Power', fontsize = 16) 
plt.xlim(1800,3280)
# plot the data as red circles with vertical errorbars
plt.errorbar(power, mag0, fmt = 'ro', yerr = mag0err)

sigma = [fitCovariances0[0,0], fitCovariances0[1,1], fitCovariances0[2,2] ]
plt.plot(power, fitFunc(po, fitParams0[0], fitParams0[1], fitParams0[2]))   

plt.savefig('Motor3_Meg0.png', bbox_inches=0)

print(fitParams0)
print(fitCovariances0)
plt.close()

plt.ylabel('Mag', fontsize = 16)
plt.xlabel('Power', fontsize = 16)
plt.xlim(1800,3280)

# plot the data as red circles with vertical errorbars
plt.errorbar(power, mag1, fmt = 'ro', yerr = mag1err)

sigma = [fitCovariances1[0,0], fitCovariances1[1,1], fitCovariances1[2,2] ]
plt.plot(power, fitFunc(po, fitParams1[0], fitParams1[1], fitParams1[2]))

plt.savefig('Motor3_Meg1.png', bbox_inches=0)

print(fitParams1)
print(fitCovariances1)
plt.close()


plt.ylabel('Mag', fontsize = 16)
plt.xlabel('Power', fontsize = 16)
plt.xlim(1800,3280)

# plot the data as red circles with vertical errorbars
plt.errorbar(power, mag2, fmt = 'ro', yerr = mag2err)

sigma = [fitCovariances2[0,0], fitCovariances2[1,1], fitCovariances2[2,2] ]
plt.plot(power, fitFunc(po, fitParams2[0], fitParams2[1], fitParams2[2]))

plt.savefig('Motor3_Meg2.png', bbox_inches=0)

print(fitParams2)
print(fitCovariances2)


