from math import exp, sin, sqrt


a = [0,0,-9.8]
v = [0,0,0]
r = [0,0,0]
o = [0,0,1,1]

t = 0

x = 0; y = 1; z = 2; w = 3

timeStep = .1
totalImpulse = 10
burnTime = 1
rocketThrust = totalImpulse/burnTime

wetMass = 70
dryMass = 50
m = wetMass

CDr = .55
CDf = .95
flapArea = 0.0839
rocketArea = 0.0182

#radians
def Propagate(flapAngle):
    t += timeStep
    density = getDensity(r[z])


    #update Mass
    if t < burnTime:
        m -= (wetMass - dryMass)/burnTime*timeStep
    else:
        m = dryMass
    
    #update Acceleration
    if t <burnTime:
        motorAccel = rocketThrust / m
        a[z] = motorAccel + 0.5/m*density*(4*CDf*flapArea*sin(flapAngle) + CDr*rocketArea)*v[z]*v[z] - 9.8
    else:
        a[z] = 0.5/m*density*(4*CDf*flapArea*sin(flapAngle) + CDr*rocketArea)*v[z]*v[z] - 9.8

    #update Velocity
    v[x] = v[x] + timeStep*a[x]
    v[y] = v[y] + timeStep*a[y]
    v[z] = v[z] + timeStep*a[z]

    #update Position
    p[x] = p[x] + timeStep*v[x] + .5*a[x]*timeStep**2
    p[y] = p[y] + timeStep*v[y] + .5*a[y]*timeStep**2
    p[z] = p[z] + timeStep*v[z] + .5*a[z]*timeStep**2

    #update orientation
    o[w] = sqrt(v[x]^2 + v[y]^2 + v[z]^2)
    o[x] = v[x]/o[w]
    o[y] = v[y]/o[w]
    o[z] = v[z]/o[w]

    return 

def getDensity(altitude):
    return 1.32*exp(-1.23E-04*altitude)