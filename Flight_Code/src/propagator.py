from math import sin, sqrt, pow


a = [0,0,-9.8] # inertial acceleration [m/s^2]
v = [0,0,0] # inertial velocity [m/s]
r = [0,0,0] # inertial position [m]
# o = [0,0,0,1]
w = [0,0,0] # angular velocity [rad/s]

t = 0 # time [s]

x = 0; y = 1; z = 2
lat = 0
long = 0

updateRate = 25 # [Hz]
timeStep = 1/updateRate # [s]
totalImpulse = 32000 # [Ns]
burnTime = 4.5 # [s]
rocketThrust = totalImpulse/burnTime # [N]

dryMass = 39; # [kg]
wetMass = 56.25; # [kg]
m = wetMass

CDr = .55
CDf = .95
flapArea = 0.0839
rocketArea = 0.0182

def Propagate(flapAngle):
    global t, a, v, r, x, y, z, lat, long, updateRate, timeStep, totalImpulse, burnTime, rocketThrust, dryMass, wetMass, m, CDr, CDf, flapArea, rocketArea
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
        a[z] = motorAccel + 0.5/m*density*(4*CDf*flapArea*sin(flapAngle) + CDr*rocketArea)*sqrt(v[z]*v[z])*v[z] - 9.8
    else:
        a[z] = 0.5/m*density*(4*CDf*flapArea*sin(flapAngle) + CDr*rocketArea)*sqrt(v[z]*v[z])*v[z] - 9.8

    #update Velocity
    v[x] = v[x] + timeStep*a[x]
    v[y] = v[y] + timeStep*a[y]
    v[z] = v[z] + timeStep*a[z]

    #update Position
    r[x] = r[x] + timeStep*v[x] + .5*a[x]*timeStep**2
    r[y] = r[y] + timeStep*v[y] + .5*a[y]*timeStep**2
    r[z] = r[z] + timeStep*v[z] + .5*a[z]*timeStep**2

    #update orientation
    # o[w] = sqrt(v[x]^2 + v[y]^2 + v[z]^2)
    # o[x] = v[x]/o[w]
    # o[y] = v[y]/o[w]
    # o[z] = v[z]/o[w]

    return 

def getDensity(h):
    """
    Calculate air density given altitude h (in meters ASL).
    
    :param h: Altitude above sea level (m)
    :return: Air density (kg/m³)
    """
    # Constants
    R = 8.31446   # Universal gas constant (J/(mol·K))
    M = 0.0289652 # Molar mass of air (kg/mol)
    L = 0.0065    # Temperature lapse rate in the troposphere (K/m)

    p0 = 101325   # Ground-level pressure (Pa)
    T0 = 288.15   # Ground-level temperature (K)
    
    # Density calculation
    density = (p0 * M) / (R * T0) * pow((1 - L * h / T0), ((9.8 * M / (R * L)) - 1))
    
    return density

def getPressure(h):

    # Constants
    R = 8.31446   # Universal gas constant (J/(mol·K))
    M = 0.0289652 # Molar mass of air (kg/mol)
    g0 = 9.81     # [m/s^2]
    Pb = 101325   # [Pa]
    Lb = -0.0065  # Temperature lapse rate in the troposphere (K/m)
    Tb = 288      # [K]
    h0 = 0        # [m]

    pressure = Pb * pow((1 + (Lb/Tb) * (h - h0)), (-g0*M/(R*Lb)))

    return pressure

def getTemperature(h):

    # Constants
    R = 8.31446   # Universal gas constant (J/(mol·K))

    temperature = getPressure(h)/(R*getDensity(h)) # ideal gas law

    return temperature

