from glider import Glider
import csv
import matplotlib.pyplot as plt
import numpy as np
glider=Glider()

num_iterations=1000
depth_values=[]
be_input_values=[]
glider.depthDes= 40
glider.veloDes = 0
maxVeloDes = 0.06
minVeloDes = 0
maxError = 1
minError = 0.4
minmaxErrorDiff = maxError-minError

def add_gaussian_noise(depth_measurement, mean, std_dev):
    noise = np.random.normal(mean, std_dev)
    noisy_depth = depth_measurement + noise
    return noisy_depth

for _ in range (num_iterations):
    depth,be_input=glider.depthMechanism()
    depth=add_gaussian_noise(depth, 0, 0.1)
    depth_values.append(depth)
    be_input_values.append(be_input)
    errorDepth = glider.depthDes - depth
    # Velocity Variable
    if (abs(errorDepth) > maxError):
        veloDes = maxVeloDes * (errorDepth)/abs(errorDepth)
        print("Velo menuju {}".format(veloDes))
    elif (abs(errorDepth) < minError):
        veloDes = 0
        print("Menuju nol velonya")
    else:
        veloDes = maxVeloDes/minmaxErrorDiff*(errorDepth)-((minError*maxVeloDes/minmaxErrorDiff)*(errorDepth)/abs(errorDepth))
        print("Transient velo")
        
    control_signal_pid = glider.calculatePID(veloDes, glider.veloNow, 200, 0, 1000)
    # control_signal = glider.controlDepthMPC(1)
    # if(control_signal>):
    #     control_signal=2
    # elif(control_signal<-2):
    #     control_signal=-2
    # elif(control_signal == 0):
    #     control_signal = 0
    # else:
    #     control_signal=3*(control_signal/abs(control_signal))
    glider.BE_input += control_signal_pid
    # glider.BE_input += control_signal_mpc[0][0]
    
    if glider.BE_input>700:
        glider.BE_input=700
    elif glider.BE_input<100:
        glider.BE_input=100

with open('simulation_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Iteration', 'Depth', 'BE_input'])
    for i in range(num_iterations):
        writer.writerow([i+1, depth_values[i], be_input_values[i]])
    
iterations = list(range(1, num_iterations+1))
plt.subplot(2,1,1)
plt.plot(iterations, depth_values)

plt.xlabel('Iteration')
plt.ylabel('Depth')
plt.title('Depth Simulation')
plt.subplot(2,1,2)
plt.plot(iterations, be_input_values)
plt.show()

