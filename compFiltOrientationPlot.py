import serial
from vpython import *
import matplotlib.pyplot as plt

# read data from microcontroller
imu_data = serial.Serial('com6', 115200)

# function to setup vpython animation, calculate orientation values and display 3D render of IMU in space


def orientation_calcs(imu_data):

    # setup vpython scene
    scene.range = 3
    scene.forward = vector(-1, -1, -1)
    scene.width = 600
    scene.height = 600

    # setup fixed x, y and z axes
    x_arrow = arrow(axis=vector(1, 0, 0), length=1.5,
                    shaftwidth=0.1, color=color.red)
    y_arrow = arrow(axis=vector(0, 1, 0), length=1.5,
                    shaftwidth=0.1, color=color.green)
    z_arrow = arrow(axis=vector(0, 0, 1), length=1.5,
                    shaftwidth=0.1, color=color.blue)
    x_arrow.opacity = 0.3
    y_arrow.opacity = 0.3
    z_arrow.opacity = 0.3

    # setup imu actual axes
    x_imu_arrow = arrow(axis=vector(1, 0, 0), length=3,
                        shaftwidth=0.1, color=color.red)
    y_imu_arrow = arrow(axis=vector(0, 1, 0), lenght=3,
                        shaftwidth=0.1, color=color.green)
    z_imu_arrow = arrow(axis=vector(0, 0, 1), length=3,
                        shaftwidth=0.1, color=color.blue)
    imu = box(length=3, width=0.5, height=0.2,
              opacity=0.5, color=color.white)
    imu.opacity = 0.2

    while (True):
        try:
            while (imu_data.inWaiting() == 0):
                pass
            # read serial data from microcontroller
            raw_packet = imu_data.readline().strip()
            raw_packet = str(raw_packet, 'utf-8')

            # split serial data by comma separator to obtain orientation data
            fin_packet = raw_packet.split(',')
            pitch = (float(fin_packet[0]))
            roll = -(float(fin_packet[1]))
            yaw = -(float(fin_packet[2]))

            pitch_rad = pitch*pi/180
            roll_rad = roll*pi/180
            yaw_rad = yaw*pi/180

            # alter imu actual axes positions using orientation values
            k = vector(cos(yaw_rad)*cos(pitch_rad),
                       sin(pitch_rad), sin(yaw_rad)*cos(pitch_rad))
            y = vector(0, 1, 0)
            s = cross(k, y)
            v = cross(s, k)
            vrot = v*cos(roll_rad) + cross(k, v)*sin(roll_rad)
            x_imu_arrow.axis = k
            y_imu_arrow.axis = vrot
            z_imu_arrow.axis = cross(k, vrot)
            imu.axis = k
            imu.up = vrot

            # live_plot(roll, pitch, yaw)
        except:
            pass

# function to plot 3x3 orientation plot


def live_plot(roll, pitch, yaw):

    # create list storing orientation values
    roll_plot.append(roll)
    pitch_plot.append(pitch)
    yaw_plot.append(yaw)

    # plot orientation values
    plt.ion()
    ax.plot(roll_plot, pitch_plot, yaw_plot)
    plt.pause(0.01)
    ax.set_xlabel('Roll')
    ax.set_ylabel('Pitch')
    ax.set_zlabel('Yaw')
    ax.set_ylim(-180, 180)
    ax.set_xlim(-180, 180)
    ax.set_zlim(-180, 180)

    if len(roll_plot) >= 100:
        # remove the first 50 elements to reduce processing time
        del roll_plot[:50]
        ax.clear()

    if len(pitch_plot) >= 100:
        # remove the first 50 elements to reduce processing time
        del pitch_plot[:50]
        ax.clear()

    if len(yaw_plot) >= 100:
        # remove the first 50 elements to reduce processing time
        del yaw_plot[:50]
        ax.clear()


# main section of the code
roll_plot = []
pitch_plot = []
yaw_plot = []
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
orientation_calcs(imu_data)
plt.show()
