import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button
import time

myIP = "0.0.0.0"
myPort = 2390
arduinoIP = "192.168.4.1" 

# I is unsigned int, B is byte, f is float, H is unsigned short (for FSR)
formatString = '<IB6f3H64f'
packetLength = struct.calcsize(formatString)

mySock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mySock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
mySock.bind((myIP, myPort))
mySock.setblocking(False)

# setup the dashboard visuals
plt.ion() 
plt.style.use('dark_background')
fig = plt.figure(figsize=(14, 9)) 
fig.canvas.manager.set_window_title('GCS')
fig.suptitle('Landing Capsule Command & Control', fontsize=18, fontweight='bold', color='orange')

myGrid = gridspec.GridSpec(2, 3, height_ratios=[1.2, 1], width_ratios=[1, 1, 1], hspace=0.3, wspace=0.3)

# Thermal picture
ax1 = fig.add_subplot(myGrid[0, 0])
emptyData = np.zeros((8, 8))
thermalImage = ax1.imshow(emptyData, cmap='jet', vmin=20, vmax=35, interpolation='bicubic')
ax1.set_title("THERMAL RECON (AMG8833)", color='white', fontweight='bold')
ax1.axis('off')

maxTempText = ax1.text(0.05, 0.90, "Temp: 0.0", transform=ax1.transAxes, color='yellow')
hotspotLabel = ax1.text(0, 0, "", color='white', ha='center', va='center')

# IMU Roll and Pitch
ax2 = fig.add_subplot(myGrid[0, 1])
ax2.set_xlim(-90, 90)
ax2.set_ylim(-90, 90)
ax2.set_title("ATTITUDE(IMU AHRS)", color='white', fontweight='bold')
ax2.grid(color='#222222', linestyle='-')
ax2.axhline(0, color='white')
ax2.axvline(0, color='gray', linestyle='--')
myCircle = plt.Circle((0, 0), 35, color='green', fill=False, linestyle='--')
ax2.add_patch(myCircle)
crosshair, = ax2.plot([0], [0], marker='+', color='red', markersize=20)

# Text Info
ax3 = fig.add_subplot(myGrid[0, 2])
ax3.axis('off')
timeText = ax3.text(0.1, 0.9, "Time: 0.0s", fontsize=18, color='white')
stateText = ax3.text(0.1, 0.75, "State: Waiting", fontsize=14, color='cyan')
altText = ax3.text(0.1, 0.60, "Alt: 0.0 cm", fontsize=14, color='yellow')
gforceText = ax3.text(0.1, 0.40, "Peak G: 0.00", fontsize=14, color='gray')
cdsiText = ax3.text(0.1, 0.25, "CDSI Score: 0.00", fontsize=14, color='gray')
classText = ax3.text(0.1, 0.10, "Rating: N/A", fontsize=14, color='gray')

# The Graph for the crash
ax4 = fig.add_subplot(myGrid[1, :])
ax4.set_title("DYNAMIC IMPACT TRACE (ADC RAW G-FORCE)", color='cyan', fontweight='bold')
ax4.set_ylabel("Dynamic Magnitude (G)")
ax4.set_xlabel("Time Since Impact (s)")
ax4.grid(True, color='#222222')
ax4.set_xlim(0, 5.0) 
ax4.set_ylim(0, 10.0) 
graphLine, = ax4.plot([], [], color='cyan', linewidth=2)

# Launch button at the bottom
buttonArea = fig.add_axes([0.45, 0.02, 0.1, 0.05]) 
launchBtn = Button(buttonArea, 'LAUNCH', color='darkred')
launchBtn.label.set_color('white')

def pressLaunch(event):
    print("Sending launch signal to arduino...")
    mySock.sendto(b"LAUNCH", (arduinoIP, myPort))
    launchBtn.color = 'green'
    launchBtn.label.set_text("Sent")
    fig.canvas.draw()

launchBtn.on_clicked(pressLaunch)

stateNames = {0: "0-Standby", 1: "1-Recon", 2: "2-Dropping", 3: "3-Armed", 4: "4-Crashed", 5: "5-Done"}

oldState = 0
printTimer = time.time()
timeData = []
gData = []
crashStartTime = None

print("Dashboard running. Waiting for data...")

plt.subplots_adjust(bottom=0.12, top=0.88, left=0.05, right=0.95) 
fig.canvas.draw()

try:
    while True:
        packetData = None
        while True:
            try:
                data, addr = mySock.recvfrom(1024)
                packetData = data
            except BlockingIOError:
                break 
                
        if packetData != None and len(packetData) == packetLength:
            # unpack the bytes into python variables
            vals = struct.unpack(formatString, packetData)
            
            t_ms = vals[0]
            s_id = vals[1]
            h = vals[2]
            p = vals[3]
            r = vals[4]
            ax = vals[5]
            ay = vals[6]
            az = vals[7]
            thermalArray = vals[11:]
            
            # update thermal map
            t_matrix = np.array(thermalArray).reshape((8, 8))
            t_matrix = np.fliplr(t_matrix) # flip it because the camera is upside down
            
            maxVal = np.max(t_matrix)
            
            thermalImage.set_clim(20.0, max(28.0, maxVal))
            thermalImage.set_data(t_matrix)
            
            hotX, hotY = np.unravel_index(np.argmax(t_matrix), t_matrix.shape)
            hotspotLabel.set_position((hotY, hotX)) 
            hotspotLabel.set_text(str(round(maxVal, 1)))
            
            maxTempText.set_text("Max Temp: " + str(round(maxVal, 1)))
            if maxVal > 30.0:
                maxTempText.set_color('red')
            else:
                maxTempText.set_color('yellow')

            # update attitude
            crosshair.set_data([r], [p])
            if abs(p) > 35 or abs(r) > 35:
                crosshair.set_color('red')
            else:
                crosshair.set_color('lime')

            # update text
            stateText.set_text("State: " + stateNames.get(s_id, "Error"))
            altText.set_text("Alt: " + str(round(h, 1)) + " cm")
            if h > 50.0:
                altText.set_color('yellow')
            else:
                altText.set_color('red')
                
            timeText.set_text("Time: " + str(round(t_ms/1000.0, 1)) + "s")
            
            # draw the graph if we crashed
            if s_id == 4:
                if crashStartTime == None:
                    crashStartTime = t_ms / 1000.0
                timeNow = (t_ms / 1000.0) - crashStartTime
                
                # total G force formula
                rawG = np.sqrt(ax**2 + ay**2 + az**2) / 9.81
                realG = abs(rawG - 1.0) # take away gravity
                
                timeData.append(timeNow)
                gData.append(realG)
                graphLine.set_data(timeData, gData)
                
                if realG > ax4.get_ylim()[1]:
                    ax4.set_ylim(0, realG + 2.0)

            # calculate score when done
            if oldState == 4 and s_id == 5:
                if len(timeData) > 1:
                    highestG = np.max(gData)
                    score = highestG / 5.0 # 5G is the limit
                    
                    if score <= 1.0:
                        rating = "Good Landing"
                        cColor = 'lime'
                    elif score <= 1.5:
                        rating = "Rough Landing"
                        cColor = 'yellow'
                    else:
                        rating = "Crashed Hard"
                        cColor = 'red'
                        
                    gforceText.set_color('white')
                    gforceText.set_text("Peak G: " + str(round(highestG, 2)))
                    cdsiText.set_text("CDSI: " + str(round(score, 3)))
                    cdsiText.set_color('white')
                    classText.set_text("Rating: " + rating)
                    classText.set_color(cColor)
                    graphLine.set_color(cColor)
                    
                    print("\n--- IMU Crash Score ---")
                    print("Highest G: " + str(round(highestG, 2)))
                    print("Score: " + str(round(score, 3)))
                    print("Result: " + rating)
                    print("-----------------------\n")
                    
            oldState = s_id
            
        fig.canvas.flush_events()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("Program closed by user.")
    mySock.close()
    plt.close()