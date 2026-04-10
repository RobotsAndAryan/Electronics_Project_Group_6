import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button
import time

# --- NETWORK CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 2390
CAPSULE_IP = "192.168.4.1" 

PACKET_FORMAT = '<IB6f3H64f'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# --- DASHBOARD SETUP ---
plt.ion() 
plt.style.use('dark_background')
fig = plt.figure(figsize=(14, 9)) 
fig.canvas.manager.set_window_title('GCS')
fig.suptitle('Landing Capsule COMMAND & CONTROL', fontsize=18, fontweight='bold', color='orange', y=0.95)

gs = gridspec.GridSpec(2, 3, height_ratios=[1.2, 1], width_ratios=[1, 1, 1], hspace=0.3, wspace=0.3)

# 1. THERMAL HUD
ax_thermal = fig.add_subplot(gs[0, 0])
grid_data = np.zeros((8, 8))
im = ax_thermal.imshow(grid_data, cmap='jet', vmin=20, vmax=35, interpolation='bicubic')
ax_thermal.set_title("THERMAL RECON (AMG8833)", color='white', fontweight='bold', pad=10)
ax_thermal.axis('off')

txt_maxtemp = ax_thermal.text(0.05, 0.90, "MAX: --.- C", transform=ax_thermal.transAxes, color='yellow', fontweight='bold', fontsize=12)
txt_hotspot = ax_thermal.text(0, 0, "", color='white', fontweight='bold', ha='center', va='center', fontsize=10, bbox=dict(facecolor='black', alpha=0.5, edgecolor='none', pad=1))

# 2. AHRS
ax_att = fig.add_subplot(gs[0, 1])
ax_att.set_xlim(-90, 90)
ax_att.set_ylim(-90, 90)
ax_att.set_title("ATTITUDE (IMU AHRS)", color='white', fontweight='bold', pad=10)
ax_att.set_xlabel("Roll (°)", color='gray')
ax_att.set_ylabel("Pitch (°)", color='gray')
ax_att.grid(color='#222222', linestyle='-', linewidth=1)
ax_att.axhline(0, color='white', linewidth=2)
ax_att.axvline(0, color='gray', linewidth=1, linestyle='--')
for p in [-60, -40, -20, 20, 40, 60]: ax_att.plot([-15, 15], [p, p], color='gray', linewidth=1)
circle = plt.Circle((0, 0), 35, color='green', fill=False, linestyle='--', linewidth=2)
ax_att.add_patch(circle)
crosshair, = ax_att.plot([0], [0], marker='+', color='red', markersize=20, markeredgewidth=3)

# 3. TELEMETRY
ax_data = fig.add_subplot(gs[0, 2])
ax_data.axis('off')
ax_data.set_title("MISSION TELEMETRY", color='white', fontweight='bold', pad=10)

txt_time  = ax_data.text(0.1, 0.9, "T+ 0.0s", fontsize=20, color='white', fontweight='bold')
txt_state = ax_data.text(0.1, 0.75, "STATE: STANDBY", fontsize=14, color='cyan')
txt_alt   = ax_data.text(0.1, 0.60, "ALT:   0.0 cm", fontsize=14, color='yellow')
txt_gforce= ax_data.text(0.1, 0.40, "PEAK G: 0.00", fontsize=14, color='gray')
txt_cdsi  = ax_data.text(0.1, 0.25, "CDSI:   0.000", fontsize=14, color='gray')
txt_class = ax_data.text(0.1, 0.10, "CLASS:  N/A", fontsize=14, color='gray')

# 4. IMPACT TRACE
ax_graph = fig.add_subplot(gs[1, :])
ax_graph.set_title("DYNAMIC IMPACT TRACE (ADC RAW G-FORCE)", color='cyan', pad=10, fontweight='bold')
ax_graph.set_ylabel("Dynamic Magnitude (G)", color='white')
ax_graph.set_xlabel("Time Since Impact (s)", color='gray')
ax_graph.grid(True, color='#222222', linestyle='-', linewidth=1)
ax_graph.set_xlim(0, 5.0) 
ax_graph.set_ylim(0, 10.0) 
ax_graph.tick_params(colors='gray')
line_gforce, = ax_graph.plot([], [], color='cyan', linewidth=2)

btn_ax = fig.add_axes([0.45, 0.02, 0.1, 0.05]) 
btn_launch = Button(btn_ax, 'LAUNCH', color='#660000', hovercolor='#ff0000')
btn_launch.label.set_fontsize(12)
btn_launch.label.set_fontweight('bold')
btn_launch.label.set_color('white')

def send_launch(event):
    print("\n[UPLINK] Transmitting LAUNCH sequence...")
    sock.sendto(b"LAUNCH", (CAPSULE_IP, UDP_PORT))
    btn_launch.color = 'green'
    btn_launch.label.set_text("ARMED")
    fig.canvas.draw()

btn_launch.on_clicked(send_launch)
plt.subplots_adjust(bottom=0.12, top=0.88, left=0.05, right=0.95) 

states = {0: "0 - STANDBY", 1: "1 - RECON", 2: "2 - DESCENT", 3: "3 - ARMED", 4: "4 - CRASH", 5: "5 - SECURE"}

previous_state = 0
last_debug_print = time.time()
impact_t_data, impact_g_data = [], []
impact_start_time = None

print("\n======================================")
print("[*] GCS Online. FSR Trigger + IMU Physics Active.")
print("======================================\n")

fig.canvas.draw()
fig.canvas.flush_events()

try:
    while True:
        latest_data = None
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                latest_data = data
            except BlockingIOError:
                break 
                
        if latest_data and len(latest_data) == PACKET_SIZE:
            unpacked = struct.unpack(PACKET_FORMAT, latest_data)
            
            timestamp_ms = unpacked[0]
            state_id = unpacked[1]
            altitude = unpacked[2]
            pitch = unpacked[3]
            roll = unpacked[4]
            ax = unpacked[5]
            ay = unpacked[6]
            az = unpacked[7]
            thermal_flat = unpacked[11:]
            
            # 1. Thermal UI
            thermal_matrix = np.array(thermal_flat).reshape((8, 8))
            thermal_matrix = np.fliplr(thermal_matrix)
            thermal_matrix = np.clip(thermal_matrix, 0, 150) 
            current_max = np.max(thermal_matrix)
            
            im.set_clim(20.0, max(28.0, current_max))
            im.set_data(thermal_matrix)
            max_idx = np.unravel_index(np.argmax(thermal_matrix), thermal_matrix.shape)
            txt_hotspot.set_position((max_idx[1], max_idx[0])) 
            txt_hotspot.set_text(f"{current_max:.1f}")
            txt_maxtemp.set_text(f"MAX: {current_max:.1f} C")
            txt_maxtemp.set_color('red' if current_max > 30.0 else 'yellow')

            # 2. AHRS UI
            crosshair.set_data([roll], [pitch])
            crosshair.set_color('red' if abs(pitch) > 35 or abs(roll) > 35 else 'lime')

            # 3. Telemetry Text
            txt_state.set_text(f"STATE: {states.get(state_id, 'ERR')}")
            txt_alt.set_text(f"ALT:   {altitude:.1f} cm")
            txt_alt.set_color('yellow' if altitude > 50.0 else 'red')
            txt_time.set_text(f"T+ {timestamp_ms/1000.0:.1f}s")
            
            # 4. IMU IMPACT TRACING
            if state_id == 4:
                if impact_start_time is None:
                    impact_start_time = timestamp_ms / 1000.0
                rel_t = (timestamp_ms / 1000.0) - impact_start_time
                
                # Convert raw m/s^2 to G-Force, subtract 1G resting gravity to get dynamic impact
                total_g = np.sqrt(ax**2 + ay**2 + az**2) / 9.81
                dynamic_g = abs(total_g - 1.0) 
                
                impact_t_data.append(rel_t)
                impact_g_data.append(dynamic_g)
                line_gforce.set_data(impact_t_data, impact_g_data)
                
                if dynamic_g > ax_graph.get_ylim()[1]:
                    ax_graph.set_ylim(0, dynamic_g + 2.0)

            # 5. CDSI POST-FLIGHT ANALYSIS
            if previous_state == 4 and state_id >= 5:
                if len(impact_t_data) > 1:
                    peak_g = np.max(impact_g_data)
                    G_safe = 5.0 # From Chapter 2
                    S_new = peak_g / G_safe
                    
                    if S_new <= 1.0:
                        classification, c_color = "SOFT", 'lime'
                    elif S_new <= 1.5:
                        classification, c_color = "MARGINAL", 'yellow'
                    else:
                        classification, c_color = "HARD", 'red'
                        
                    txt_gforce.set_color('white')
                    txt_gforce.set_text(f"PEAK G: {peak_g:.2f}")
                    txt_cdsi.set_text(f"CDSI:   {S_new:.3f}")
                    txt_cdsi.set_color('white')
                    txt_class.set_text(f"CLASS:  {classification}")
                    txt_class.set_color(c_color)
                    line_gforce.set_color(c_color)
                    
                    print("\n" + "="*45)
                    print(">>> IMU POST-CRASH ANALYSIS COMPLETE <<<")
                    print(f"Peak Dynamic G:   {peak_g:.2f} G")
                    print(f"CDSI Score:       {S_new:.3f}")
                    print(f"Classification:   {classification}")
                    print("="*45 + "\n")
                    
            previous_state = state_id
            
        fig.canvas.flush_events()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("\n[*] GCS Shutting Down.")
    sock.close()
    plt.close()