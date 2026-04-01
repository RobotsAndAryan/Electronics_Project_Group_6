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
fig.canvas.manager.set_window_title('ALRS-007 GCS')
fig.suptitle('ALRS-007 COMMAND & CONTROL (HYBRID THERMAL)', fontsize=18, fontweight='bold', color='orange', y=0.95)

gs = gridspec.GridSpec(2, 3, height_ratios=[1.2, 1], width_ratios=[1, 1, 1], hspace=0.3, wspace=0.3)

# 1. THERMAL HUD
ax_thermal = fig.add_subplot(gs[0, 0])
grid_data = np.zeros((8, 8))
im = ax_thermal.imshow(grid_data, cmap='jet', vmin=20, vmax=35, interpolation='bicubic')
ax_thermal.set_title("THERMAL RECON (QWIIC)", color='white', fontweight='bold', pad=10)
ax_thermal.axis('off')

# Text Overlays for Thermal
txt_maxtemp = ax_thermal.text(0.05, 0.90, "MAX: --.- C", transform=ax_thermal.transAxes, color='yellow', fontweight='bold', fontsize=12)
txt_hotspot = ax_thermal.text(0, 0, "", color='white', fontweight='bold', ha='center', va='center', fontsize=10, bbox=dict(facecolor='black', alpha=0.5, edgecolor='none', pad=1))

ax_att = fig.add_subplot(gs[0, 1])
ax_att.set_xlim(-90, 90)
ax_att.set_ylim(-90, 90)
ax_att.set_title("ATTITUDE (AHRS)", color='gray', fontweight='bold', pad=10)
ax_att.grid(color='#222222', linestyle='-', linewidth=1)
ax_att.text(0, 0, "IMU OFFLINE\nSENSOR BYPASSED", color='red', fontsize=14, fontweight='bold', ha='center', va='center')

ax_data = fig.add_subplot(gs[0, 2])
ax_data.axis('off')
ax_data.set_title("MISSION TELEMETRY", color='white', fontweight='bold', pad=10)

txt_time  = ax_data.text(0.1, 0.9, "T+ 0.0s", fontsize=20, color='white', fontweight='bold')
txt_state = ax_data.text(0.1, 0.75, "STATE: STANDBY", fontsize=14, color='cyan')
txt_alt   = ax_data.text(0.1, 0.60, "ALT:   0.0 cm", fontsize=14, color='yellow')
txt_gforce= ax_data.text(0.1, 0.35, "CDSI METRICS OFFLINE", fontsize=12, color='red')
txt_fsr   = ax_data.text(0.1, 0.20, "FSR [0, 0, 0]", fontsize=14, color='gray')

ax_graph = fig.add_subplot(gs[1, :])
ax_graph.set_title("DYNAMIC IMPACT TRACE (OFFLINE)", color='gray', pad=10, fontweight='bold')
ax_graph.set_ylabel("Dynamic Magnitude (G)", color='gray')
ax_graph.set_xlabel("Time Since Impact (s)", color='gray')
ax_graph.grid(True, color='#222222', linestyle='-', linewidth=1)
ax_graph.set_xlim(0, 5.0) 
ax_graph.set_ylim(0, 10.0) 
ax_graph.tick_params(colors='gray')

btn_ax = fig.add_axes([0.45, 0.02, 0.1, 0.05]) 
btn_launch = Button(btn_ax, 'LAUNCH', color='#660000', hovercolor='#ff0000')
btn_launch.label.set_fontsize(12)
btn_launch.label.set_fontweight('bold')
btn_launch.label.set_color('white')

def send_launch(event):
    print("\n[UPLINK] Transmitting LAUNCH sequence to capsule...")
    sock.sendto(b"LAUNCH", (CAPSULE_IP, UDP_PORT))
    btn_launch.color = 'green'
    btn_launch.label.set_text("ARMED")
    fig.canvas.draw()

btn_launch.on_clicked(send_launch)
plt.subplots_adjust(bottom=0.12, top=0.88, left=0.05, right=0.95) 

states = {
    0: "0 - PRE-FLIGHT",
    1: "1 - RECON", 
    2: "2 - DESCENT", 
    3: "3 - ARMED", 
    4: "4 - CRASH", 
    5: "5 - SECURE/TOUCHDOWN"
}

previous_state = 0
last_debug_print = time.time()
packets_received = 0
last_fsr1, last_fsr2, last_fsr3 = 0, 0, 0

print("\n======================================")
print("[*] GCS Online. Hybrid Thermal Anchor Active.")
print(f"[*] Expected Packet Size: {PACKET_SIZE} bytes")
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
                packets_received += 1
            except BlockingIOError:
                break 
                
        current_time = time.time()
        if current_time - last_debug_print > 2.0:
            if packets_received == 0:
                print(f"[WARNING] No telemetry received. Connect to '{CAPSULE_IP}'.")
            else:
                print(f"[STATUS] Telemetry Link Active. FSR Base: [{last_fsr1}, {last_fsr2}, {last_fsr3}]")
                packets_received = 0
            last_debug_print = current_time
            
        if latest_data and len(latest_data) == PACKET_SIZE:
            unpacked = struct.unpack(PACKET_FORMAT, latest_data)
            
            timestamp_ms = unpacked[0]
            state_id = unpacked[1]
            altitude = unpacked[2]
            
            last_fsr1 = unpacked[8]
            last_fsr2 = unpacked[9]
            last_fsr3 = unpacked[10]
            
            thermal_flat = unpacked[11:]
            
            # --- THERMAL UI UPDATES (HYBRID ANCHOR PATCH) ---
            thermal_matrix = np.array(thermal_flat).reshape((8, 8))
            
            # 1. Fix Spatial Inversion
            thermal_matrix = np.fliplr(thermal_matrix)
            
            # 2. Filter garbage data
            thermal_matrix = np.clip(thermal_matrix, 0, 150) 
            
            current_max = np.max(thermal_matrix)
            
            # 3. HYBRID SCALING: Floor is locked to 20C. Ceiling stretches to fit the hottest object, but never drops below 28C.
            visual_floor = 20.0
            visual_ceiling = max(28.0, current_max)
            im.set_clim(visual_floor, visual_ceiling)
            im.set_data(thermal_matrix)
            
            # 4. RAW NUMBER OVERLAY: Print the exact temperature on the hottest pixel
            max_idx = np.unravel_index(np.argmax(thermal_matrix), thermal_matrix.shape)
            txt_hotspot.set_position((max_idx[1], max_idx[0])) # x, y
            txt_hotspot.set_text(f"{current_max:.1f}")
            
            txt_maxtemp.set_text(f"MAX: {current_max:.1f} C")
            if current_max > 30.0: txt_maxtemp.set_color('red')
            else: txt_maxtemp.set_color('yellow')
            # ----------------------------------------------

            state_str = states.get(state_id, "ERR")
            txt_state.set_text(f"STATE: {state_str}")
            
            alt_color = 'yellow' if altitude > 50.0 else 'red'
            txt_alt.set_text(f"ALT:   {altitude:.1f} cm")
            txt_alt.set_color(alt_color)
            
            txt_time.set_text(f"T+ {timestamp_ms/1000.0:.1f}s")
            txt_fsr.set_text(f"FSR [{last_fsr1}, {last_fsr2}, {last_fsr3}]")
            
            if previous_state == 4 and state_id >= 5:
                print("\n" + "="*45)
                print(">>> POST-CRASH ANALYSIS COMPLETE <<<")
                
                fsr_vals = [last_fsr1, last_fsr2, last_fsr3]
                max_fsr = max(fsr_vals)
                min_fsr = min(fsr_vals)
                
                if (max_fsr - min_fsr) > 250:
                    print(f"[!] WARNING: ASYMMETRIC LANDING DETECTED [!]")
                    print(f"    Resting Load Spread: {fsr_vals}")
                    print(f"    Capsule is resting at a severe angle.")
                    txt_fsr.set_color('red')
                else:
                    print(f"[OK] Symmetric Landing Confirmed.")
                    print(f"    Resting Load Spread: {fsr_vals}")
                    txt_fsr.set_color('lime')
                    
                print("=============================================\n")
                
            previous_state = state_id
            
        fig.canvas.flush_events()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("\n[*] GCS Shutting Down.")
    sock.close()
    plt.close()