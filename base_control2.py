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

PACKET_FORMAT = '<IB6f64f'
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
fig.suptitle('ALRS-007 COMMAND & CONTROL', fontsize=18, fontweight='bold', color='cyan', y=0.95)

gs = gridspec.GridSpec(2, 3, height_ratios=[1.2, 1], width_ratios=[1, 1, 1], hspace=0.3, wspace=0.3)

# 1. THERMAL HUD
ax_thermal = fig.add_subplot(gs[0, 0])
grid_data = np.zeros((8, 8))
im = ax_thermal.imshow(grid_data, cmap='inferno', vmin=20, vmax=35, interpolation='bicubic')
ax_thermal.set_title("THERMAL RECON", color='white', fontweight='bold', pad=10)
ax_thermal.axis('off')
txt_maxtemp = ax_thermal.text(0.05, 0.90, "MAX: --.- C", transform=ax_thermal.transAxes, color='yellow', fontweight='bold', fontsize=12)

# 2. ATTITUDE INDICATOR
ax_att = fig.add_subplot(gs[0, 1])
ax_att.set_xlim(-90, 90)
ax_att.set_ylim(-90, 90)
ax_att.set_title("ATTITUDE (AHRS)", color='white', fontweight='bold', pad=10)
ax_att.set_xlabel("Roll (°)", color='gray')
ax_att.set_ylabel("Pitch (°)", color='gray')
ax_att.grid(color='#222222', linestyle='-', linewidth=1)
ax_att.axhline(0, color='white', linewidth=2)
ax_att.axvline(0, color='gray', linewidth=1, linestyle='--')

for p in [-60, -40, -20, 20, 40, 60]:
    ax_att.plot([-15, 15], [p, p], color='gray', linewidth=1)
circle = plt.Circle((0, 0), 35, color='green', fill=False, linestyle='--', linewidth=2)
ax_att.add_patch(circle)
crosshair, = ax_att.plot([0], [0], marker='+', color='red', markersize=20, markeredgewidth=3)

# 3. AVIONICS DATA PANEL
ax_data = fig.add_subplot(gs[0, 2])
ax_data.axis('off')
ax_data.set_title("MISSION TELEMETRY", color='white', fontweight='bold', pad=10)

txt_time  = ax_data.text(0.1, 0.9, "T+ 0.0s", fontsize=20, color='white', fontweight='bold')
txt_state = ax_data.text(0.1, 0.75, "STATE: STANDBY", fontsize=14, color='cyan')
txt_alt   = ax_data.text(0.1, 0.60, "ALT:   0.0 cm", fontsize=14, color='yellow')

txt_gforce= ax_data.text(0.1, 0.35, "PEAK G: 0.00", fontsize=14, color='gray')
txt_cdsi  = ax_data.text(0.1, 0.20, "CDSI:   0.000", fontsize=14, color='gray')
txt_class = ax_data.text(0.1, 0.05, "CLASS:  N/A", fontsize=14, color='gray')

# 4. CRASH IMPULSE GRAPH
ax_graph = fig.add_subplot(gs[1, :])
ax_graph.set_title("DYNAMIC IMPACT TRACE", color='cyan', pad=10, fontweight='bold')
ax_graph.set_ylabel("Dynamic Magnitude (G)", color='white')
ax_graph.set_xlabel("Time Since Impact (s)", color='gray')
ax_graph.grid(True, color='#222222', linestyle='-', linewidth=1)
ax_graph.set_xlim(0, 5.0) 
ax_graph.set_ylim(0, 10.0) 
ax_graph.tick_params(colors='gray')
line_gforce, = ax_graph.plot([], [], color='cyan', linewidth=2)
ax_graph.fill_between([0, 5], 6, 100, color='darkred', alpha=0.3) 

# 5. LAUNCH BUTTON
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

# DICTIONARY
states = {
    0: "0 - PRE-FLIGHT",
    1: "1 - RECON", 
    2: "2 - DESCENT", 
    3: "3 - ARMED", 
    4: "4 - CRASH", 
    5: "5 - RECOVERY"
}

# Tracking Variables
max_g_force = 0.0
impact_t_data = []
impact_g_data = []
impact_start_time = None
previous_state = 0
last_debug_print = time.time()
packets_received = 0

print("\n======================================")
print("[*] GCS Online. Diagnostic Mode Active.")
print(f"[*] Listening on UDP Port {UDP_PORT}...")
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
                print(f"[WARNING] No telemetry received from capsule. Ensure you are connected to '{CAPSULE_IP}' WiFi.")
            else:
                print(f"[STATUS] Receiving Telemetry. Packet rate healthy.")
                packets_received = 0
            last_debug_print = current_time
            
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
            thermal_flat = unpacked[8:]
            
            # --- UI UPDATES ---
            thermal_matrix = np.array(thermal_flat).reshape((8, 8))
            im.set_data(thermal_matrix)
            
            max_temp_val = np.max(thermal_matrix)
            txt_maxtemp.set_text(f"MAX: {max_temp_val:.1f} C")
            if max_temp_val > 30.0:
                txt_maxtemp.set_color('red')
            else:
                txt_maxtemp.set_color('yellow')
            
            crosshair.set_data([roll], [pitch])
            if abs(pitch) > 35 or abs(roll) > 35:
                crosshair.set_color('red')
            else:
                crosshair.set_color('lime')
            
            state_str = states.get(state_id, "ERR")
            txt_state.set_text(f"STATE: {state_str}")
            
            alt_color = 'yellow' if altitude > 50.0 else 'red'
            txt_alt.set_text(f"ALT:   {altitude:.1f} cm")
            txt_alt.set_color(alt_color)
            
            txt_time.set_text(f"T+ {timestamp_ms/1000.0:.1f}s")
            
            # --- DYNAMIC IMPACT DATA COLLECTION (STATE 4) ---
            if state_id == 4:
                if impact_start_time is None:
                    impact_start_time = timestamp_ms / 1000.0
                    
                rel_t = (timestamp_ms / 1000.0) - impact_start_time
                
                # PHYSICS FIX: Extract Absolute Magnitude, subtract 1G baseline to get Dynamic Force
                total_g = np.sqrt(ax**2 + ay**2 + az**2) / 9.81
                dynamic_g = abs(total_g - 1.0) 
                
                impact_t_data.append(rel_t)
                impact_g_data.append(dynamic_g)
                line_gforce.set_data(impact_t_data, impact_g_data)
                
                if dynamic_g > ax_graph.get_ylim()[1]:
                    ax_graph.set_ylim(0, dynamic_g + 2.0)
            
            # --- POST-CRASH CDSI ANALYSIS (TRANSITION 4 -> 5) ---
            if previous_state == 4 and state_id >= 5:
                if len(impact_t_data) > 1:
                    t_arr = np.array(impact_t_data)
                    g_arr = np.array(impact_g_data) # This array is now purely Dynamic G
                    
                    peak_g = np.max(g_arr)
                    
                    # Threshold lowered to 1.5 Dynamic Gs for crash bounds
                    over_thresh = np.where(g_arr > 1.5)[0]
                    if len(over_thresh) > 0:
                        impact_duration = t_arr[over_thresh[-1]] - t_arr[over_thresh[0]]
                    else:
                        impact_duration = 0.0
                        
                    jerk = np.gradient(g_arr, t_arr)
                    peak_jerk = np.max(np.abs(jerk))
                    
                    energy_proxy = np.trapz(g_arr**2, t_arr)
                    
                    G_safe, J_safe, E_safe = 3.0, 50.0, 10.0
                    S_new = 0.5 * (peak_g / G_safe) + 0.3 * (peak_jerk / J_safe) + 0.2 * (energy_proxy / E_safe)
                    
                    peaks = np.where((g_arr[1:-1] > g_arr[:-2]) & (g_arr[1:-1] > g_arr[2:]))[0] + 1
                    valid_peaks = [p for p in peaks if g_arr[p] > 0.5] # 0.5 Dynamic G bounce threshold
                    
                    damping_ratio = None
                    if len(valid_peaks) >= 2:
                        # PHYSICS FIX: g_arr is now resting at 0, no need to subtract 1.0 anymore
                        x1 = g_arr[valid_peaks[0]]
                        x2 = g_arr[valid_peaks[1]]
                        if x1 > x2 > 0:
                            delta = np.log(x1 / x2)
                            damping_ratio = delta / np.sqrt(4 * np.pi**2 + delta**2)
                            
                    if S_new < 1.0:
                        classification = "SOFT"
                        c_color = 'lime'
                    elif S_new < 1.8:
                        classification = "HARD"
                        c_color = 'yellow'
                    else:
                        classification = "CATASTROPHIC"
                        c_color = 'red'
                        
                    txt_gforce.set_color('white')
                    txt_gforce.set_text(f"PEAK G: {peak_g:.2f}")
                    txt_cdsi.set_text(f"CDSI:   {S_new:.3f}")
                    txt_cdsi.set_color('white')
                    txt_class.set_text(f"CLASS:  {classification}")
                    txt_class.set_color(c_color)
                    line_gforce.set_color(c_color)
                    
                    print("\n" + "="*35)
                    print(">>> POST-CRASH CDSI ANALYSIS COMPLETE <<<")
                    print(f"Peak Dynamic G:   {peak_g:.2f} G")
                    print(f"Impact Duration:  {impact_duration:.3f} s")
                    print(f"Peak Jerk:        {peak_jerk:.2f} G/s")
                    print(f"Energy Proxy:     {energy_proxy:.2f}")
                    print(f"Composite Index:  {S_new:.3f} (S_new)")
                    if damping_ratio:
                        print(f"Damping Ratio:    {damping_ratio:.3f} (ζ)")
                    print(f"Classification:   {classification}")
                    print("="*35 + "\n")

            # --- RESET FOR NEXT LAUNCH ---
            if state_id < 3 and previous_state >= 4:
                impact_t_data.clear()
                impact_g_data.clear()
                impact_start_time = None
                line_gforce.set_data([], [])
                line_gforce.set_color('cyan')
                ax_graph.set_ylim(0, 10.0)
                txt_gforce.set_color('gray')
                txt_gforce.set_text("PEAK G: 0.00")
                txt_cdsi.set_color('gray')
                txt_cdsi.set_text("CDSI:   0.000")
                txt_class.set_color('gray')
                txt_class.set_text("CLASS:  N/A")
                
            previous_state = state_id
            
        fig.canvas.flush_events()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("\n[*] GCS Shutting Down.")
    sock.close()
    plt.close()