import time
import sys
from pymavlink import mavutil
from pynput import keyboard

VELOCITY_XY = 2.5   # m/s
VELOCITY_Z = 1.0    # m/s
YAW_RATE = 0.5      # rad/s

keys_held = {
    'w': False, 's': False, 'a': False, 'd': False,
    'up': False, 'down': False, 'left': False, 'right': False,
    'space': False
}

stop_flag = False

def on_press(key):
    global stop_flag
    try:
        if key.char in keys_held:
            keys_held[key.char] = True
    except AttributeError:
        if key == keyboard.Key.up: keys_held['up'] = True
        elif key == keyboard.Key.down: keys_held['down'] = True
        elif key == keyboard.Key.left: keys_held['left'] = True
        elif key == keyboard.Key.right: keys_held['right'] = True
        elif key == keyboard.Key.space: keys_held['space'] = True
        elif key == keyboard.Key.esc:
            print("\n[ESC] pressed. Landing...")
            stop_flag = True

def on_release(key):
    try:
        if key.char in keys_held:
            keys_held[key.char] = False
    except AttributeError:
        if key == keyboard.Key.up: keys_held['up'] = False
        elif key == keyboard.Key.down: keys_held['down'] = False
        elif key == keyboard.Key.left: keys_held['left'] = False
        elif key == keyboard.Key.right: keys_held['right'] = False
        elif key == keyboard.Key.space: keys_held['space'] = False

def send_velocity(master, vx, vy, vz, yaw_rate):
    type_mask = 0b0000011111000111
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, yaw_rate
    )

def main():
    global stop_flag
    print("== Connecting to ArduPilot SITL (udp:127.0.0.1:14550)...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    print("== Heartbeat received!")

    print("== Setting mode to GUIDED...")
    master.set_mode('GUIDED')
    time.sleep(1)

    print("== Arming vehicle...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("== Motors armed!")

    target_alt = 3.0
    print(f"== Commanding takeoff to {target_alt} m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_alt
    )
    time.sleep(7)
    print("== Drone airborne! Keyboard controls active:\n")
    print("  W / S       : Forward / Backward")
    print("  A / D       : Left / Right")
    print("  UP / DOWN   : Climb / Descend")
    print("  LEFT / RIGHT: Yaw Left / Right")
    print("  SPACE       : Hover / Brake")
    print("  ESC         : Land and Exit\n")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while not stop_flag:
            vx, vy, vz, yaw_rate = 0.0, 0.0, 0.0, 0.0
            if not keys_held['space']:
                if keys_held['w']: vx += VELOCITY_XY
                if keys_held['s']: vx -= VELOCITY_XY
                if keys_held['d']: vy += VELOCITY_XY
                if keys_held['a']: vy -= VELOCITY_XY
                if keys_held['up']: vz -= VELOCITY_Z
                if keys_held['down']: vz += VELOCITY_Z
                if keys_held['right']: yaw_rate += YAW_RATE
                if keys_held['left']: yaw_rate -= YAW_RATE

            send_velocity(master, vx, vy, vz, yaw_rate)
            time.sleep(0.1)
    finally:
        listener.stop()
        print("== Setting mode to LAND...")
        master.set_mode('LAND')

if __name__ == '__main__':
    main()