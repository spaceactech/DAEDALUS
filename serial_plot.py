import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time

PORT = "COM6"
BAUD = 115200

ser = None

def connect():
    global ser
    while True:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=1)
            time.sleep(2)              # wait for board reset
            ser.reset_input_buffer()   # clear garbage
            print("Connected to", PORT)
            break
        except:
            print("Waiting for board...")
            time.sleep(2)

connect()

max_points = 500

time_data = deque(maxlen=max_points)
target_data = deque(maxlen=max_points)
current_data = deque(maxlen=max_points)

fig, ax = plt.subplots()

line_target, = ax.plot([], [], label="Target")
line_current, = ax.plot([], [], label="Current")

ax.set_xlabel("Time (sec)")
ax.set_ylabel("Angle")
ax.set_title("Servo PID Response")
ax.legend()

last_time = 0


def update(frame):

    global ser, last_time

    try:

        line = ser.readline().decode(errors="ignore").strip()

        if not line:
            return line_target, line_current

        values = line.split()

        if len(values) != 3:
            return line_target, line_current

        t, target, current = map(float, values)

        # detect new run
        if t < last_time:
            print("New dataset detected")
            time_data.clear()
            target_data.clear()
            current_data.clear()

        last_time = t

        time_data.append(t)
        target_data.append(target)
        current_data.append(current)

        line_target.set_data(time_data, target_data)
        line_current.set_data(time_data, current_data)

        ax.set_xlim(min(time_data), max(time_data))
        ax.set_ylim(min(target_data + current_data) - 5,
                    max(target_data + current_data) + 5)

    except serial.SerialException:
        print("Serial disconnected — reconnecting...")
        try:
            ser.close()
        except:
            pass
        connect()

    except:
        pass

    return line_target, line_current


ani = animation.FuncAnimation(fig, update, interval=50)

plt.show()