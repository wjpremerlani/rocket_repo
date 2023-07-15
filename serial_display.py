import serial
import matplotlib.pyplot as plt
import pandas as pd

from matplotlib.animation import FuncAnimation

# Use the dark background style.
plt.style.use('dark_background')

# Open the serial port with a baud rate of 19200.
ser = serial.Serial('/dev/ttyUSB0', 19200)

def convert_time(time_str):
    # Split the time string into minutes, seconds, and milliseconds.
    minutes, seconds = time_str.split(':')
    total_seconds = float(minutes) * 60 + float(seconds)
    return total_seconds

def read_data():
    while True:
        # Read a line from the serial port.
        line = ser.readline().decode('ascii', errors='ignore').strip()  # Use ASCII for decoding
        print(f"Received line: {line}")  # Print the received line for debugging
        # Split the line into fields.
        fields = line.split(',')
        try:
            # Convert the time field to seconds.
            fields[0] = convert_time(fields[0])
            # Convert the rest of the fields to the appropriate types.
            data = [float(x) for x in fields]
            yield data
        except Exception:
            pass

# Initialize the data frame
full_df = pd.DataFrame(columns=['time', 'cntlModeYwPtch', 'cntlModeRoll', 'accelOn', 'launchCount', 'launched', 'tilt_count', 'apogee', 'rollAngle', 'rollDeviation', 'vertX', 'vertY', 'vertZ', 'accX', 'accY', 'accZ', 'gyroX', 'gyroY', 'gyroZ', 'yawFbVert', 'pitchFbVert', 'rollFbVert', 'yawFbHoriz', 'pitchFbHoriz', 'rollFbHoriz', 'out1', 'out2', 'out3', 'out4', 'out5', 'out6', 'out7', 'out8'])

# Set up the figure for plotting.
fig, ((ax_roll, ax_gyro), (ax_accel, ax_vert), (ax_fb_vert, ax_fb_horiz), (ax_flags, ax_out), (ax_counts, ax0)) = plt.subplots(5, 2, sharex=True)  # Create 8 subplots arranged in a 4x2 grid


# This function is called for each frame of the animation.
def update(frame):
    global full_df
    # Add the new data to the data frame.
    full_df.loc[len(full_df)] = frame

    # Remove data that is older than 300 seconds.
    partial_df = full_df[full_df['time'] >= (full_df['time'].iloc[-1] - 300)]

    # Clear the axes for the new plots.
    ax_roll.clear()
    ax_gyro.clear()
    ax_accel.clear()
    ax_vert.clear()
    ax_fb_vert.clear()
    ax_fb_horiz.clear()
    ax_flags.clear()
    ax_out.clear()
    ax_counts.clear()
    ax0.clear()

    # Plot the new data.
    ax_roll.plot(partial_df['time'], partial_df['rollAngle'], label='Roll Angle')
    ax_roll.plot(partial_df['time'], partial_df['rollDeviation'], label='Roll Deviation')
    ax_gyro.plot(partial_df['time'], partial_df['gyroX'], label='Gyro X')
    ax_gyro.plot(partial_df['time'], partial_df['gyroY'], label='Gyro Y')
    ax_gyro.plot(partial_df['time'], partial_df['gyroZ'], label='Gyro Z')
    ax_accel.plot(partial_df['time'], partial_df['accX'], label='Accel X')
    ax_accel.plot(partial_df['time'], partial_df['accY'], label='Accel Y')
    ax_accel.plot(partial_df['time'], partial_df['accZ'], label='Accel Z')
    ax_vert.plot(partial_df['time'], partial_df['vertX'], label='Vert X')
    ax_vert.plot(partial_df['time'], partial_df['vertY'], label='Vert Y')
    ax_vert.plot(partial_df['time'], partial_df['vertZ'], label='Vert Z')
    ax_fb_vert.plot(partial_df['time'], partial_df['yawFbVert'], label='Yaw FB Vert')
    ax_fb_vert.plot(partial_df['time'], partial_df['pitchFbVert'], label='Pitch FB Vert')
    ax_fb_vert.plot(partial_df['time'], partial_df['rollFbVert'], label='Roll FB Vert')
    ax_fb_horiz.plot(partial_df['time'], partial_df['yawFbHoriz'], label='Yaw FB Horiz')
    ax_fb_horiz.plot(partial_df['time'], partial_df['pitchFbHoriz'], label='Pitch FB Horiz')
    ax_fb_horiz.plot(partial_df['time'], partial_df['rollFbHoriz'], label='Roll FB Horiz')
    ax_flags.plot(partial_df['time'], partial_df['accelOn'], label='Accel On')
    ax_flags.plot(partial_df['time'], partial_df['launched'], label='Launched')
    ax_flags.plot(partial_df['time'], partial_df['apogee'], label='Apogee')
    ax_out.plot(partial_df['time'], partial_df['out1'], label='Out1')
    ax_out.plot(partial_df['time'], partial_df['out2'], label='Out2')
    ax_out.plot(partial_df['time'], partial_df['out3'], label='Out3')
    ax_out.plot(partial_df['time'], partial_df['out4'], label='Out4')
    ax_out.plot(partial_df['time'], partial_df['out5'], label='Out5')
    ax_out.plot(partial_df['time'], partial_df['out6'], label='Out6')
    ax_out.plot(partial_df['time'], partial_df['out7'], label='Out7')
    ax_out.plot(partial_df['time'], partial_df['out8'], label='Out8')
    ax_counts.plot(partial_df['time'], partial_df['tilt_count'], label='Tilt Count')
    ax_counts.plot(partial_df['time'], partial_df['launchCount'], label='Launch Count')
    ax0.plot(partial_df['time'], partial_df['cntlModeYwPtch'], label='Cntl Mode Yw Pitch')
    ax0.plot(partial_df['time'], partial_df['cntlModeRoll'], label='Cntl Mode Roll')

    ax_roll.set_ylabel('Roll')
    ax_gyro.set_ylabel('Gyro')
    ax_accel.set_ylabel('Accel')
    ax_vert.set_ylabel('Vert')
    ax_fb_vert.set_ylabel('FB Vert')
    ax_fb_horiz.set_ylabel('FB Horiz')
    ax_flags.set_ylabel('Flags')
    ax_out.set_ylabel('Out')
    ax_counts.set_ylabel('Counts')
    ax0.set_ylabel('Control Flags')

    # Display the legend on each subplot.
    ax_roll.legend()
    ax_gyro.legend()
    ax_accel.legend()
    ax_vert.legend()
    ax_fb_vert.legend()
    ax_fb_horiz.legend()
    ax_flags.legend()
    ax_out.legend()
    ax_counts.legend()
    ax0.legend()

# Create the animation.
ani = FuncAnimation(fig, update, frames=read_data, interval=100, blit=False)

# Show the plot.
plt.show()