
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import numpy as np
import struct

# --- Serial setup ---
ser = serial.Serial('COM13', 115200, timeout=0.01)  # short timeout to prevent blocking

# --- Plot setup ---
win = pg.GraphicsLayoutWidget(show=True, title="Real-Time ESP32 Data", size=(1850, 1000))
win.setGeometry(0, 29, 1800, 840)

plot = win.addPlot(title="waka waka ee ee")
curveR = plot.plot(pen='w', name="Value1 (0xAA)")
curveL = plot.plot(pen='g', name="Value2 (0xAB)")
curveT = plot.plot(pen='r', name="Value3 (0xAC)")
curveJ = plot.plot(pen='y', name="Value4 (0xAD)")

plot.setYRange(-1000, 1000)
plot.showGrid(x=True, y=True)

# --- Data buffers ---
dataR = np.zeros(1000)
dataL = np.zeros(1000)
dataT = np.zeros(1000)
dataJ = np.zeros(1000)

# --- Update function ---
def update():
    global dataR, dataL, dataT, dataJ

    try:
        while ser.in_waiting > 0:
            header = ser.read(1)

            if header not in {b'\xAA', b'\xAB', b'\xAC', b'\xAD'}:
                continue  # skip noise

            raw = ser.read(4)
            if len(raw) < 4:
                break  # incomplete packet

            value = struct.unpack('<f', raw)[0]

            # Debug once to see if we are decoding correctly

            if header == b'\xAA':
                dataR = np.roll(dataR, -1)
                dataR[-1] = value
            elif header == b'\xAB':
                dataL = np.roll(dataL, -1)
                dataL[-1] = value
            elif header == b'\xAC':
                dataT = np.roll(dataT, -1)
                dataT[-1] = value
            elif header == b'\xAD':
                dataJ = np.roll(dataJ, -1)
                dataJ[-1] = value

        # Update plots
        curveR.setData(dataR)
        curveL.setData(dataL)
        curveT.setData(dataT)
        curveJ.setData(dataJ)

    except Exception as e:
        print("Error reading serial:", e)
        ser.reset_input_buffer()

# --- Timer setup ---
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)  # 50 updates per second (safe refresh rate)

# --- Run the app ---
if __name__ == '__main__':
    pg.exec()
