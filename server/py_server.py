import asyncio
import websockets
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
import numpy as np
import struct
from qasync import QEventLoop
from PyQt6.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout,
    QPushButton, QLineEdit
)
import sys

# ------------------------
# FORCE SERVER IP + PORT
# ------------------------
HOST = "172.18.173.234"   # fixed IP
PORT = 8765               # fixed port

# Global: store client connection
connected_client = None

# ------------------------
# Plot setup
# ------------------------
win = pg.GraphicsLayoutWidget(show=False, title="Real-Time ESP32 Data", size=(1850, 1000))
win.setGeometry(0, 29, 1800, 840)

plot = win.addPlot(title="waka waka ee ee")
curveR = plot.plot(pen='w')
curveL = plot.plot(pen='g')
curveT = plot.plot(pen='r')
curveJ = plot.plot(pen='y')

plot.setYRange(-1000, 1000)
plot.showGrid(x=True, y=True)

dataR = np.zeros(100)
dataL = np.zeros(100)
dataT = np.zeros(100)
dataJ = np.zeros(100)

data_buffer = bytearray()

# ------------------------
# WebSocket handler
# ------------------------
async def handle_client(websocket):
    global data_buffer, connected_client
    connected_client = websocket

    print(f"ESP32 connected: {websocket.remote_address}")

    try:
        async for message in websocket:
            data_buffer.extend(message)

    except:
        print("Client disconnected")
    finally:
        connected_client = None


# ------------------------
# Update plots
# ------------------------
def update():
    global dataR, dataL, dataT, dataJ, data_buffer

    try:
        while len(data_buffer) >= 5:
            header = data_buffer[0]

            if header not in (0xAA, 0xAB, 0xAC, 0xAD):
                data_buffer.pop(0)
                continue

            raw = data_buffer[1:5]
            value = struct.unpack("<f", raw)[0]

            data_buffer = data_buffer[5:]

            if header == 0xAA:
                dataR = np.roll(dataR, -1); dataR[-1] = value
            elif header == 0xAB:
                dataL = np.roll(dataL, -1); dataL[-1] = value
            elif header == 0xAC:
                dataT = np.roll(dataT, -1); dataT[-1] = value
            elif header == 0xAD:
                dataJ = np.roll(dataJ, -1); dataJ[-1] = value

        curveR.setData(dataR)
        curveL.setData(dataL)
        curveT.setData(dataT)
        curveJ.setData(dataJ)

    except Exception as e:
        print("Process error:", e)
        data_buffer.clear()

# ------------------------
# Start server
# ------------------------
async def start_server():
    server = await websockets.serve(handle_client, HOST, PORT)
    print(f"WebSocket running at ws://{HOST}:{PORT}")
    await server.wait_closed()


# ------------------------
# GUI: Start/Stop + Send message
# ------------------------
async def send_to_client(msg: str):
    if connected_client:
        try:
            await connected_client.send(msg)
        except:
            print("Failed to send, client disconnected")


def on_start_button_clicked():
    if start_button.text() == "Start":
        start_button.setText("Stop")
        loop.create_task(send_to_client("CMD:START"))
    else:
        start_button.setText("Start")
        loop.create_task(send_to_client("CMD:STOP"))


def on_send_button_clicked():
    text = message_input.text().strip()
    if text != "":
        loop.create_task(send_to_client("CMD:" + text.upper()))


# ------------------------
# GUI layout structure
# ------------------------
app = QApplication(sys.argv)

main_widget = QWidget()
main_layout = QHBoxLayout(main_widget)

# Left → Plot
main_layout.addWidget(win, stretch=1)

# Right → Control Panel
control_panel = QVBoxLayout()

start_button = QPushButton("Start")
message_input = QLineEdit()
send_button = QPushButton("Send")

message_input.setPlaceholderText("Enter message...")

start_button.clicked.connect(on_start_button_clicked)
send_button.clicked.connect(on_send_button_clicked)
message_input.returnPressed.connect(on_send_button_clicked)

control_panel.addWidget(start_button)
control_panel.addWidget(message_input)
control_panel.addWidget(send_button)
control_panel.addStretch()

control_widget = QWidget()
control_widget.setLayout(control_panel)
control_widget.setFixedWidth(240)  # <<< SMALL SIDE PANEL

main_layout.addWidget(control_widget)

main_widget.show()

# ------------------------
# Qt event loop
# ------------------------
loop = QEventLoop(app)
asyncio.set_event_loop(loop)

loop.create_task(start_server())

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

with loop:
    loop.run_forever()
