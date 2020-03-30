import logging
import tkinter as tk
import tkinter.font as tkFont
from enum import Enum
from queue import Queue
from tkinter import ttk
from types import SimpleNamespace

from ClampModel import ClampModel
from serial.tools import list_ports

logger_ui = logging.getLogger("app.UI")


class BackgroundCommand(Enum):
    SERIAL_CONNECT = 1  # Rename these to UI_*
    CMD_GOTO = 2
    CMD_STOP = 3
    CMD_HOME = 4
    CMD_VELO = 5
    LOGGING = 6
    UI_ROS_CONNECT = 7
    ROS_VEL_GOTO_COMMAND = 8
    ROS_STOP_COMMAND = 9


def create_commander_gui(root, q: Queue, clamps):
    tk.font_key = tkFont.Font(family="Lucida Grande", size=10)
    tk.font_value = tkFont.Font(family="Lucida Console", size=20)

    ui_handles = {}
    ui_handles['connect'] = create_ui_connect(root, q)
    ui_handles['status'] = create_ui_status(root, q, clamps)
    ui_handles['control'] = create_ui_control(root, q)
    #ui_handles['logging'] = create_ui_logging(root, q)
    ui_handles['ros'] = create_ui_ros(root, q)
    return ui_handles


def create_ui_connect(root, q: Queue):
    ui_handles = {}
    # Title and frame
    title = tk.Label(root, text="Connection")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Label
    label_1 = tk.Label(frame, text="Serial Port for USB Radio")
    label_1.pack(side=tk.LEFT)

    # Combo box
    def list_serial_ports():
        return list_ports.comports()

    def cb_update_serial_ports():
        serial_cb["values"] = list_serial_ports()
    serial_cb = ttk.Combobox(frame, width=40, values=[], postcommand=cb_update_serial_ports)
    serial_cb.pack(side=tk.LEFT, padx=10)
    ui_handles['serial_cb'] = serial_cb

    # Button
    def on_connect_button_click(event=None):
        cb_value = serial_cb.get()
        ports = list_serial_ports()
        # Loop though ports to find the selected port object
        logger_ui.info("Button Pressed: Connect Serial")
        for port in ports:
            if (cb_value == port.__str__()):
                logger_ui.info("Selected Port: %s" % port[0])
                q.put(SimpleNamespace(type=BackgroundCommand.SERIAL_CONNECT, port=port[0]))
                break
    button = tk.Button(frame, text="Connect / Reconnect", command=on_connect_button_click)
    button.pack(side=tk.LEFT)

    return ui_handles


def create_ui_status(root, q: Queue, clamps):
    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="Status")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Create one row of status per clamp
    for clamp in clamps:
        ui_handles[clamp.receiver_address] = create_one_ui_status(frame, q, clamp)

    return ui_handles


def create_one_ui_status(root, q: Queue, clamp: ClampModel):
    ui_handles = {}

    # Within that one row, We pack left with fixed width
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    font_key = tkFont.Font(family="Lucida Grande", size=10)
    font_value = tkFont.Font(family="Lucida Grande", size=20)

    tk.Label(frame, text="com?", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y)
    ui_handles['checkbox'] = tk.BooleanVar(value=True)
    tk.Checkbutton(frame, variable=ui_handles['checkbox']).pack(side=tk.LEFT, padx=10)

    def create_label_pair(label_text, handle_name):
        tk.Label(frame, text=label_text, font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y)
        ui_handles[handle_name] = tk.StringVar()
        ui_handles[handle_name].set("<?>")
        tk.Label(frame, textvariable=ui_handles[handle_name], font=tk.font_value).pack(side=tk.LEFT, fill=tk.Y, padx=5)

    # Create the fields
    create_label_pair("addr", "addr")
    ui_handles["addr"].set(clamp.receiver_address)
    create_label_pair("home", "home")
    create_label_pair("motion", "motion")
    create_label_pair("pos", "position")
    create_label_pair("err", "error")
    create_label_pair("power", "power")
    create_label_pair("batt", "battery")
    create_label_pair("last_com", "last_com")
    create_label_pair("last_pos", "last_pos")
    create_label_pair("last_vel", "last_vel")
    return ui_handles


def create_ui_control(root, q: Queue):

    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="Control")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Buttons
    def on_goto_button_click(position):
        logger_ui.info("Button Pressed: Go to Position %s" % position)
        q.put(SimpleNamespace(type=BackgroundCommand.CMD_GOTO, position=position))

    tk.Label(frame, text="Go to Position: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y)
    tk.Button(frame, text="95mm", command=lambda: on_goto_button_click(95)).pack(side=tk.LEFT)
    tk.Button(frame, text="100mm", command=lambda: on_goto_button_click(100)).pack(side=tk.LEFT)
    tk.Button(frame, text="110mm", command=lambda: on_goto_button_click(110)).pack(side=tk.LEFT)
    tk.Button(frame, text="120mm", command=lambda: on_goto_button_click(120)).pack(side=tk.LEFT)
    tk.Button(frame, text="140mm", command=lambda: on_goto_button_click(140)).pack(side=tk.LEFT)
    tk.Button(frame, text="200mm", command=lambda: on_goto_button_click(200)).pack(side=tk.LEFT)
    tk.Button(frame, text="220mm", command=lambda: on_goto_button_click(220)).pack(side=tk.LEFT)

    def on_velo_button_click(velocity):
        logger_ui.info("Button Pressed: Set Velocity %s" % velocity)
        q.put(SimpleNamespace(type=BackgroundCommand.CMD_VELO, velocity=velocity))

    tk.Label(frame, text="Set Velocity: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    tk.Button(frame, text="1mm/s", command=lambda: on_velo_button_click(1)).pack(side=tk.LEFT)
    tk.Button(frame, text="2mm/s", command=lambda: on_velo_button_click(2)).pack(side=tk.LEFT)
    tk.Button(frame, text="3mm/s", command=lambda: on_velo_button_click(3)).pack(side=tk.LEFT)
    tk.Button(frame, text="4mm/s", command=lambda: on_velo_button_click(4)).pack(side=tk.LEFT)
    tk.Button(frame, text="5mm/s", command=lambda: on_velo_button_click(5)).pack(side=tk.LEFT)

    def on_stop_button_click():
        logger_ui.info("Button Pressed: STOP")
        q.put(SimpleNamespace(type=BackgroundCommand.CMD_STOP))

    tk.Label(frame, text="Stop: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    tk.Button(frame, text="Stop Now", command=on_stop_button_click).pack(side=tk.LEFT)

    def on_home_button_click():
        logger_ui.info("Button Pressed: HOME")
        q.put(SimpleNamespace(type=BackgroundCommand.CMD_HOME))

    tk.Label(frame, text="Home: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    tk.Button(frame, text="Home Now", command=on_home_button_click).pack(side=tk.LEFT)

    return ui_handles


def create_ui_logging(root, q: Queue):

    ui_handles = {}

    font_key = tkFont.Font(family="Lucida Grande", size=10)
    font_value = tkFont.Font(family="Lucida Grande", size=20)

    # Title and frame
    title = tk.Label(root, text="Logging")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Button
    def on_logging_button_click(event=None):
        logger_ui.info("Button Pressed: logging_button")
        q.put(SimpleNamespace(type=BackgroundCommand.LOGGING))

    ui_handles['logging_button_text'] = tk.StringVar(value="Start Logging")
    tk.Button(frame, textvariable=ui_handles['logging_button_text'], command=on_logging_button_click).pack(side=tk.LEFT)
    return ui_handles




def create_ui_ros(root, q: Queue):

    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text="ROS Connection")
    title.pack(anchor=tk.NW, expand=0, side=tk.TOP, padx=3, pady=3)
    frame = ttk.Frame(root, borderwidth=2, relief='solid')
    frame.pack(fill=tk.BOTH, expand=0, side=tk.TOP, padx=6, pady=3)

    # Button Handle
    def on_ros_connect_button_click(event=None):
        logger_ui.info("Button Pressed: Connect to ROS")
        ip = ros_ip_entrybox.get()
        q.put(SimpleNamespace(type=BackgroundCommand.UI_ROS_CONNECT, ip=ip))

    tk.Label(frame, text="ROS Core IP Address: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['ros_ip_entry'] = tk.StringVar(value="127.0.0.0")
    ros_ip_entrybox = tk.Entry(frame, textvariable=ui_handles['ros_ip_entry'])
    ros_ip_entrybox.pack(side=tk.LEFT)
    tk.Button(frame, text="Connect", command=on_ros_connect_button_click).pack(side=tk.LEFT)
    # Status Label
    tk.Label(frame, text="Status: ", font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)
    ui_handles['ros_status'] = tk.StringVar(value="Not Connected")
    tk.Label(frame, textvariable=ui_handles['ros_status'], font=tk.font_key, anchor=tk.SE).pack(side=tk.LEFT, fill=tk.Y, padx=10)

    return ui_handles
