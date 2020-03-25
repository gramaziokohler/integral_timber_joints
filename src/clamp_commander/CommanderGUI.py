import tkinter as tk
import tkinter.font as tkFont
from enum import Enum
from queue import Queue
from tkinter import ttk
from types import SimpleNamespace
from ClampModel import ClampModel

from serial.tools import list_ports

class UiCommand(Enum):
    SERIAL_CONNECT = 1
    CMD_GOTO = 2
    CMD_STOP = 3
    CMD_HOME = 4
    CMD_VELO = 5
# class ClampCommanderApp(tk.Tk):
#     def __init__(self):
#         tk.Tk.__init__(self)

#         # Initialize models behind
#         self.commander = SerialCommanderTokyo()

#         # Create UI
#         self.create_ui_connect()
#         self.create_Ui_status()
#         self.create_ui_control()
#         self.create_ui_log()
#         # self.remaining = 0
#         # self.countdown(10)
def create_commander_gui(root, q:Queue, clamps):
    ui_handles = {}
    ui_handles['connect'] = create_ui_connect(root, q)
    ui_handles['status'] = create_ui_status(root, q, clamps)
    ui_handles['control'] = create_ui_control(root, q)
    return ui_handles
    
def create_ui_connect(root, q:Queue):
    ui_handles = {}
    # Title and frame
    title = tk.Label(root, text = "Connection")
    title.pack(anchor = tk.NW, expand= 0, side = tk.TOP, padx = 3, pady = 3)
    frame = ttk.Frame(root, borderwidth = 2, relief = 'solid')
    frame.pack(fill = tk.BOTH, expand = 0, side = tk.TOP, padx = 6, pady = 3)

    # Label
    label_1 = tk.Label(frame, text = "Serial Port for USB Radio")
    label_1.pack(side = tk.LEFT)

    # Combo box
    def list_serial_ports():    
        return list_ports.comports()
    def cb_update_serial_ports():
        serial_cb["values"] = list_serial_ports()
    serial_cb = ttk.Combobox(frame, width = 40, values=[], postcommand=cb_update_serial_ports)
    serial_cb.pack(side = tk.LEFT, padx = 10)
    ui_handles['serial_cb'] = serial_cb

    # Button
    def on_connect_button_click(event=None):
        cb_value = serial_cb.get()
        ports = list_serial_ports()
        for port in ports:
            if (cb_value == port.__str__()):
                print ("UI: Selected Port: %s" % port[0])
                q.put(SimpleNamespace(type=UiCommand.SERIAL_CONNECT, port=port[0]))
                break
    button = tk.Button(frame, text = "Connect / Reconnect", command = on_connect_button_click)
    button.pack(side = tk.LEFT)

    return ui_handles


def create_ui_status(root, q:Queue, clamps):
    ui_handles = {}

    # Title and frame
    title = tk.Label(root, text = "Status")
    title.pack(anchor = tk.NW, expand= 0, side = tk.TOP, padx = 3, pady = 3)
    frame = ttk.Frame(root, borderwidth = 2, relief = 'solid')
    frame.pack(fill = tk.BOTH, expand = 0, side = tk.TOP, padx = 6, pady = 3)

    # Create one row of status per clamp
    for clamp in clamps:
        ui_handles[clamp.receiver_address] = create_one_ui_status(frame, q, clamp)

    return ui_handles

def create_one_ui_status(root, q:Queue, clamp:ClampModel):
    ui_handles = {}
    
    # Within that one row, We pack left with fixed width
    frame = ttk.Frame(root, borderwidth = 2, relief = 'solid')
    frame.pack(fill = tk.BOTH, expand = 0, side = tk.TOP, padx = 6, pady = 3)

    font_key = tkFont.Font(family="Lucida Grande", size=10)
    font_value = tkFont.Font(family="Lucida Grande", size=20)

    tk.Label(frame, text = "com?", font = font_key, anchor = tk.SE).pack(side = tk.LEFT , fill=tk.Y)
    ui_handles['checkbox'] = tk.BooleanVar(value = True)
    tk.Checkbutton(frame, variable=ui_handles['checkbox']).pack(side = tk.LEFT, padx=10)
    
    def create_label_pair(label_text, handle_name):
        tk.Label(frame, text = label_text, font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y)
        ui_handles[handle_name] = tk.StringVar()
        ui_handles[handle_name].set("<?>")
        tk.Label(frame, textvariable = ui_handles[handle_name], font = font_value).pack(side = tk.LEFT, fill=tk.Y, padx=5)

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

def create_ui_control(root, q:Queue):
    ui_handles = {}

    font_key = tkFont.Font(family="Lucida Grande", size=10)
    font_value = tkFont.Font(family="Lucida Grande", size=20)

    # Title and frame
    title = tk.Label(root, text = "Control")
    title.pack(anchor = tk.NW, expand= 0, side = tk.TOP, padx = 3, pady = 3)
    frame = ttk.Frame(root, borderwidth = 2, relief = 'solid')
    frame.pack(fill = tk.BOTH, expand = 0, side = tk.TOP, padx = 6, pady = 3)

    # Buttons
    def on_goto_button_click(position):
        print("UI: Button: Go to Position %s" % position)
        q.put(SimpleNamespace(type = UiCommand.CMD_GOTO, position = position))

    tk.Label(frame, text = "Go to Position: ", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y)
    tk.Button(frame, text = "95mm", command = lambda: on_goto_button_click(95)).pack(side = tk.LEFT)
    tk.Button(frame, text = "100mm", command = lambda: on_goto_button_click(100)).pack(side = tk.LEFT)
    tk.Button(frame, text = "110mm", command = lambda: on_goto_button_click(110)).pack(side = tk.LEFT)
    tk.Button(frame, text = "120mm", command = lambda: on_goto_button_click(120)).pack(side = tk.LEFT)
    tk.Button(frame, text = "140mm", command = lambda: on_goto_button_click(140)).pack(side = tk.LEFT)
    tk.Button(frame, text = "200mm", command = lambda: on_goto_button_click(200)).pack(side = tk.LEFT)
    tk.Button(frame, text = "220mm", command = lambda: on_goto_button_click(220)).pack(side = tk.LEFT)
    
    def on_velo_button_click(velocity):
        print("UI: Button: Set Velocity %s" % velocity)
        q.put(SimpleNamespace(type = UiCommand.CMD_VELO, velocity = velocity))

    tk.Label(frame, text = "Set Velocity: ", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y, padx = 10)
    tk.Button(frame, text = "1mm/s", command = lambda: on_velo_button_click(1)).pack(side = tk.LEFT)
    tk.Button(frame, text = "2mm/s", command = lambda: on_velo_button_click(2)).pack(side = tk.LEFT)
    tk.Button(frame, text = "3mm/s", command = lambda: on_velo_button_click(3)).pack(side = tk.LEFT)
    tk.Button(frame, text = "4mm/s", command = lambda: on_velo_button_click(4)).pack(side = tk.LEFT)
    tk.Button(frame, text = "5mm/s", command = lambda: on_velo_button_click(5)).pack(side = tk.LEFT)


    def on_stop_button_click():
        print("UI: Button: STOP")
        q.put(SimpleNamespace(type = UiCommand.CMD_STOP))

    tk.Label(frame, text = "Stop: ", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y, padx = 10)
    tk.Button(frame, text = "Stop Now", command = on_stop_button_click).pack(side = tk.LEFT)
    
    def on_home_button_click():
        print("UI: Button: HOME")
        q.put(SimpleNamespace(type = UiCommand.CMD_HOME))

    tk.Label(frame, text = "Home: ", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y, padx = 10)
    tk.Button(frame, text = "Home Now", command = on_home_button_click).pack(side = tk.LEFT)

    return ui_handles

def create_ui_log(root):
    pass



    # def countdown(self, remaining = None):
    #     if remaining is not None:
    #         self.remaining = remaining

    #     if self.remaining <= 0:
    #         self.label.configure(text="time's up!")
    #     else:
    #         self.label.configure(text="%d" % self.remaining)
    #         self.remaining = self.remaining - 1
    #         self.after(1000, self.countdown)

