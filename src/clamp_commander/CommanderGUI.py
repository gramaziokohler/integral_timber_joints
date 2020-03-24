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
    
    tk.Label(frame, text = "addr", font = font_key, anchor = tk.SE).pack(side = tk.LEFT , fill=tk.Y)
    tk.Label(frame, text = clamp.receiver_address, font = font_value, anchor = tk.SW).pack(side = tk.LEFT, fill=tk.Y, padx=10)

    tk.Label(frame, text = "home", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y)
    ui_handles['homed'] = tk.StringVar()
    ui_handles['homed'].set("<?>")
    tk.Label(frame, textvariable = ui_handles['homed'], font = font_value).pack(side = tk.LEFT, fill=tk.Y, padx=10)
    
    tk.Label(frame, text = "pos", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y)
    ui_handles['position'] = tk.StringVar()
    ui_handles['position'].set("<?>")
    tk.Label(frame, textvariable = ui_handles['position'], font = font_value).pack(side = tk.LEFT, fill=tk.Y)

    tk.Label(frame, text = "err", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y)
    ui_handles['error'] = tk.StringVar()
    ui_handles['error'].set("<?>")
    tk.Label(frame, textvariable = ui_handles['error'], font = font_value).pack(side = tk.LEFT, fill=tk.Y)

    tk.Label(frame, text = "batt", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y)
    ui_handles['battery'] = tk.StringVar()
    ui_handles['battery'].set("<?>")
    tk.Label(frame, textvariable = ui_handles['battery'], font = font_value).pack(side = tk.LEFT, fill=tk.Y)

    tk.Label(frame, text = "last_com", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y)
    ui_handles['last_com'] = tk.StringVar()
    ui_handles['last_com'].set("<?>")
    tk.Label(frame, textvariable = ui_handles['last_com'], font = font_value).pack(side = tk.LEFT, fill=tk.Y)

    tk.Label(frame, text = "last_pos", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y)
    ui_handles['last_pos'] = tk.StringVar()
    ui_handles['last_pos'].set("<?>")
    tk.Label(frame, textvariable = ui_handles['last_pos'], font = font_value).pack(side = tk.LEFT, fill=tk.Y)

    tk.Label(frame, text = "last_vel", font = font_key, anchor = tk.SE).pack(side = tk.LEFT, fill=tk.Y)
    ui_handles['last_vel'] = tk.StringVar()
    ui_handles['last_vel'].set("<?>")
    tk.Label(frame, textvariable = ui_handles['last_vel'], font = font_value).pack(side = tk.LEFT, fill=tk.Y)


    return ui_handles

def create_ui_control(root):
    pass

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

