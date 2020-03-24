import tkinter as tk
from enum import Enum
import queue
from threading import Thread
from tkinter import ttk
from types import SimpleNamespace

from ClampModel import ClampModel
from CommanderGUI import *
from SerialCommanderTokyo import SerialCommanderTokyo
import time
current_milli_time = lambda: int(round(time.time() * 1000))

def handle_ui_commands(guiref, commander:SerialCommanderTokyo, q):
    msg = None

    while True:
        # Process UI commands
        try:
            msg = q.get(timeout=0.1)
            
            if hasattr(msg, 'type'):
                if msg.type == UiCommand.SERIAL_CONNECT:
                    print("Ctr: Command Received to Connect to %s" % msg.port)
                    commander.connect(msg.port)

        except queue.Empty:
            pass
        
        # Read Status from Clamps
        # read = commander.port.read(commander.port.in_waiting)
        # if len(read) > 0:
        #     guiref.label.set(read.decode())

def update_status(guiref, commander:SerialCommanderTokyo, run_interval_millis = 500):
    while True:

        if ((commander.serial_port is not None) and commander.serial_port.isOpen()):
            commander.update_clamps_status(1)
            for clamp in commander.clamps.values():
                if clamp.ishomed is not None:
                    guiref['status'][clamp.receiver_address]['homed'].set("Yes" if clamp.ishomed else "No")
                if clamp.currentJawPosition is not None:
                    guiref['status'][clamp.receiver_address]['position'].set("%smm" % clamp.currentJawPosition)
                    guiref['status'][clamp.receiver_address]['error'].set("%i steps" % int(clamp.currentMotorPosition - clamp.currentMotorTarget))
                if clamp.batteryPercentage is not None :
                    guiref['status'][clamp.receiver_address]['battery'].set("%i%%"%clamp.batteryPercentage)
                if clamp._state_timestamp is not None :
                    guiref['status'][clamp.receiver_address]['last_com'].set("%dms"%(current_milli_time() - clamp._state_timestamp))
                if clamp._last_set_position is not None :
                    guiref['status'][clamp.receiver_address]['last_pos'].set("%smm"%clamp._last_set_position)
                if clamp._last_set_velocity is not None :
                    guiref['status'][clamp.receiver_address]['last_vel'].set("%smm"%clamp._last_set_velocity)

        time.sleep(run_interval_millis / 1000.0)

if __name__ == "__main__":
    # Root TK Object
    root = tk.Tk()
    root.title("Tokyo Clamps Commander")
    root.geometry("1000x400")
    # Command queue
    q = queue.Queue()
    # Create Model
    commander = SerialCommanderTokyo()
    # Get GUI Reference
    guiref = create_commander_gui(root, q, commander.clamps.values())
    
    # 
    
    # Start the background thread that processes UI commands
    t1 = Thread(target=handle_ui_commands, args=(guiref, commander, q))
    t1.daemon = True
    t1.start()

    # Start the background thread that updates clamps status automatically
    t2 = Thread(target=update_status, args=(guiref, commander, 200))
    t2.daemon = True
    t2.start()

    # Start the TK GUI Thread
    tk.mainloop()
