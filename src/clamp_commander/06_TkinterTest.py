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

# Initialize multi-tasking variables.
last_status_update_time = 0

def background_thread(guiref, commander:SerialCommanderTokyo, q):
    # This is a sudo time-split multi-task with priority execution.
    # Tasks higher up in the list have higher priority.
    # Task must return True if it was executed and return False if it was not executed.
    while True:
        if handle_ui_commands(guiref, commander, q): continue
        if update_status(guiref, commander): continue

def get_checkbox_selected_clamps(guiref, commander:SerialCommanderTokyo):
    # Determine if the clamps are selected
    clamps_selected = []
    for clamp in commander.clamps.values():
        if (guiref['status'][clamp.receiver_address]['checkbox'].get()):
            clamps_selected.append(clamp)
    return clamps_selected

def handle_ui_commands(guiref, commander:SerialCommanderTokyo, q):
    try:
        msg = None
        msg = q.get(timeout=0.1)
        if hasattr(msg, 'type'):
            # Handelling SERIAL_CONNECT
            if msg.type == UiCommand.SERIAL_CONNECT:
                print("Ctr: Command Received to Connect to %s" % msg.port)
                commander.connect(msg.port)
                
            # Handelling CMD_GOTO
            if msg.type == UiCommand.CMD_GOTO:
                if not commander.is_connected:
                    print("Ctr: Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                position = msg.position
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                if len(clamps_to_communicate) > 0:
                    result = commander.send_clamps_to_jaw_position(clamps_to_communicate, msg.position, None)
                    if result:
                        print ("Ctr: Message successfully sent.")
                    else:
                        print ("Ctr: Message failed to send.")

            # Handelling CMD_STOP
            if msg.type == UiCommand.CMD_STOP:
                if not commander.is_connected:
                    print("Ctr: Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                if len(clamps_to_communicate) == 0: return True
                result = commander.stop_clamps(clamps_to_communicate)
                if result:
                    print ("Ctr: Message successfully sent.")
                else:
                    print ("Ctr: Message failed to send.")

            # Handelling CMD_HOME
            if msg.type == UiCommand.CMD_HOME:
                if not commander.is_connected:
                    print("Ctr: Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                # Need to loop through.
                for clamp in clamps_to_communicate:
                    response = commander.message_clamp(clamp,'h',3)
                    if response is not None:
                        print ("Ctr: Message to Clamp %s successfully sent." % clamp.receiver_address)
                    else:
                        print ("Ctr: Message to Clamp %s failed to send." % clamp.receiver_address)

            # Handelling CMD_VELO
            if msg.type == UiCommand.CMD_VELO:
                if not commander.is_connected:
                    print("Ctr: Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                velocity = msg.velocity
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                # Need to loop through.
                for clamp in clamps_to_communicate:
                    result = commander.set_clamp_velocity(clamp, velocity)
                    if result:
                        print ("Ctr: Message to Clamp %s successfully sent." % clamp.receiver_address)
                    else:
                        print ("Ctr: Message to Clamp %s failed to send." % clamp.receiver_address)


            # Return True if q.get() didn't return empty
            return True
    except queue.Empty:
        return False

        # Update Status

def update_status(guiref, commander:SerialCommanderTokyo):
    global last_status_update_time
    if ((commander.serial_port is None) or (not commander.serial_port.isOpen())):
        return False
    if (last_status_update_time + commander.status_update_interval_ms <= current_milli_time()):
        last_status_update_time = current_milli_time()
        # Request status from commander
        commander.update_clamps_status(1)
        # Set UI variables
        for clamp in commander.clamps.values():
            if clamp.ishomed is not None:
                guiref['status'][clamp.receiver_address]['home'].set("Yes" if clamp.ishomed else "No")
            if clamp.isMotorRunning is not None:
                if clamp.isMotorRunning:
                    if clamp.isDirectionExtend:
                        guiref['status'][clamp.receiver_address]['motion'].set("Extending")
                    else:
                        guiref['status'][clamp.receiver_address]['motion'].set("Retracting")
                else:
                    guiref['status'][clamp.receiver_address]['motion'].set("Stopped")
            if clamp.currentJawPosition is not None:
                guiref['status'][clamp.receiver_address]['position'].set("%04.1fmm" % clamp.currentJawPosition)
                guiref['status'][clamp.receiver_address]['error'].set("%i steps" % int(clamp._raw_currentPosition - clamp._raw_currentTarget))
            if clamp.currentMotorPowerPercentage is not None :
                guiref['status'][clamp.receiver_address]['power'].set("%i%%"%clamp.currentMotorPowerPercentage)
            if clamp.batteryPercentage is not None :
                guiref['status'][clamp.receiver_address]['battery'].set("%i%%"%clamp.batteryPercentage)
            if clamp._state_timestamp is not None :
                guiref['status'][clamp.receiver_address]['last_com'].set("%dms"%(current_milli_time() - clamp._state_timestamp))
            if clamp._last_set_position is not None :
                guiref['status'][clamp.receiver_address]['last_pos'].set("%04.1fmm"%clamp._last_set_position)
            if clamp._last_set_velocity is not None :
                guiref['status'][clamp.receiver_address]['last_vel'].set("%3.1fmm/s"%clamp._last_set_velocity)
        return True
    else:
        return False

if __name__ == "__main__":
    # Root TK Object
    root = tk.Tk()
    root.title("Tokyo Clamps Commander")
    root.geometry("1300x400")
    # Command queue
    q = queue.Queue()
    # Create Model
    commander = SerialCommanderTokyo()
    commander.status_update_interval_ms = 250
    # Get GUI Reference
    guiref = create_commander_gui(root, q, commander.clamps.values())
    
    # Start the background thread that processes UI commands
    t1 = Thread(target=background_thread, args=(guiref, commander, q))
    t1.daemon = True
    t1.start()

    # Start the TK GUI Thread
    tk.mainloop()
