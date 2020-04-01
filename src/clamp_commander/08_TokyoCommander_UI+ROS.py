import datetime
import logging
import queue
import time
import tkinter as tk
from enum import Enum
from functools import partial
from threading import Thread
from tkinter import ttk
from types import SimpleNamespace

from ClampModel import ClampModel
from CommanderGUI import *
from RosClampCommandListener import RosClampCommandListener
from SerialCommanderTokyo import SerialCommanderTokyo


def current_milli_time(): return int(round(time.time() * 1000))

# This UI implemented a Model-View-Controller pattern.

# Model is a single SerialCommanderTokyo() object
# View is the TKInter UI that is created by CommanderGUI.create_commander_gui()
# Controller for the UI is the tk.mainloop().
# Controller for background task is the background_thread() that runs on a separate thread.
# - The background thread implemented a time-share based multi-task execution.
# - It is separated to handle_ui_commands() and update_status()


# Initialize logger for the controller
logger_ctr = logging.getLogger("app.ctr")
logger_ros = logging.getLogger("app.ros")
# Initialize multi-tasking variables.
last_status_update_time = 0


def background_thread(guiref, commander: SerialCommanderTokyo, q):
    # This is a sudo time-split multi-task with priority execution.
    # Tasks higher up in the list have higher priority.
    # Task must return True if it was executed and return False if it was not executed.
    logging.getLogger("app.bg").info("Background Thread Started")
    while True:
        if handle_background_commands(guiref, commander, q):
            continue
        if update_status(guiref, commander):
            continue
    logging.getLogger("app.bg").info("Background Thread Stopped")


def get_checkbox_selected_clamps(guiref, commander: SerialCommanderTokyo):
    # Determine if the clamps are selected
    clamps_selected = []
    for clamp in commander.clamps.values():
        if (guiref['status'][clamp.receiver_address]['checkbox'].get()):
            clamps_selected.append(clamp)
    return clamps_selected


def handle_background_commands(guiref, commander: SerialCommanderTokyo, q):

    try:
        msg = None
        msg = q.get(timeout=0.1)
        if hasattr(msg, 'type'):
            # Handelling SERIAL_CONNECT
            if msg.type == BackgroundCommand.SERIAL_CONNECT:
                logger_ctr.info("Command Received to Connect to %s" % msg.port)
                commander.connect(msg.port)

            # Handelling UI_ROS_CONNECT
            if msg.type == BackgroundCommand.UI_ROS_CONNECT:
                logger_ctr.info("Command Received to Connect to ROS at %s" % msg.ip)
                # Disconnect from previous host
                if (commander.ros_client is not None) and (commander.ros_client.is_connected):
                    try:
                        commander.ros_client.close()
                        # commander.ros_client.terminate()
                        logger_ctr.info("Previous ROS host disconnected")
                    except:
                        pass
                # Connect to new ROS host
                guiref['ros']['ros_status'].set("Connecting to ROS")
                commander.ros_client = RosClampCommandListener(msg.ip, partial(ros_command_callback, q=q))
                try:
                    commander.ros_client.run(timeout=2)  # This starts a separate thread
                    guiref['ros']['ros_status'].set("Connected to ROS")
                    logger_ctr.info("Ros Connected")
                except:
                    guiref['ros']['ros_status'].set("Not Connected")
                    pass

            # Handelling CMD_GOTO
            if msg.type == BackgroundCommand.CMD_GOTO:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True

                # Check for selected clamps
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                if len(clamps_to_communicate) == 0:
                    logger_ctr.warning("No clamp is selected for the movement command.")
                    return True

                # Instruct commander to send command
                position = msg.position
                # Beware this is not a syncronous command, because the clamp velocity might be different.
                # However, we still implement a stop_clamps() in case of messaging failure
                successes = []
                processed_clamps = []
                for clamp in clamps_to_communicate:
                    success = commander.send_clamp_to_jaw_position(clamp, position)
                    successes.append(success)
                    processed_clamps.append(clamp)
                    if not success:
                        commander.stop_clamps(processed_clamps)
                logger_ctr.info("Movement command executed. position = %smm, clamps = %s, result = %s" % (position, clamps_to_communicate, successes))

            # Handelling CMD_STOP
            if msg.type == BackgroundCommand.CMD_STOP:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                if len(clamps_to_communicate) == 0:
                    return True
                result = commander.stop_clamps(clamps_to_communicate)
                logger_ctr.info("Sending stop command to %s, result = %s" % (clamps_to_communicate, result))

            # Handelling CMD_HOME
            if msg.type == BackgroundCommand.CMD_HOME:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                results = commander.home_clamps(clamps_to_communicate)
                logger_ctr.info("Sending home command to %s, results = %s" % (clamps_to_communicate, results))

            # Handelling CMD_VELO
            if msg.type == BackgroundCommand.CMD_VELO:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Instruct commander to send command
                velocity = msg.velocity
                clamps_to_communicate = get_checkbox_selected_clamps(guiref, commander)
                results = commander.set_clamps_velocity(clamps_to_communicate, velocity)
                logger_ctr.info("Sending Velocity command (%s) to %s, results = %s" % (velocity, clamps_to_communicate, results))

            # Handelling ROS_VEL_GOTO_COMMAND
            if msg.type == BackgroundCommand.ROS_VEL_GOTO_COMMAND:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True

                # Retrieve the list of tuples
                instructions = msg.clmap_pos_velo
                if len(instructions) == 0:
                    logger_ctr.warning("ROS_VEL_GOTO_COMMAND has no instructions")
                    return True

                # Replace the clamp_id with the retrived ClampModel
                clamp_pos_velo_list = [(commander.clamps[clamp_id], position, velocity) for clamp_id, position, velocity in instructions]
                
                # Instruct commander to send command
                success = commander.sync_clamps_move(clamp_pos_velo_list)

                if success:
                    logger_ctr.info("ROS_VEL_GOTO_COMMAND Command Executed: Instructions = %s, results = Success" % (instructions))
                else:
                    clamps = [commander.clamps[clamp_id] for clamp_id, position, velocity in instructions]
                    positions = [position for clamp_id, position, velocity in instructions]
                    logger_ctr.warning("ROS Command Fail: send_clamp_to_jaw_position(%s,%s) Fail" % (clamps, positions))

            # Handelling ROS_STOP_COMMAND
            if msg.type == BackgroundCommand.ROS_STOP_COMMAND:
                if not commander.is_connected:
                    logger_ctr.warning("Connect to Serial Radio first")
                    return True
                # Get the clamp objects from commander
                clamps_to_communicate = [commander.clamps[clamp_id] for clamp_id in msg.clamps_id]
                # Instruct commander to send command
                result = commander.stop_clamps(clamps_to_communicate)
                logger_ctr.info("ROS_STOP_COMMAND Executed: Stop command to %s, result = %s" % (clamps_to_communicate, result))

            # Return True if q.get() didn't return empty
            return True
    except queue.Empty:
        return False

        # Update Status


def update_status(guiref, commander: SerialCommanderTokyo):
    global last_status_update_time
    if ((commander.serial_port is None) or (not commander.serial_port.isOpen())):
        return False
    if (last_status_update_time + commander.status_update_interval_ms <= current_milli_time()):
        last_status_update_time = current_milli_time()
        # Request status from commander
        commander.update_all_clamps_status(1)
        # Set UI variables
        for clamp in commander.clamps.values():
            if clamp.ishomed is not None:
                guiref['status'][clamp.receiver_address]['home'].set("Yes" if clamp.ishomed else "No")
                guiref['status'][clamp.receiver_address]['home_label'].config(fg = "red" if not clamp.ishomed else "black")
            # Clamp Running Label
            if clamp.isMotorRunning is not None:
                if clamp.isMotorRunning:
                    if clamp.isDirectionExtend:
                        guiref['status'][clamp.receiver_address]['motion'].set("Extending")
                        guiref['status'][clamp.receiver_address]['motion_label'].config(fg = "green")
                    else:
                        guiref['status'][clamp.receiver_address]['motion'].set("Retracting")
                        guiref['status'][clamp.receiver_address]['motion_label'].config(fg = "green")
                else:
                    guiref['status'][clamp.receiver_address]['motion'].set("Stopped")
                    guiref['status'][clamp.receiver_address]['motion_label'].config(fg = "black")
            # Jaw Position Label
            if clamp.currentJawPosition is not None:
                guiref['status'][clamp.receiver_address]['position'].set("%04.1fmm" % clamp.currentJawPosition)
            # Step Error with orange Label > abs(100 steps)
            if clamp._raw_currentPosition is not None:    
                guiref['status'][clamp.receiver_address]['error'].set("%3i steps" % int(clamp._raw_currentPosition - clamp._raw_currentTarget))
                guiref['status'][clamp.receiver_address]['power_label'].config(fg = "orange" if abs(clamp._raw_currentPosition - clamp._raw_currentTarget) > 100 else "black")
            # Motor Power percentage with Orange Label > 90%
            if clamp.currentMotorPowerPercentage is not None:
                guiref['status'][clamp.receiver_address]['power'].set("%3i%%" % clamp.currentMotorPowerPercentage)
                guiref['status'][clamp.receiver_address]['power_label'].config(fg = "orange" if abs(clamp.currentMotorPowerPercentage) > 90 else "black")

            # Battery percentage with Red Label < 10%
            if clamp.batteryPercentage is not None:
                guiref['status'][clamp.receiver_address]['battery'].set("%2i%%" % clamp.batteryPercentage)
                guiref['status'][clamp.receiver_address]['battery_label'].config(fg = "red" if clamp.batteryPercentage < 10 else "black")

            # Clamp Last Communicate Time with Read Label > 500ms 
            if clamp._state_timestamp is not None:
                guiref['status'][clamp.receiver_address]['last_com'].set("%2dms" % (current_milli_time() - clamp._state_timestamp))
                guiref['status'][clamp.receiver_address]['last_com_label'].config(fg = "red" if (current_milli_time() - clamp._state_timestamp) > 500 else "black")
            if clamp._last_set_position is not None:
                guiref['status'][clamp.receiver_address]['last_pos'].set("%04.1fmm" % clamp._last_set_position)
            if clamp._last_set_velocity is not None:
                guiref['status'][clamp.receiver_address]['last_vel'].set("%3.1fmm/s" % clamp._last_set_velocity)
        return True
    else:
        return False


def initialize_logging(filename: str):
    # Logging Setup
    logger = logging.getLogger("app")
    logger.setLevel(logging.DEBUG)
    # create formatter and add it to the handlers
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # create file handler which logs even debug messages
    log_file_handler = logging.FileHandler(filename)
    log_file_handler.setLevel(logging.DEBUG)
    log_file_handler.setFormatter(formatter)
    # create console handler with a higher log level
    log_console_handler = logging.StreamHandler()
    log_console_handler.setLevel(logging.INFO)
    log_console_handler.setFormatter(formatter)
    # add the handlers to logger
    logger.addHandler(log_file_handler)
    logger.addHandler(log_console_handler)
    logger.info("App Started")


def ros_command_callback(message, q=None):
    # ROS command comes from a separate thread.
    # To maintain single threaded access to the Radio / Clamp,
    # we convert the ROS Command to a BackgroundCommand and place it in background command queue

    message_type = message['instruction_type']

    logger_ros.info("Ros Message Received: %s" % message)
    if message_type == "ROS_VEL_GOTO_COMMAND":
        sequence_id = message['sequence_id']
        instructions = message['instruction_body']
        q.put(SimpleNamespace(type=BackgroundCommand.ROS_VEL_GOTO_COMMAND, clmap_pos_velo=instructions))

    if message_type == "ROS_STOP_COMMAND":
        sequence_id = message['sequence_id']
        clamps_id = message['instruction_body']
        q.put(SimpleNamespace(type=BackgroundCommand.ROS_STOP_COMMAND, clamps_id=clamps_id))


if __name__ == "__main__":

    # Initialize Logger
    initialize_logging("TokyoCommander." + datetime.date.today().strftime("%Y-%m-%d") + ".debug.log")

    # Root TK Object
    root = tk.Tk()
    root.title("Tokyo Clamps Commander")
    root.geometry("1300x400")
    # Command queue
    q = queue.Queue()
    # Create Model
    commander = SerialCommanderTokyo()
    # Get GUI Reference
    guiref = create_commander_gui(root, q, commander.clamps.values())

    # Start the background thread that processes UI commands
    t1 = Thread(target=background_thread, args=(guiref, commander, q))
    t1.daemon = True
    t1.start()

    # Override default ip
    guiref['ros']['ros_ip_entry'].set('192.168.0.115')
    # hostip = '192.168.43.141'
    # try:
    #     commander.ros_client = RosCommandListener(hostip, partial(ros_command_callback, q = q))
    #     commander.ros_client.run() # This starts a separate thread
    # except:
    #     logger_ctr.info(Initia)

    # Start the TK GUI Thread
    tk.mainloop()
