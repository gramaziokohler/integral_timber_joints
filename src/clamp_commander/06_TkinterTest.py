import tkinter as tk
from tkinter import ttk

from SerialCommanderTokyo import SerialCommanderTokyo
from ClampModel import ClampModel
import serial.tools.list_ports


class ClampCommanderApp(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)

        # Initialize models behind
        self.commander = SerialCommanderTokyo()

        # Create UI
        self.create_ui_connect()
        self.create_Ui_status()
        self.create_ui_control()
        self.create_ui_log()
        # self.remaining = 0
        # self.countdown(10)

    def create_ui_connect(self):
        # Title and frame
        title = tk.Label(self, text = "Connection")
        title.pack(anchor = tk.NW, expand= 0, side = tk.TOP, padx = 3, pady = 3)
        frame = ttk.Frame(self, borderwidth = 2, relief = 'solid')
        frame.pack(fill = tk.BOTH, expand = 0, side = tk.TOP, padx = 6, pady = 3)

        # Label
        label_1 = tk.Label(frame, text = "Serial Port for USB Radio")
        label_1.pack(side = tk.LEFT)

        # Combo box
        def list_serial_ports():    
            return serial.tools.list_ports.comports()
        def on_serial_cb_select(event=None):
            # get selection from event    
            print("event.widget:", event.widget.get())
        self.serial_cb = ttk.Combobox(frame, values=[1,2,3])
        self.serial_cb.pack(side = tk.LEFT)
        self.serial_cb.bind('<<ComboboxSelected>>', on_serial_cb_select)

        # Button
        def on_connect_button_click(event=None):
            self.commander.start()
        button = tk.Button(frame, text = "Connect / Reconnect", command = on_connect_button_click)
        button.pack(side = tk.LEFT)


    def create_Ui_status(self):
        # Title and frame
        title = tk.Label(self, text = "Status")
        title.pack(anchor = tk.NW, expand= 0, side = tk.TOP, padx = 3, pady = 3)
        frame = ttk.Frame(self, borderwidth = 2, relief = 'solid')
        frame.pack(fill = tk.BOTH, expand = 0, side = tk.TOP, padx = 6, pady = 3)

        # Create one row of status per clamp         
        checkbox_1 = tk.Checkbutton(frame)
        checkbox_1.pack(side = tk.LEFT)

    def create_ui_control(self):
        pass

    def create_ui_log(self):
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



if __name__ == "__main__":
    app = ClampCommanderApp()
    app.mainloop()