import tkinter as tk
import tkinter.ttk as ttk
import serial.tools.list_ports

# --- functions ---

def serial_ports():    
    return [1,2,3]
    return serial.tools.list_ports.comports()

def on_select(event=None):

    # get selection from event    
    print("event.widget:", event.widget.get())

    # or get selection directly from combobox
    print("comboboxes: ", cb.get())

# --- main ---

root = tk.Tk()

cb = ttk.Combobox(root, values=serial_ports())
cb.pack()

# assign function to combobox
cb.bind('<<ComboboxSelected>>', on_select)

root.mainloop()