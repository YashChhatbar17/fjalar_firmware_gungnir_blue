import time as trolled
from FjalarParser import FjalarParser
import tkinter as tk
from utils.widgets import *
import sys
import argparse

def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["serial", "file"], help="input type")
    p.add_argument("path", help="device (COM3, /dev/ttyUSB0) or file path")
    args = p.parse_args(sys.argv[1:])

    fjalar = FjalarParser()

    if args.mode == "serial":
        fjalar.open_serial(args.path)
    elif args.mode == "file":
        fjalar.open_file(args.path)

    root = tk.Tk()
    padding = {"padx": 5, "pady": 5}
    data_panel = tk.Frame(root)
    control_panel = tk.Frame(root)

    altitude = TextLastValue(data_panel, "altitude: ", fjalar.data["altitude"])
    altitude.grid(row=0,column=0)
    accel = TextLastValue(data_panel, "acceleration: ", fjalar.data["az"])
    accel.grid(row=1,column=0)
    velocity = TextLastValue(data_panel, "velocity: ", fjalar.data["velocity"])
    velocity.grid(row=2,column=0)
    state = TextLastValue(data_panel, "state: ", fjalar.data["flightState"])
    state.grid(row=3,column=0)
    state = TextLastValue(data_panel, "sudo: ", fjalar.data["sudo"])
    state.grid(row=4,column=0)
    flash = FlashUsed(data_panel, fjalar)
    flash.grid(row=5,column=0)


    altitude_graph = AltitudeGraph(root, fjalar)
    velocity_graph = VelocityGraph(root, fjalar)
    accel_graph = AccelGraph(root, fjalar)
    altitude_graph.widget.grid(row=0,column=0)
    velocity_graph.widget.grid(row=1,column=0)
    accel_graph.widget.grid(row=0,column=1)


    toggle_sudo = tk.Button(control_panel, text="toggle sudo", command=fjalar.toggle_sudo)
    toggle_sudo.grid(row=0, column=0)
    clear_flash = tk.Button(control_panel, text="clear flash", command=fjalar.clear_flash)
    clear_flash.grid(row=1, column=0)
    enter_initiate = tk.Button(control_panel, text="enter initiate", command=fjalar.enter_initiate)
    enter_initiate.grid(row=2, column=0)
    enter_launch = tk.Button(control_panel, text="enter launch", command=fjalar.enter_launch)
    enter_launch.grid(row=3, column=0)

    def dump_flash():
        path = tk.filedialog.asksaveasfilename(
            title="Save flash dump",
            defaultextension=".bin",
            filetypes=[("Binary files", "*.bin"), ("All files", "*.*")]
        )
        if not path:
            return

        f = open(path, "wb")
        max_index = 0x8000000 // 8  # make it int (was / 8 -> float)
        print("max index", max_index)
        current_index = 0
        while True:
            to_read = min(64, max_index - current_index)
            if to_read == 0:
                break
            result = fjalar.read_flash(current_index, to_read)
            if result is None:
                print("failed read")
                continue

            buf = bytes(result)  # move BEFORE using it

            only_empty = True
            for v in buf:
                if v != 0xff:
                    only_empty = False
                    break
            if only_empty:
                break

            f.write(buf)
            current_index += len(buf)
        f.close()
        print("done")


    dump_flash = tk.Button(control_panel, text="dump flash", command=dump_flash)
    dump_flash.grid(row=4, column=0)
    data_panel.grid(row=0, column=2, padx=80, pady=80)
    control_panel.grid(row=1, column=2, padx=80, pady=80)

    root.mainloop()

main()
