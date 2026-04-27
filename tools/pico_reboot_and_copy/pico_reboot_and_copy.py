#!/usr/bin/env python3
import os
import sys
import glob
import shutil
import time
import rclpy
from std_srvs.srv import Trigger
import tkinter as tk
from tkinter import ttk, messagebox


def find_uf2_files(folder):
    """Recursively scan folder for .uf2 files"""
    return glob.glob(os.path.join(folder, "**", "*.uf2"), recursive=True)


def detect_pico_mount(timeout=10):
    """Detect Pico mount under /media/<user>"""
    media_root = f"/media/{os.environ['USER']}"
    start = time.time()
    while time.time() - start < timeout:
        if os.path.exists(media_root):
            for device_dir in os.listdir(media_root):
                device_path = os.path.join(media_root, device_dir)
                if os.path.isdir(device_path):
                    for f in os.listdir(device_path):
                        if f.lower() == "info_uf2.txt":
                            return device_path
        time.sleep(0.5)
    return None


def reboot_pico_service():
    """Call micro-ROS reboot service (fire-and-forget)"""
    rclpy.init()
    node = rclpy.create_node("pico_uf2_rebooter")
    client = node.create_client(Trigger, "reboot_to_bootloader")

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for reboot service...")

    req = Trigger.Request()
    client.call_async(req)  # fire-and-forget
    node.get_logger().info("Requested Pico reboot, waiting 2 seconds...")
    time.sleep(2)
    node.destroy_node()
    rclpy.shutdown()


def upload_uf2(selected_uf2, pico_mount):
    """Copy the selected UF2 to the Pico mount"""
    if not selected_uf2 or not pico_mount:
        messagebox.showerror("Error", "UF2 file or Pico mount not selected/found!")
        return

    try:
        shutil.copy(selected_uf2, pico_mount)
        messagebox.showinfo(
            "Success", f"Copied {os.path.basename(selected_uf2)} to Pico!"
        )
    except Exception as e:
        messagebox.showerror("Error", f"Failed to copy UF2: {e}")


def build_ui(uf2_files, pico_mount):
    """Create simple Tkinter UI"""
    root = tk.Tk()
    root.title("Pico UF2 Uploader")

    tk.Label(root, text="Select UF2 file:").pack(pady=5)

    selected_file = tk.StringVar()
    combo = ttk.Combobox(
        root,
        values=[os.path.basename(f) for f in uf2_files],
        textvariable=selected_file,
    )
    combo.pack(pady=5)
    if uf2_files:
        combo.current(0)

    tk.Label(root, text=f"Pico mount detected: {pico_mount}").pack(pady=5)

    def on_upload():
        file_index = combo.current()
        if file_index < 0:
            messagebox.showerror("Error", "No UF2 file selected")
            return
        upload_uf2(uf2_files[file_index], pico_mount)

    tk.Button(root, text="Upload UF2", command=on_upload).pack(pady=10)
    root.mainloop()


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 pico_uf2_upload.py <folder_with_uf2>")
        sys.exit(1)

    folder = sys.argv[1]
    if not os.path.isdir(folder):
        print(f"Error: {folder} is not a valid directory")
        sys.exit(1)

    uf2_files = find_uf2_files(folder)
    if not uf2_files:
        print(f"No UF2 files found in {folder}")
        sys.exit(1)

    print("Calling reboot service...")
    reboot_pico_service()

    print("Detecting Pico mount...")
    pico_mount = detect_pico_mount()
    if not pico_mount:
        print("Pico mount not found under /media. Please ensure it is mounted.")
        sys.exit(1)
    print(f"Pico mounted at: {pico_mount}")

    build_ui(uf2_files, pico_mount)


if __name__ == "__main__":
    main()
