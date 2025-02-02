import os
import sys
import json
import paramiko
from paramiko import SSHClient
from scp import SCPClient
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QFileDialog, QLabel, QLineEdit, QInputDialog, QMessageBox
from PyQt5.QtCore import Qt

SETTINGS_FILE = "scp_transfer_settings.json"

class SCPTransferUI(QWidget):
    def __init__(self):
        super().__init__()
        self.ssh = None
        self.scp = None
        self.files_to_transfer = []
        self.total_size = 0
        self.transferred_size = 0
        self.remote_directory = '/home/astro/ros2_ws'  # Default remote directory
        self.initUI()  # Initialize UI first
        self.load_settings()  # Then load settings

    def initUI(self):
        self.setWindowTitle("SCP File Transfer")
        self.setGeometry(100, 100, 400, 250)

        layout = QVBoxLayout()

        self.ip_label = QLabel("Enter target IP:")
        layout.addWidget(self.ip_label)

        self.ip_input = QLineEdit()
        layout.addWidget(self.ip_input)

        self.label = QLabel("Select folder to transfer:")
        layout.addWidget(self.label)

        self.select_button = QPushButton("Select Folder")
        self.select_button.clicked.connect(self.select_folders)
        layout.addWidget(self.select_button)

        self.remote_dir_label = QLabel(f"Remote Directory: {self.remote_directory}")
        layout.addWidget(self.remote_dir_label)

        self.select_remote_dir_button = QPushButton("Select Remote Directory")
        self.select_remote_dir_button.clicked.connect(self.select_remote_directory)
        layout.addWidget(self.select_remote_dir_button)

        self.transfer_button = QPushButton("Start Transfer")
        self.transfer_button.clicked.connect(self.start_transfer)
        layout.addWidget(self.transfer_button)

        self.setLayout(layout)

    def load_settings(self):
        """Load the last used settings from the settings file."""
        if os.path.exists(SETTINGS_FILE):
            with open(SETTINGS_FILE, "r") as f:
                settings = json.load(f)
                self.ip_input.setText(settings.get("last_ip", ""))
                self.files_to_transfer = settings.get("last_folder", [])
                self.remote_directory = settings.get("last_remote_directory", '/home/astro/ros2_ws')
                if self.files_to_transfer:
                    self.label.setText(f"Selected: {self.files_to_transfer[0]}")

    def save_settings(self):
        """Save the current settings to the settings file."""
        settings = {
            "last_ip": self.ip_input.text().strip(),
            "last_folder": self.files_to_transfer[:1],  # Save only the latest folder
            "last_remote_directory": self.remote_directory
        }
        with open(SETTINGS_FILE, "w") as f:
            json.dump(settings, f)

    def select_folders(self):
        """Allow user to select a folder and update settings."""
        folder = QFileDialog.getExistingDirectory(self, "Select Folder", os.getcwd(), QFileDialog.ShowDirsOnly)
        if folder:
            self.files_to_transfer = [folder]  # Store only the latest selected folder
            self.label.setText(f"Selected: {folder}")
            self.save_settings()

    def select_remote_directory(self):
        """Allow user to select a remote directory and update settings."""
        remote_dir, ok = QInputDialog.getText(self, "Remote Directory", "Enter Remote Directory:", text=self.remote_directory)
        if ok and remote_dir:
            self.remote_directory = remote_dir
            self.remote_dir_label.setText(f"Remote Directory: {self.remote_directory}")
            self.save_settings()

    def calculate_total_size(self):
        """Calculate total size of files to be transferred."""
        self.total_size = 0
        for folder in self.files_to_transfer:
            for root, _, files in os.walk(folder):
                for file in files:
                    self.total_size += os.path.getsize(os.path.join(root, file))
        self.transferred_size = 0  # Reset transferred size

    def start_transfer(self):
        if not self.files_to_transfer:
            self.label.setText("No folder selected!")
            return

        target_ip = self.ip_input.text().strip()
        if not target_ip:
            self.label.setText("Enter a valid IP address!")
            return

        # Reset the label to indicate the transfer is starting
        self.label.setText("Transferring...")
        QApplication.processEvents()  # Force the UI to update

        # Connect to the remote host via SSH
        self.ssh = SSHClient()
        self.ssh.load_system_host_keys()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(hostname=target_ip, port=22, username='astro', password='astro123')

        self.scp = SCPClient(self.ssh.get_transport(), progress=self.progress_callback)

        self.calculate_total_size()

        transferred_folders = []

        # Loop through each folder and transfer
        for folder in self.files_to_transfer:
            self.scp.put(folder, recursive=True, remote_path=self.remote_directory)
            transferred_folders.append(folder)
            QApplication.processEvents()  # Ensure UI updates during the transfer

        self.scp.close()
        self.label.setText("Transfer complete!")

        # Show a pop-up message after transfer is complete
        self.show_popup(transferred_folders)

        self.save_settings()

    def progress_callback(self, filename, size, sent):
        """Progress callback function that updates progress."""
        self.transferred_size += sent
        # Ensure the transferred size does not exceed total size (due to rounding errors)
        if self.transferred_size > self.total_size:
            self.transferred_size = self.total_size

    def show_popup(self, transferred_folders):
        """Show a pop-up message after the transfer is complete."""
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle("Transfer Complete")
        msg.setText(f"Transfer complete! Transferred: {', '.join(transferred_folders)} to {self.remote_directory}")

        # Set the custom light green background for the pop-up
        msg.setStyleSheet("QMessageBox { background-color: lightgreen; }")

        msg.exec_()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SCPTransferUI()
    window.show()
    sys.exit(app.exec_())
