import subprocess
import sys

def save_submodule_shas(file_path):
    # Get the list of submodules with their status
    submodules = subprocess.check_output(['git', 'submodule', 'status']).decode('utf-8').strip().split('\n')

    with open(file_path, 'w') as file:
        for submodule in submodules:
            # Each submodule line is of the form: <SHA> <path> <status>
            parts = submodule.split()
            sha = parts[0]  # First part is the SHA
            path = parts[1]  # Second part is the submodule path
            file.write(f"{sha} {path}\n")
            print(f"Saved: {path} -> SHA: {sha}")

def restore_submodules(file_path):
    with open(file_path, 'r') as file:
        for line in file:
            sha, path = line.strip().split()
            print(f"Restoring: {path} -> SHA: {sha}")
            # Checkout the correct SHA for the submodule
            subprocess.run(['git', 'submodule', 'update', '--init', '--recursive', '--checkout'], check=True)

def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py <command> [file_path]")
        sys.exit(1)

    command = sys.argv[1]
    file_path = sys.argv[2] if len(sys.argv) > 2 else 'submodule_shas.txt'

    if command == "save":
        save_submodule_shas(file_path)
    elif command == "restore":
        restore_submodules(file_path)
    else:
        print("Invalid command. Use 'save' or 'restore'.")
        sys.exit(1)

if __name__ == "__main__":
    main()
