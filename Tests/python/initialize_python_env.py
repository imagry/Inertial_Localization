# %% Import libraries
import os
import sys
import subprocess
import pkg_resources


def initialize_environment():
    # Adjust the path to include the parent directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
    sys.path.insert(0, parent_dir)
   
    # check if the required packages are installed and install them if not
    pre_required_install_check()
    print('finished')

def install_tkinter():
    try:
        # Attempt to install tkinter via the system package manager
        if sys.platform.startswith('linux'):
            # On Ubuntu/Debian-based systems
            subprocess.check_call(['sudo', 'apt-get', 'install', '-y', 'python3-tk'])
        elif sys.platform == 'darwin':  # macOS
            subprocess.check_call(['brew', 'install', 'python-tk'])
        elif sys.platform == 'win32':
            print("Please install tkinter manually via the Python installer.")
            sys.exit(1)
        else:
            print("Unsupported OS. Please install tkinter manually.")
            sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"Failed to install tkinter: {e}")
        sys.exit(1)

try:
    import tkinter as tk
except ImportError:
    print("tkinter is not installed. Installing...")
    install_tkinter()

def pre_required_install_check():
    """
    Checks if the required packages are installed and installs them if not.
    """
    packages = ['pandas', 'matplotlib', 'numpy', 'scipy', 'folium', 'multipledispatch', 'debugpy']

    for package_name in packages:
        try:
            pkg_resources.get_distribution(package_name)
        except pkg_resources.DistributionNotFound:
            print(f"{package_name} is not installed. Installing now...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])
 
# Initialize the environment
initialize_environment()