#!/bin/sh

# Exit immediately if a command exits with a non-zero status
set -e

# General SETUP for Python environment

echo "WELCOME TO THE PYTHON ENVIRONMENT SETUP FOR: LACORO RL CHALLENGE 2025"
echo
echo "Remember, first it is necessary to create a ROS 2 workspace (e.g.: warehouse_ws)."
echo "Inside the workspace, in the path: warehouse_ws/src, you must clone the environment."
echo
printf "Did you already create the warehouse_ws workspace? (y/n): "

# Read user answer
read -r answer

if [ "$answer" = "n" ]; then
    echo
    echo "Sorry, you must create your workspace first."
    echo "Please follow the instructions from the README at: https://github.com/roboticsuantof/warehouse_simulator.git"
    exit 1

elif [ "$answer" = "y" ]; then
    echo
    echo "Great! Continuing with the installation..."

    # --- System packages preparation ---
    # Update package lists
    sudo apt update
    # Install Python venv package (add -y to avoid interactive prompt)
    sudo apt install -y python3.10-venv

    # --- Create and activate virtual environment ---
    # It is recommended to run this inside your workspace
    if ! cd "$HOME/warehouse_ws"; then
        echo "Error: workspace directory '$HOME/warehouse_ws' not found."
        echo "Please create it and run this script again."
        exit 1
    fi

    # Create virtual environment named .venv
    python3 -m venv .venv

    # Activate the virtual environment
    # shellcheck disable=SC1091
    . .venv/bin/activate

    # Check which Python is being used
    which python3  # should point to ~/warehouse_ws/.venv/bin/python3

    # --- Python dependencies installation ---
    # Upgrade pip
    pip install --upgrade pip

    # Install core RL and numerical packages (this may take a few minutes)
    pip install "gymnasium[classic-control]" stable-baselines3 numpy

    # Check that gymnasium is available
    python3 -c "import gymnasium as gym; print('gymnasium version:', gym.__version__)"

    # Install additional tools
    pip install tensorboard
    pip show tensorboard
    pip install pyyaml
    pip install typeguard
    pip install tqdm rich

    # --- If we reach this point, everything was installed correctly ---
    echo
    echo "=============================================================="
    echo " Python environment installation completed SUCCESSFULLY!"
    echo " Workspace:  $HOME/warehouse_ws"
    echo " Virtualenv: $HOME/warehouse_ws/.venv"
    echo " To activate it later, run:"
    echo "   source ~/warehouse_ws/.venv/bin/activate"
    echo "=============================================================="

    # Deactivate virtual environment (optional)
    deactivate 2>/dev/null || true

    # Exit with success code
    exit 0

else
    echo
    echo "Invalid input. Script will now exit."
    exit 1
fi
