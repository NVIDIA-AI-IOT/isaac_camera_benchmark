#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

###### VARIABLE DEFINITION #########################

PROJECT_PATH=$(pwd)
ISAAC_SIM_VERSION="2022.2.1"  # Isaac SIM version
ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac_sim-$ISAAC_SIM_VERSION"

ISAAC_SIM_DEMO_PATH="$PROJECT_PATH/camera_benchmark.py"

#####################################################

# Define color variables using tput
BLACK=$(tput setaf 0)
RED=$(tput setaf 1)

GREEN=$(tput setaf 2)
YELLOW=$(tput setaf 3)
BLUE=$(tput setaf 4)
MAGENTA=$(tput setaf 5)
CYAN=$(tput setaf 6)
WHITE=$(tput setaf 7)
BOLD=$(tput bold)
NC=$(tput sgr0) # No Color

# Function to colorize echo messages
colorize_echo()
{
    local color="$1"
    local message="$2"
    echo "${color}${message}${NC}"
}

usage()
{
    if [ "$1" != "" ]; then
        echo "${red}$1${reset}" >&2
    fi
    
    local name=$(basename ${0})
    echo "Run NVIDIA Isaac SIM $ISAAC_SIM_VERSION and test camera benchmark"
}

main()
{
    
    # Decode all information from startup
    while [ -n "$1" ]; do
        case "$1" in
            -h|--help) # Load help
                usage
                exit 0
            ;;
            *)
                colorize_echo $RED "[ERROR] Unknown option: $1"
                exit 1
            ;;
        esac
        shift 1
    done
    
    # https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/main/docs/tutorial-isaac-sim.md
    # Run Isaac ROS with Carter in a Warehouse
    echo " - $(colorize_echo $GREEN "Start Isaac SIM ${BOLD}$ISAAC_SIM_VERSION")"
    echo "   $(colorize_echo $GREEN Path): $ISAAC_SIM_DEMO_PATH"
    # Run Isaac SIM demo
    $ISAAC_SIM_PATH/python.sh $ISAAC_SIM_DEMO_PATH

}

main $@
# EOF