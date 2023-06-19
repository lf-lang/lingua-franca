#!/usr/bin/env bash

# Phase 0: Preflight check
# Verify baseline dependencies

# Phase 1: Setup dev environment
# Install the software packages from APT or Homebrew
# Create a directory called pico
# Download the pico-sdk repository and submodules
# Define env variables: PICO_SDK_PATH
# On Raspberry Pi only: configure the UART for use with Raspberry Pi Pico

# Phase 2: Setting up tutorial repos
# Download pico-examples, pico-extras, pico-playground repositories, and submodules
# Build the blink and hello_world examples

# Phase 3: Recommended tools
# Download and build picotool (see Appendix B), and copy it to /usr/local/bin.
# Download and build picoprobe (see Appendix A) and OpenOCD
# Download and install Visual Studio Code and required extensions


# Exit on error
set -e

# Trying to use a non-existent variable is an error
set -u

# if printenv DEBUG >& /dev/null; then
    # Show all commands
    set -x

    env
# fi

# Number of cores when running make
JNUM=4

# Where will the output go?
if printenv TARGET_DIR; then
    echo "Using target dir from \$TARGET_DIR: ${TARGET_DIR}"
else
    TARGET_DIR="$(pwd)/pico"
    echo "Using target dir: ${TARGET_DIR}"
fi

linux() {
    # Returns true iff this is running on Linux
    uname | grep -q "^Linux$"
    return ${?}
}

raspbian() {
    # Returns true iff this is running on Raspbian or close derivative such as Raspberry Pi OS, but not necessarily on a Raspberry Pi computer
    grep -q '^NAME="Raspbian GNU/Linux"$' /etc/os-release
    return ${?}
}

debian() {
    # Returns true iff this is running on Debian
    grep -q '^NAME="Debian GNU/Linux"$' /etc/os-release
    return ${?}
}

ubuntu() {
    # Returns true iff this is running on Ubuntu
    grep -q '^NAME="Ubuntu"$' /etc/os-release
    return ${?}
}

mac() {
    # Returns true iff this is running on macOS and presumably Apple hardware
    uname | grep -q "^Darwin$"
    return ${?}
}

raspberry_pi() {
    # Returns true iff this is running on a Raspberry Pi computer, regardless of the OS
    if [ -f /proc/cpuinfo ]; then
        grep -q "^Model\s*: Raspberry Pi" /proc/cpuinfo
        return ${?}
    fi
    return 1
}

sudo_wrapper() {
    # Some platforms have different needs for invoking sudo. This wrapper encapsulates that complexity.
    # The output of this function should be a string on stdout, which will be used as a command. Example:
    #   `$(sudo_wrapper) whoami`
    # The above may equate to:
    #   `sudo -i whoami`

    if [ "${USER}" = root ]; then
        # if we're already root, don't sudo at all. Relevant to some Docker images that don't have sudo but already run as root.
        return
    fi

    # EC2 AMIs tend to have the user password unset, so you can't sudo without -i. It will cd /root, so you have to be
    # careful with relative paths in the command.
    echo sudo -i
}

phase_0() {
    # Preflight check
    # Checks the baseline dependencies. If you don't have these, this script won't work.
    echo "Entering phase 0: Preflight check"
    
    if mac; then
        echo "Running on macOS"
        if which brew >> /dev/null; then
            echo "Found brew"
            brew update
        else
            echo -e 'This script requires Homebrew, the missing package manager for macOS. See https://docs.brew.sh/Installation. For quick install, run:\n/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"'
            echo "Stopping."
            exit 1
        fi
    else
        if linux; then
            echo "Running on Linux"
        else
            echo "Platform $(uname) not recognized. Use at your own risk. Continuing as though this were Linux."
        fi

        if which apt >> /dev/null; then
            echo "Found apt"
            $(sudo_wrapper) apt update
        else
            echo 'This script requires apt, the default package manager for Debian and Debian-derived distros such as Ubuntu and Raspberry Pi OS.'
            echo "Stopping."
            exit 1
        fi
    fi
}

fail() {
    # Outputs a failure message and exits with the error code output by the previous call.
    # All args are echoed as a failure message.

    R="${?}"
    echo "Validation failed! :'-("
    if [ ${*} ]; then
        echo "${*}"
    fi
    exit ${R}
}

validate_git_repo() {
    # tests that the given relative path exists and is a git repo
    git -C ${TARGET_DIR}/${1} status >& /dev/null || fail
}

validate_toolchain_linux() {
    # test that the expected packages are installed
    dpkg-query -s git cmake gcc-arm-none-eabi build-essential gdb-multiarch automake autoconf build-essential texinfo libtool libftdi-dev libusb-1.0-0-dev >& /dev/null || fail
}

install_dev_env_deps_linux() {
    # Install development environment dependencies for Linux

    # Avoid a certain dependency by installing ssh-client without recommends. See
    # https://github.com/raspberrypi/pico-setup/pull/20#discussion_r608793993 for details.
    $(sudo_wrapper) apt install -y --no-install-recommends ssh-client

    DEPS="autoconf automake build-essential cmake gcc-arm-none-eabi gdb-multiarch git libftdi-dev libtool libusb-1.0-0-dev minicom pkg-config python3 texinfo"
    if debian || ubuntu; then
        DEPS="${DEPS} libstdc++-arm-none-eabi-newlib"
    fi
    $(sudo_wrapper) apt install -y ${DEPS}
}

brew_install_idempotent() {
    # For some reason, brew install is not idempotent. This function succeeds even when the package is already installed.
    brew list ${*} || brew install ${*}
    return ${?}
}

validate_toolchain_mac() {
    # test that the expected packages are installed
    brew list git cmake pkg-config libtool automake libusb wget pkg-config gcc texinfo arm-none-eabi-gcc >& /dev/null 
}

install_dev_env_deps_mac() {
    # Install development environment dependencies for mac

    brew_install_idempotent ArmMbed/homebrew-formulae/arm-none-eabi-gcc automake cmake git libtool libusb gcc minicom pkg-config texinfo wget
}

create_TARGET_DIR() {
    # Creates ./pico directory if necessary

    mkdir -p "${TARGET_DIR}"
}

clone_raspberrypi_repo() {
    # Clones the given repo name from GitHub and inits any submodules
    # $1 should be the full name of the repo, ex: pico-sdk
    # $2 should be the branch name. Defaults to master.
    # all other args are passed to git clone
    REPO_NAME="${1}"
    if shift && [ ${#} -gt 0 ]; then
        BRANCH="${1}"
        # Can't just say `shift` because `set -e` will think it's an error and terminate the script.
        shift || true
    else
        BRANCH=master
    fi

    REPO_URL="https://github.com/raspberrypi/${REPO_NAME}.git"
    REPO_DIR="${TARGET_DIR}/${REPO_NAME}"

    if [ -d "${REPO_DIR}" ]; then
        echo "${REPO_DIR} already exists. Updating."
        git -C "${REPO_DIR}" pull --ff-only
    else
        echo "Cloning ${REPO_URL}"
        if [ ${#} -gt 0 ]; then
            git -C "${TARGET_DIR}" clone -b "${BRANCH}" "${REPO_URL}" ${*}
        else
            git -C "${TARGET_DIR}" clone -b "${BRANCH}" "${REPO_URL}"
        fi

        # Any submodules
        git -C "${REPO_DIR}" submodule update --init
    fi
}

warn_for_bashrc() {
    # Earlier versions of this script set environment variables in .bashrc. The location has moved to .profile or
    # .zprofile. If the user has a .bashrc defining any pico dev env variables, they could conflict with the settings
    # in the other files. This function raises a warning for the user.

    REGEX="^\s*export\s+\"?PICO_SDK_PATH="
    if grep -q "${REGEX}" ~/.bashrc; then
        echo "Your ~/.bashrc file contains the following line, which may conflict with this script's settings. We recommend removing it to prevent possible issues."
        echo -n "    "; grep "${REGEX}" ~/.bashrc
    fi
}

set_env() {
    # Permanently sets an environment variable by adding it to the current user's profile script
    # The form of the arguments should be `FOO foo`, which sets the environment variable `FOO=foo`
    NAME="${1}"
    VALUE="${2}"
    EXPR="${NAME}=${VALUE}"

    # detect appropriate file for setting env vars
    if echo "${SHELL}" | grep -q zsh; then
        # zsh detected
        FILE=~/.zprofile
    else
        # sh, bash and others
        FILE=~/.profile
    fi

    # ensure that appends go to a new line
    if [ -f "${FILE}" ]; then
        if ! ( tail -n 1 "${FILE}" | grep -q "^$" ); then
            # FILE exists but has no trailing newline. Adding newline.
            echo >> "${FILE}"
        fi
    fi

    # set for now
    export "${EXPR}"

    # set for later
    REGEX="^\s*export\s+\"?${NAME}="
    if grep -q "${REGEX}" "${FILE}"; then
        # Warn the user
        echo "Your ${FILE} already contains the following environment variable definition(s):"
        grep "${REGEX}" "${FILE}"
        echo "This script would normally set the following line. We're adding it, but commented out, so that you can choose which you want."
        echo "export \"${EXPR}\""
        # Write to file
        echo "# pico_setup.sh commented out the following line because it conflicts with another line in this file. You should choose one or the other." >> "${FILE}"
        echo "# export \"${EXPR}\"" >> "${FILE}"
    else
        echo "Setting env variable ${EXPR} in ${FILE}"
        echo "export \"${EXPR}\"" >> "${FILE}"
    fi
}

validate_pico_sdk() {
    validate_git_repo pico-sdk

    # test that the SDK env var is set and correct
    test "${PICO_SDK_PATH}" = "${TARGET_DIR}/pico-sdk" || fail
}

setup_sdk() {
    # Download the SDK
    clone_raspberrypi_repo pico-sdk

    # Set env var PICO_SDK_PATH
    set_env PICO_SDK_PATH "${TARGET_DIR}/pico-sdk"
}

validate_uart() {
    # test that the UART is configured. Only works on Raspberry Pi OS on Raspberry Pi hardware.
    dpkg-query -s minicom  >& /dev/null || fail
    grep -q "enable_uart=1" /boot/config.txt || fail
    # note that the test for console=serial0 tests for the absence of a string
    grep -q "console=serial0" /boot/cmdline.txt && fail
}

enable_uart() {
    # Enable UART
    echo "Disabling Linux serial console (UART) so we can use it for pico"
    $(sudo_wrapper) raspi-config nonint do_serial 2
    echo "You must run sudo reboot to finish UART setup"
}

phase_1() {
    # Setup minimum dev environment
    echo "Entering phase 1: Setup minimum dev environment"

    if mac; then
        install_dev_env_deps_mac
        validate_toolchain_mac
    else
        install_dev_env_deps_linux
        validate_toolchain_linux
    fi

    create_TARGET_DIR
    setup_sdk
    validate_pico_sdk

    if raspberry_pi && which raspi-config >> /dev/null; then
        enable_uart
        validate_uart
    else
        echo "Not configuring UART, because either this is not a Raspberry Pi computer, or raspi-config is not available."
    fi
}

build_examples() {
    # Build a couple of examples
    echo "Building selected examples"
    
    # Save the working directory
    pushd "${TARGET_DIR}/pico-examples" >> /dev/null

    mkdir -p build
    cd build
    cmake ../ -DCMAKE_BUILD_TYPE=Debug

    for EXAMPLE in blink hello_world; do
        echo "Building $EXAMPLE"
        cd "$EXAMPLE"
        make -j${JNUM}
        cd ..
    done

    # Restore the working directory
    popd >> /dev/null
}

validate_pico_extras() {
    validate_git_repo pico-extras
}

validate_pico_examples() {
    validate_git_repo pico-examples

    # test that blink is built
    test -f ${TARGET_DIR}/pico-examples/build/blink/blink.uf2 || fail

    # test that hello_serial is built
    test -f ${TARGET_DIR}/pico-examples/build/hello_world/serial/hello_serial.uf2 || fail

    # test that hello_usb is built
    test -f ${TARGET_DIR}/pico-examples/build/hello_world/usb/hello_usb.uf2 || fail
}

validate_pico_playground() {
    validate_git_repo pico-playground
}

phase_2() {
    # Setup tutorial repos
    echo "Entering phase 2: Setting up tutorial repos"

    for REPO_NAME in pico-examples pico-extras pico-playground; do
        clone_raspberrypi_repo "${REPO_NAME}"
    done

    build_examples

    validate_pico_examples
    validate_pico_extras
    validate_pico_playground
}

validate_picotool() {
    validate_git_repo picotool

    # test that the binary is built
    test -x ${TARGET_DIR}/picotool/build/picotool || fail

    # test that picotool is installed in the expected location
    test -x /usr/local/bin/picotool || fail
}

setup_picotool() {
    # Downloads, builds, and installs picotool
    echo "Setting up picotool"

    # Save the working directory
    pushd "${TARGET_DIR}" >> /dev/null

    clone_raspberrypi_repo picotool
    cd "${TARGET_DIR}/picotool"
    mkdir -p build
    cd build
    cmake ../
    make -j${JNUM}

    echo "Installing picotool to /usr/local/bin/picotool"
    $(sudo_wrapper) cp "${TARGET_DIR}/picotool/build/picotool" /usr/local/bin/

    # Restore the working directory
    popd >> /dev/null
}

validate_openocd() {
    validate_git_repo openocd

    # test that the binary is built
    test -x ${TARGET_DIR}/openocd/src/openocd || fail
}

setup_openocd() {
    # Download, build, and install OpenOCD for picoprobe and bit-banging without picoprobe
    echo "Setting up OpenOCD"

    # Save the working directory
    pushd "${TARGET_DIR}" >> /dev/null

    clone_raspberrypi_repo openocd picoprobe --depth=1
    cd "${TARGET_DIR}/openocd"
    ./bootstrap
    OPTS="--enable-ftdi --enable-bcm2835gpio  --enable-picoprobe"
    if linux; then
        # sysfsgpio is only available on linux
        OPTS="${OPTS} --enable-sysfsgpio"
    fi
    ./configure ${OPTS}
    make -j${JNUM}
    $(sudo_wrapper) make -C "${TARGET_DIR}/openocd" install

    # Restore the working directory
    popd >> /dev/null
}

validate_picoprobe() {
    validate_git_repo picoprobe || fail

    # test that the binary is built
    test -f ${TARGET_DIR}/picoprobe/build/picoprobe.uf2 || fail
}

setup_picoprobe() {
    # Download and build picoprobe. Requires that OpenOCD is already setup
    echo "Setting up picoprobe"

    # Save the working directory
    pushd "${TARGET_DIR}" >> /dev/null

    clone_raspberrypi_repo picoprobe
    cd "${TARGET_DIR}/picoprobe"
    mkdir -p build
    cd build
    cmake ..
    make -j${JNUM}

    # Restore the working directory
    popd >> /dev/null
}

validate_vscode_linux() {
    dpkg-query -s code  >& /dev/null || fail
}

install_vscode_linux() {
    # Install Visual Studio Code

    # VS Code is specially added to Raspberry Pi OS repos, but might not be present on Debian/Ubuntu. So we check first.
    if ! apt-cache show code >& /dev/null; then
        echo "It appears that your APT repos do not offer Visual Studio Code. Skipping."
        return
    fi

    echo "Installing Visual Studio Code"

    $(sudo_wrapper) apt install -y code

    # Get extensions
    code --install-extension marus25.cortex-debug
    code --install-extension ms-vscode.cmake-tools
    code --install-extension ms-vscode.cpptools
}

validate_vscode_mac() {
    echo "Not yet implemented: testing Visual Studio Code on macOS"
}

install_vscode_mac() {
    echo "Not yet implemented: installing Visual Studio Code on macOS"
}

phase_3() {
    # Setup recommended tools
    echo "Setting up recommended tools"

    setup_picotool
    validate_picotool

    setup_openocd
    validate_openocd

    setup_picoprobe
    validate_picoprobe

    # Install Visual Studio Code
    if mac; then
        install_vscode_mac
        validate_vscode_mac
    else
        if dpkg-query -s xserver-xorg >& /dev/null; then
            install_vscode_linux
            validate_vscode_linux
        else
            echo "Not installing Visual Studio Code because it looks like XWindows is not installed."
        fi
    fi
}

main() {
    phase_0
    phase_1
    phase_2
    phase_3

    echo "Congratulations, installation is complete. :D"
}

main
