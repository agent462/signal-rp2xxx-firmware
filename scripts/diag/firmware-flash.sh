#!/bin/bash
#
# Firmware Flash & Monitor
# Builds firmware locally, copies to a Pi, flashes via BOOTSEL,
# and reads serial output to verify the firmware is running.
#
# Usage:
#   ./scripts/diag/firmware-flash.sh [options]
#
# Options:
#   --board NAME         Board directory name (default: scorpio)
#   -p, --player HOST    Pi host (user@ip, default: signal@192.168.86.238)
#   -b, --build          Build firmware before flashing
#   -s, --serial SEC     Monitor serial output for SEC seconds (default: 8)
#   -S, --serial-only    Skip flash, just monitor serial output
#   -v, --verbose        Show detailed output
#   -h, --help           Show this help
#
# Boards:
#   scorpio              Adafruit Feather RP2040 SCORPIO (default)
#   pico2w_8ch           Raspberry Pi Pico 2 W (8-channel)
#   signal8              Signal 8 controller (RP2350A + CM5)
#
# Examples:
#   ./scripts/diag/firmware-flash.sh                    # flash SCORPIO (default)
#   ./scripts/diag/firmware-flash.sh -b                 # build + flash SCORPIO
#   ./scripts/diag/firmware-flash.sh --board pico2w_8ch -b  # build + flash Pico 2 W
#   ./scripts/diag/firmware-flash.sh -b -s 20           # build, flash, 20s serial
#   ./scripts/diag/firmware-flash.sh -S                 # just read serial output
#   ./scripts/diag/firmware-flash.sh -p pi@10.0.0.5     # custom host
#
# Prerequisites:
#   - Board must be in BOOTSEL mode (hold BOOTSEL while plugging USB)
#     OR already running firmware with USB serial (for --serial-only)
#   - Pi must be reachable via SSH (key-based auth recommended)
#   - For --build: ARM toolchain installed (brew install --cask gcc-arm-embedded)
#

set -e

# Default configuration
PLAYER_HOST="signal@192.168.86.238"
BOARD="scorpio"
BUILD=false
SERIAL_SECS=8
SERIAL_ONLY=false
VERBOSE=false

# Paths (FW_DIR and UF2_FILE set after argument parsing)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
MOUNT_PATH="/media"  # set dynamically after SSH_USER is known

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'
BOLD='\033[1m'

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --board)
            BOARD="$2"
            shift 2
            ;;
        -p|--player)
            PLAYER_HOST="$2"
            shift 2
            ;;
        -b|--build)
            BUILD=true
            shift
            ;;
        -s|--serial)
            SERIAL_SECS="$2"
            shift 2
            ;;
        -S|--serial-only)
            SERIAL_ONLY=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            sed -n '3,36p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# resolve board-specific paths
FW_DIR="$REPO_DIR/boards/$BOARD"

if [[ ! -d "$FW_DIR" ]]; then
    echo -e "${RED}[error]${NC} Board directory not found: $FW_DIR"
    echo "Available boards:"
    ls "$REPO_DIR/boards/"
    exit 1
fi

# map board name to UF2 binary name
case $BOARD in
    scorpio)      UF2_NAME="signal_rp2040.uf2" ;;
    pico2w_8ch)   UF2_NAME="signal_pico2w.uf2" ;;
    signal8)      UF2_NAME="signal_8.uf2" ;;
    *)            UF2_NAME="signal_${BOARD}.uf2" ;;
esac
UF2_FILE="$FW_DIR/build/$UF2_NAME"

# extract SSH user for mount path
SSH_USER="${PLAYER_HOST%%@*}"
if [[ "$SSH_USER" == "$PLAYER_HOST" ]]; then
    SSH_USER="signal"
fi
MOUNT_PATH="/media/$SSH_USER/RPI-RP2"

# Helper functions
log() {
    echo -e "${CYAN}[flash]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[flash]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[flash]${NC} $1"
}

log_error() {
    echo -e "${RED}[flash]${NC} $1"
}

verbose() {
    if $VERBOSE; then
        echo -e "${BLUE}[debug]${NC} $1"
    fi
}

ssh_cmd() {
    local host=$1
    shift
    ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$host" "$@" 2>/dev/null
}

# Phase 1: Build (optional)
build_firmware() {
    if ! $BUILD; then
        return
    fi

    log "Building firmware..."

    if [[ ! -f "$FW_DIR/CMakeLists.txt" ]]; then
        log_error "CMakeLists.txt not found at $FW_DIR"
        exit 1
    fi

    # ensure build directory exists and is configured
    if [[ ! -f "$FW_DIR/build/Makefile" ]]; then
        log "Running cmake..."
        mkdir -p "$FW_DIR/build"
        (cd "$FW_DIR/build" && cmake .. 2>&1) | while IFS= read -r line; do
            verbose "  cmake: $line"
        done
    fi

    # build
    local output
    output=$(cd "$FW_DIR/build" && make -j4 2>&1)
    local rc=$?

    if $VERBOSE; then
        echo "$output" | while IFS= read -r line; do
            verbose "  make: $line"
        done
    fi

    if [[ $rc -ne 0 ]]; then
        log_error "Build failed"
        echo "$output" | tail -20
        exit 1
    fi

    local size
    size=$(ls -lh "$UF2_FILE" | awk '{print $5}')
    log_success "Build complete: $size"
}

# Phase 2: Copy UF2 to Pi
copy_firmware() {
    log "Copying UF2 to $PLAYER_HOST..."

    if [[ ! -f "$UF2_FILE" ]]; then
        log_error "UF2 not found: $UF2_FILE"
        log_error "Run with --build or build manually first"
        exit 1
    fi

    local size
    size=$(ls -lh "$UF2_FILE" | awk '{print $5}')
    verbose "UF2 size: $size"

    scp -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$UF2_FILE" "$PLAYER_HOST:/tmp/$UF2_NAME" 2>/dev/null
    log_success "Copied to /tmp/$UF2_NAME"
}

# Phase 3: Flash via BOOTSEL
flash_firmware() {
    log "Checking for board in BOOTSEL mode..."

    # check for RP2 Boot USB device (vendor 2e8a, product 0003=RP2040, 000f=RP2350)
    local usb_info
    usb_info=$(ssh_cmd "$PLAYER_HOST" "lsusb 2>/dev/null | grep '2e8a'" || true)

    if echo "$usb_info" | grep -qE "0003|000f"; then
        verbose "USB: RP2 Boot detected"
    elif echo "$usb_info" | grep -q "2e8a"; then
        log_error "Board is running firmware, not in BOOTSEL mode"
        log_error "Hold BOOTSEL while plugging USB, or hold BOOTSEL + press RESET"
        exit 1
    else
        log_error "No RP2040/RP2350 device found on USB"
        log_error "Check USB connection to board"
        exit 1
    fi

    # ensure mount point exists
    if ! ssh_cmd "$PLAYER_HOST" "test -d '$MOUNT_PATH'" 2>/dev/null; then
        verbose "Mount point does not exist, creating..."
        ssh_cmd "$PLAYER_HOST" "sudo mkdir -p '$MOUNT_PATH'"
    fi

    # unmount any stale mount from a previous flash cycle.
    # Pi OS Lite has no automounter, so the mount becomes stale
    # when SCORPIO reboots after flashing.
    if ssh_cmd "$PLAYER_HOST" "mountpoint -q '$MOUNT_PATH'" 2>/dev/null; then
        verbose "Unmounting stale mount at $MOUNT_PATH"
        ssh_cmd "$PLAYER_HOST" "sudo umount '$MOUNT_PATH'" 2>/dev/null || true
    fi

    # find the BOOTSEL block device (RP2040 presents as 128M FAT volume)
    local blkdev
    blkdev=$(ssh_cmd "$PLAYER_HOST" "lsblk -rno NAME,SIZE | grep '128M' | head -1 | awk '{print \$1}'" || true)

    if [[ -z "$blkdev" ]]; then
        log_error "Cannot find BOOTSEL block device"
        exit 1
    fi

    # use the partition if it exists, otherwise the raw device
    local part="${blkdev}1"
    if ssh_cmd "$PLAYER_HOST" "test -b '/dev/$part'" 2>/dev/null; then
        blkdev="$part"
    fi

    verbose "Mounting /dev/$blkdev at $MOUNT_PATH"
    ssh_cmd "$PLAYER_HOST" "sudo mount '/dev/$blkdev' '$MOUNT_PATH'"

    # verify mount
    if ! ssh_cmd "$PLAYER_HOST" "test -f '$MOUNT_PATH/INFO_UF2.TXT'" 2>/dev/null; then
        log_error "BOOTSEL drive mounted but INFO_UF2.TXT not found"
        log_error "Try: ssh $PLAYER_HOST 'ls $MOUNT_PATH/'"
        exit 1
    fi

    verbose "BOOTSEL drive mounted at $MOUNT_PATH"

    # flash
    log "Flashing..."
    ssh_cmd "$PLAYER_HOST" "sudo cp '/tmp/$UF2_NAME' '$MOUNT_PATH/'"

    log_success "Firmware written, board is rebooting..."

    # wait for reboot: BOOTSEL device disappears, ttyACM appears
    sleep 3
}

# Phase 4: Monitor serial output
monitor_serial() {
    log "Waiting for USB serial..."

    # wait for ttyACM device (up to 10 seconds)
    local attempts=0
    local tty_dev=""
    while [[ $attempts -lt 20 ]]; do
        tty_dev=$(ssh_cmd "$PLAYER_HOST" "ls /dev/ttyACM0 2>/dev/null" || true)
        if [[ -n "$tty_dev" ]]; then
            break
        fi
        sleep 0.5
        attempts=$((attempts + 1))
    done

    if [[ -z "$tty_dev" ]]; then
        log_warn "No /dev/ttyACM0 found after 10s"
        log_warn "Check USB connection or try: ssh $PLAYER_HOST 'ls /dev/ttyACM*'"
        return
    fi

    verbose "Serial device: $tty_dev"

    # verify it's our firmware via USB product string
    local usb_info
    usb_info=$(ssh_cmd "$PLAYER_HOST" "lsusb | grep '2e8a'" || true)
    verbose "USB: $usb_info"

    log "Reading serial output for ${SERIAL_SECS}s..."
    echo ""

    # read serial with timeout
    ssh_cmd "$PLAYER_HOST" "timeout $SERIAL_SECS cat /dev/ttyACM0 2>/dev/null" || true

    echo ""
    log_success "Serial monitoring complete"
}

# Main
main() {
    echo ""
    echo -e "${BOLD}${CYAN}Firmware Flash & Monitor${NC}"
    echo "========================"
    echo ""
    echo "Board: $BOARD"
    echo "Host:  $PLAYER_HOST"

    if $SERIAL_ONLY; then
        echo "Mode: serial monitor only"
        echo ""
        monitor_serial
    else
        if $BUILD; then
            echo "Mode: build + flash + serial"
        else
            echo "Mode: flash + serial"
        fi
        echo ""

        build_firmware
        copy_firmware
        flash_firmware
        monitor_serial
    fi

    log_success "Done"
}

main "$@"
