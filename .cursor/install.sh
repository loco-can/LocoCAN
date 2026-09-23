#!/usr/bin/env bash
#
# Cloud Agent install script for the LocoCAN Arduino library.
#
# Sets up an Arduino toolchain (arduino-cli + arduino-lint + the AVR core)
# and registers this repository as an installed library so example sketches
# can be compiled and the library manifest can be linted.
#
# The script is idempotent: it can be re-run safely and skips work that is
# already done.

set -euo pipefail

# Pinned tool versions for reproducible builds.
ARDUINO_CLI_VERSION="1.5.1"
ARDUINO_LINT_VERSION="1.3.0"
AVR_CORE="arduino:avr"

# Resolve the repository root from this script's location so the symlink is
# correct regardless of the working directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

BINDIR="/usr/local/bin"

# Prefer sudo for writing into /usr/local/bin, but fall back gracefully.
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  if command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
    SUDO="sudo"
  else
    BINDIR="$HOME/.local/bin"
    mkdir -p "$BINDIR"
  fi
fi

install_arduino_cli() {
  if command -v arduino-cli >/dev/null 2>&1 \
      && arduino-cli version | grep -q "$ARDUINO_CLI_VERSION"; then
    echo "arduino-cli $ARDUINO_CLI_VERSION already installed."
    return
  fi
  echo "Installing arduino-cli $ARDUINO_CLI_VERSION..."
  local tmp
  tmp="$(mktemp -d)"
  curl -fsSL "https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh" \
    | BINDIR="$tmp" sh -s "$ARDUINO_CLI_VERSION"
  $SUDO install -m 0755 "$tmp/arduino-cli" "$BINDIR/arduino-cli"
  rm -rf "$tmp"
}

install_arduino_lint() {
  if command -v arduino-lint >/dev/null 2>&1 \
      && arduino-lint --version | grep -q "$ARDUINO_LINT_VERSION"; then
    echo "arduino-lint $ARDUINO_LINT_VERSION already installed."
    return
  fi
  echo "Installing arduino-lint $ARDUINO_LINT_VERSION..."
  local tmp
  tmp="$(mktemp -d)"
  curl -fsSL "https://raw.githubusercontent.com/arduino/arduino-lint/main/etc/install.sh" \
    | BINDIR="$tmp" sh -s "$ARDUINO_LINT_VERSION"
  $SUDO install -m 0755 "$tmp/arduino-lint" "$BINDIR/arduino-lint"
  rm -rf "$tmp"
}

install_arduino_cli
install_arduino_lint

export PATH="$BINDIR:$PATH"

# Initialise the arduino-cli config (idempotent).
arduino-cli config init --overwrite >/dev/null

# Update the package index and install the AVR core used by the boards this
# library targets (ATmega328 / ATtiny85). core install is a no-op if present.
arduino-cli core update-index
arduino-cli core install "$AVR_CORE"

# Register this repository as an Arduino library via a symlink so sketches can
# `#include <can_protocol.h>` / `#include <loco_config.h>`.
USERDIR="$(arduino-cli config get directories.user)"
mkdir -p "$USERDIR/libraries"
ln -sfn "$REPO_ROOT" "$USERDIR/libraries/LocoCAN"

echo "LocoCAN environment ready."
arduino-cli lib list | grep -i lococan || true
