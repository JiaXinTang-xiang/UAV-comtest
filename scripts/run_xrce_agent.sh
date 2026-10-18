#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
AGENT_PREFIX="$PROJECT_DIR/Micro-XRCE-DDS-Agent/install"
if [[ ! -x "$AGENT_PREFIX/bin/MicroXRCEAgent" ]]; then
    echo "Agent executable missing: $AGENT_PREFIX/bin/MicroXRCEAgent" >&2
    exit 1
fi

# Keep these DDS libraries local to the Agent process.
export LD_LIBRARY_PATH="$AGENT_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if (( $# == 0 )); then
    set -- udp4 -p 8888
fi
exec "$AGENT_PREFIX/bin/MicroXRCEAgent" "$@"
