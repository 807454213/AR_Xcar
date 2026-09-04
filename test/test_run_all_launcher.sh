#!/bin/bash

set -euo pipefail

SOURCE_SCRIPT="/home/orangepi/Desktop/run_all.sh"
RUN_USER="orangepi"
RUN_UID="$(id -u "$RUN_USER")"
RUN_HOME="/home/orangepi"
TMP_DIR="$(mktemp -d)"

cleanup() {
    if [ -n "${TMP_DIR:-}" ] && [ -d "$TMP_DIR" ]; then
        find "$TMP_DIR" -type f -exec unlink {} \; 2>/dev/null || true
        find "$TMP_DIR" -type d -empty -delete 2>/dev/null || true
    fi
}
trap cleanup EXIT

chmod 0777 "$TMP_DIR"

APP1="$TMP_DIR/app1"
APP2="$TMP_DIR/app2"
FIXTURE="$TMP_DIR/run_all.sh"

cat > "$APP1" <<'APP'
#!/bin/bash
{
    echo "user=$(id -un)"
    echo "uid=$(id -u)"
    echo "DISPLAY=${DISPLAY:-}"
    echo "XAUTHORITY=${XAUTHORITY:-}"
} > "$(dirname "$0")/app1.env"
sleep 30
APP

cat > "$APP2" <<'APP'
#!/bin/bash
{
    echo "user=$(id -un)"
    echo "uid=$(id -u)"
    echo "DISPLAY=${DISPLAY:-}"
    echo "XAUTHORITY=${XAUTHORITY:-}"
} > "$(dirname "$0")/app2.env"
sleep 30
APP

chmod 0755 "$APP1" "$APP2"

cp "$SOURCE_SCRIPT" "$FIXTURE"
sed -i \
    -e "s|^APP1=.*|APP1=\"$APP1\"|" \
    -e "s|^APP2=.*|APP2=\"$APP2\"|" \
    -e "s|^KEY_FIFO=.*|KEY_FIFO=\"$TMP_DIR/xcar_key_cmd\"|" \
    "$FIXTURE"
chmod 0755 "$FIXTURE"

set +e
timeout 5s "$FIXTURE" > "$TMP_DIR/output.log" 2>&1
status=$?
set -e

if [ "$status" -ne 124 ]; then
    echo "expected fixture to stay running until timeout, got exit $status"
    cat "$TMP_DIR/output.log"
    exit 1
fi

if [ ! -f "$TMP_DIR/app1.env" ] || [ ! -f "$TMP_DIR/app2.env" ]; then
    echo "expected both fake apps to start"
    cat "$TMP_DIR/output.log"
    exit 1
fi

for env_file in "$TMP_DIR/app1.env" "$TMP_DIR/app2.env"; do
    grep -q "^user=$RUN_USER$" "$env_file" || {
        echo "$env_file did not run as $RUN_USER"
        cat "$env_file"
        exit 1
    }
    grep -q "^uid=$RUN_UID$" "$env_file" || {
        echo "$env_file did not run as uid $RUN_UID"
        cat "$env_file"
        exit 1
    }
    grep -q "^DISPLAY=:0$" "$env_file" || {
        echo "$env_file did not receive DISPLAY=:0"
        cat "$env_file"
        exit 1
    }
    grep -q "^XAUTHORITY=$RUN_HOME/.Xauthority$" "$env_file" || {
        echo "$env_file did not receive the expected XAUTHORITY"
        cat "$env_file"
        exit 1
    }
done

echo "run_all launcher guard ok"
