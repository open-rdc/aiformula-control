#!/usr/bin/env bash
#
# aiformula 足回り制御機 システム状態チェック
#
# chassis_driver.launch.py を起動した状態で実行し、以下をまとめて確認する。
#
#   1. ROS 2 実行環境
#   2. ノードの生存
#   3. トピックの出版状況と周波数
#   4. ODrive の内部状態 (エラー / 温度 / バス電圧・電流)
#   5. CAN バス
#   6. Jetsonとの接続
#
# このリポジトリ (aiformula-control) は DellPC 上での実行のみを前提とする。
# 知覚 / 自己位置 / 計画 / TF といった Jetson 側で確認すべき項目は扱わない。
# DellPC は日本語ロケール非対応のため、出力はすべて英語で行う
# (このファイル内のコメントは表示されないので日本語のままとする)。
#
#   使い方:
#     ./system_check.sh
#

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# 設定
#   このスクリプトを置くマシンに合わせて書き換える。
# ---------------------------------------------------------------------------

# 各マシンの固定 IP
readonly JETSON_IP="192.168.10.10"
readonly DELL_PC_IP="192.168.10.20"

readonly OWN_NAME="DellPC"  OWN_IP="$DELL_PC_IP"
readonly PEER_NAME="Jetson" PEER_IP="$JETSON_IP"

# 周波数の計測時間 [秒]
readonly MEASURE_DURATION=10.0

# --- ODrive のしきい値 -----------------------------------------------------
#   機体の構成 (バッテリ電圧 / モータ / 放熱) に合わせて調整する。
#   温度は計測期間中の最大値、バス電圧は最小値と最大値で判定する。
readonly ODRIVE_FET_TEMP_WARN=70.0     # インバータ (FET) 温度 [degC] 警告
readonly ODRIVE_FET_TEMP_NG=90.0       # インバータ (FET) 温度 [degC] 異常
readonly ODRIVE_MOTOR_TEMP_WARN=80.0   # モータ温度 [degC] 警告
readonly ODRIVE_MOTOR_TEMP_NG=100.0    # モータ温度 [degC] 異常
readonly ODRIVE_BUS_VOLTAGE_MIN=20.0   # バス電圧 [V] 下限 (バッテリ構成に合わせる)
readonly ODRIVE_BUS_VOLTAGE_MAX=52.0   # バス電圧 [V] 上限 (回生時の跳ね上がり込み)
readonly ODRIVE_BUS_CURRENT_WARN=20.0  # バス電流 [A] 警告 (絶対値)

# odrive_can_launch.yaml が読めなかったときの既定値
readonly ODRIVE_NODE_ID_DEFAULT=24
readonly ODRIVE_NS_DEFAULT="odrive_axis0"
readonly ODRIVE_NODE_NAME_DEFAULT="can_node"

# ---------------------------------------------------------------------------
# ログ出力
# ---------------------------------------------------------------------------
OK_COUNT=0
NG_COUNT=0
WARN_COUNT=0
SKIP_COUNT=0

if [[ -t 1 ]]; then
  C_OK=$'\033[32m'; C_NG=$'\033[31m'; C_WARN=$'\033[33m'
  C_SKIP=$'\033[90m'; C_SEC=$'\033[1;36m'; C_OFF=$'\033[0m'
else
  C_OK=""; C_NG=""; C_WARN=""; C_SKIP=""; C_SEC=""; C_OFF=""
fi

NAME_WIDTH=48

# 端末上の表示幅を数える (全角文字は 2 桁として扱う)
disp_width() {
  local s="$1" w=0 c cp i
  for (( i = 0; i < ${#s}; i++ )); do
    c="${s:i:1}"
    printf -v cp '%d' "'${c}" 2>/dev/null || cp=63
    if (( cp >= 0x1100 && ( \
          cp <= 0x115F || \
          (cp >= 0x2E80 && cp <= 0xA4CF) || \
          (cp >= 0xAC00 && cp <= 0xD7A3) || \
          (cp >= 0xF900 && cp <= 0xFAFF) || \
          (cp >= 0xFE30 && cp <= 0xFE6F) || \
          (cp >= 0xFF00 && cp <= 0xFF60) || \
          (cp >= 0xFFE0 && cp <= 0xFFE6) ) )); then
      w=$(( w + 2 ))
    else
      w=$(( w + 1 ))
    fi
  done
  printf '%d' "$w"
}

section() {
  printf "\n%s===== %s =====%s\n" "$C_SEC" "$1" "$C_OFF"
}

# report <OK|NG|WARN|SKIP> <項目名> [メッセージ]
#   OK   : 計測値などをその行に併記する
#   それ以外: 失敗内容と対処を次行にインデントして出す
report() {
  local status="$1" name="$2" msg="${3:-}"
  local color label pad len

  case "$status" in
    OK)   label="OK  "; color="$C_OK";   OK_COUNT=$((OK_COUNT + 1)) ;;
    NG)   label="NG  "; color="$C_NG";   NG_COUNT=$((NG_COUNT + 1)) ;;
    WARN) label="WARN"; color="$C_WARN"; WARN_COUNT=$((WARN_COUNT + 1)) ;;
    SKIP) label="SKIP"; color="$C_SKIP"; SKIP_COUNT=$((SKIP_COUNT + 1)) ;;
    *)    label="????"; color="" ;;
  esac

  len=$(disp_width "$name")
  if (( len < NAME_WIDTH )); then
    pad=$(printf '%*s' $((NAME_WIDTH - len)) '')
    pad=${pad// /.}
  else
    pad=" "
  fi

  if [[ "$status" == "OK" ]]; then
    printf "  %s %s %s%s%s  %s\n" "$name" "$pad" "$color" "$label" "$C_OFF" "$msg"
  else
    printf "  %s %s %s%s%s\n" "$name" "$pad" "$color" "$label" "$C_OFF"
    [[ -n "$msg" ]] && printf "       %s-> %s%s\n" "$color" "$msg" "$C_OFF"
  fi
}

info() {
  printf "       %s\n" "$1"
}

fatal() {
  printf "\n  %s%s%s\n\n" "$C_NG" "$1" "$C_OFF"
  exit 1
}

# ---------------------------------------------------------------------------
# 0. 実行環境
# ---------------------------------------------------------------------------
section "Environment"

if ! command -v ros2 >/dev/null 2>&1; then
  report NG "ros2 command" "ros2 command not found. Check that /opt/ros/humble/setup.bash and install/setup.bash are sourced."
  fatal "Aborting because the ROS 2 environment is not loaded."
fi
report OK "ros2 command" "$(command -v ros2)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  report NG "ROS_DISTRO" "ROS_DISTRO is not set. Check that /opt/ros/humble/setup.bash is sourced."
else
  report OK "ROS_DISTRO" "$ROS_DISTRO"
fi

report OK "ROS_DOMAIN_ID" "${ROS_DOMAIN_ID:-0 (unset)} (must match between DellPC and Jetson)"
report OK "RMW_IMPLEMENTATION" "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (default)} (must match on both machines)"
report OK "role" "DellPC (chassis control PC, aiformula-control)"

# ---------------------------------------------------------------------------
# パラメータ (chassis_driver_params.yaml / odrive_can_launch.yaml)
#   install 側を優先して探す (--symlink-install なら src と同一実体)。
# ---------------------------------------------------------------------------
CAN_IF="can_main"
ODRIVE_NODE_ID="$ODRIVE_NODE_ID_DEFAULT"
ODRIVE_NS="$ODRIVE_NS_DEFAULT"
ODRIVE_NODE_NAME="$ODRIVE_NODE_NAME_DEFAULT"

find_first() {
  local candidate
  for candidate in "$@"; do
    if [[ -f "$candidate" ]]; then
      realpath "$candidate" 2>/dev/null || printf '%s' "$candidate"
      return 0
    fi
  done
  return 1
}

CHASSIS_YAML="$(find_first \
  "${SCRIPT_DIR}/../install/chassis_driver/share/chassis_driver/config/chassis_driver_params.yaml" \
  "${SCRIPT_DIR}/chassis_driver/chassis_driver/config/chassis_driver_params.yaml")" || CHASSIS_YAML=""

ODRIVE_YAML="$(find_first \
  "${SCRIPT_DIR}/../install/chassis_driver/share/chassis_driver/launch/odrive_can_launch.yaml" \
  "${SCRIPT_DIR}/chassis_driver/chassis_driver/launch/odrive_can_launch.yaml")" || ODRIVE_YAML=""

if [[ -z "$CHASSIS_YAML" ]]; then
  report WARN "chassis_driver_params.yaml" "chassis_driver_params.yaml was not found. Falling back to the default CAN interface name (${CAN_IF})."
else
  PARAM_DUMP="$(python3 - "$CHASSIS_YAML" "$ODRIVE_YAML" <<'PY' 2>/dev/null
import sys
import yaml


def load(path):
    if not path:
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


chassis = load(sys.argv[1])
can = (chassis.get('socketcan_interface_node') or {}).get('ros__parameters', {}) or {}
print('CAN_IF=%s' % can.get('if_name', 'can_main'))

# odrive_can_launch.yaml から namespace / ノード名 / node_id を取り出す。
# CAN ID (base = node_id << 5) の算出に使うため、実際の起動設定から読む。
odrive = load(sys.argv[2] if len(sys.argv) > 2 else '')
for entry in (odrive.get('launch') or []):
    node = entry.get('node') if isinstance(entry, dict) else None
    if not node or node.get('exec') != 'odrive_can_node':
        continue
    print('ODRIVE_NS=%s' % node.get('namespace', 'odrive_axis0'))
    print('ODRIVE_NODE_NAME=%s' % node.get('name', 'can_node'))
    for p in (node.get('param') or []):
        if isinstance(p, dict) and p.get('name') == 'node_id':
            print('ODRIVE_NODE_ID=%d' % int(p.get('value', 24)))
    break
PY
)"

  if [[ -z "$PARAM_DUMP" ]]; then
    report WARN "chassis_driver_params.yaml" "Failed to parse ${CHASSIS_YAML}. Check the YAML syntax and that python3-yaml is installed. Using defaults."
  else
    while IFS='=' read -r key value; do
      case "$key" in
        CAN_IF)           CAN_IF="$value" ;;
        ODRIVE_NS)        ODRIVE_NS="$value" ;;
        ODRIVE_NODE_NAME) ODRIVE_NODE_NAME="$value" ;;
        ODRIVE_NODE_ID)   ODRIVE_NODE_ID="$value" ;;
      esac
    done <<< "$PARAM_DUMP"
    report OK "chassis_driver_params.yaml" "$CHASSIS_YAML"
  fi
fi

report OK "CAN interface name" "$CAN_IF"

if [[ -n "$ODRIVE_YAML" ]]; then
  report OK "odrive_can_launch.yaml" "$ODRIVE_YAML"
else
  report WARN "odrive_can_launch.yaml" "odrive_can_launch.yaml was not found. Assuming node_id=${ODRIVE_NODE_ID} and namespace /${ODRIVE_NS}."
fi

# ODrive の CAN ID は base = node_id << 5 に cmd_id を OR したもの
ODRIVE_CAN_BASE=$(( ODRIVE_NODE_ID << 5 ))
odrive_can_id() { printf '%03X' $(( ODRIVE_CAN_BASE | $1 )); }

ID_HEARTBEAT="$(odrive_can_id 0x01)"   # ControllerStatus: エラー / axis_state
ID_ERROR="$(odrive_can_id 0x03)"       # ODriveStatus: active_errors / disarm_reason
ID_ENCODER="$(odrive_can_id 0x09)"     # ControllerStatus: pos / vel
ID_IQ="$(odrive_can_id 0x14)"          # ControllerStatus: Iq
ID_TEMP="$(odrive_can_id 0x15)"        # ODriveStatus: FET / モータ温度
ID_BUS_VI="$(odrive_can_id 0x17)"      # ODriveStatus: バス電圧 / 電流
ID_TORQUE="$(odrive_can_id 0x1C)"      # ControllerStatus: トルク

report OK "ODrive CAN base ID" "node_id ${ODRIVE_NODE_ID} -> 0x${ID_HEARTBEAT} (heartbeat) .. 0x${ID_TORQUE} (torques)"

# ---------------------------------------------------------------------------
# 1〜4. ノード / トピック / 周波数 / ODrive 内部状態 (rclpy でまとめて計測)
# ---------------------------------------------------------------------------
run_ros_checks() {
  python3 - "$MEASURE_DURATION" "$ODRIVE_NS" "$ODRIVE_NODE_NAME" "$ODRIVE_NODE_ID" \
    "$ODRIVE_FET_TEMP_WARN" "$ODRIVE_FET_TEMP_NG" \
    "$ODRIVE_MOTOR_TEMP_WARN" "$ODRIVE_MOTOR_TEMP_NG" \
    "$ODRIVE_BUS_VOLTAGE_MIN" "$ODRIVE_BUS_VOLTAGE_MAX" \
    "$ODRIVE_BUS_CURRENT_WARN" <<'PY' 2>/dev/null
import sys
import time

DURATION = float(sys.argv[1])
ODRV_NS = '/' + sys.argv[2].strip('/')
ODRV_NODE = ODRV_NS + '/' + sys.argv[3]
ODRV_NODE_ID = int(sys.argv[4])
FET_WARN = float(sys.argv[5])
FET_NG = float(sys.argv[6])
MOTOR_WARN = float(sys.argv[7])
MOTOR_NG = float(sys.argv[8])
VBUS_MIN = float(sys.argv[9])
VBUS_MAX = float(sys.argv[10])
IBUS_WARN = float(sys.argv[11])

DISCOVERY_WAIT = 2.0
ODRV_BASE = ODRV_NODE_ID << 5

try:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.executors import SingleThreadedExecutor
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:  # noqa: BLE001
    print('RES\tNG\trclpy import\tFailed to import rclpy (%s). Check that install/setup.bash is sourced.' % exc)
    sys.exit(1)


def emit(kind, *fields):
    print('\t'.join([kind] + [str(f) for f in fields]), flush=True)


def can_topic(cmd_id):
    """ODrive の cmd_id に対応する socketcan_interface の受信トピック名。"""
    return '/can_rx_%03X' % (ODRV_BASE | cmd_id)


# ---------------------------------------------------------------------------
# ODrive のエラー定義 (odrive_base/include/odrive_enums.h と一致させること)
# ---------------------------------------------------------------------------
ODRIVE_ERRORS = [
    (0x00000001, 'INITIALIZING', 'The ODrive is still booting. Wait a moment and check again.'),
    (0x00000002, 'SYSTEM_LEVEL', 'Internal firmware error. Power-cycle the ODrive and check the firmware version.'),
    (0x00000004, 'TIMING_ERROR', 'The control loop missed its deadline. Check the firmware version and the CAN/encoder load.'),
    (0x00000008, 'MISSING_ESTIMATE', 'No position/velocity estimate. Check the encoder wiring and that calibration completed.'),
    (0x00000010, 'BAD_CONFIG', 'Invalid configuration. Review the ODrive config and run save_configuration().'),
    (0x00000020, 'DRV_FAULT', 'Gate driver fault. Check the motor phase wiring for shorts and the power supply.'),
    (0x00000040, 'MISSING_INPUT', 'No control input arrived within the watchdog window. Check that %s/control_message is published at 500 Hz.' % ODRV_NS),
    (0x00000100, 'DC_BUS_OVER_VOLTAGE', 'Bus over-voltage, usually regenerative braking. Check the brake resistor and dc_max_negative_current.'),
    (0x00000200, 'DC_BUS_UNDER_VOLTAGE', 'Bus under-voltage. Check the battery charge, the main switch, and the power wiring.'),
    (0x00000400, 'DC_BUS_OVER_CURRENT', 'Bus over-current. Check the current limit and the mechanical load.'),
    (0x00000800, 'DC_BUS_OVER_REGEN_CURRENT', 'Regenerative current exceeded the limit. Check the brake resistor rating.'),
    (0x00001000, 'CURRENT_LIMIT_VIOLATION', 'Measured current exceeded the limit. Check current_soft_max / current_hard_max and the motor load.'),
    (0x00002000, 'MOTOR_OVER_TEMP', 'Motor over-temperature. Stop driving and let it cool down; check motor_thermistor settings.'),
    (0x00004000, 'INVERTER_OVER_TEMP', 'Inverter (FET) over-temperature. Stop driving, improve cooling, and lower the current limit.'),
    (0x00008000, 'VELOCITY_LIMIT_VIOLATION', 'Velocity exceeded the limit. Check vel_limit and the gear ratio.'),
    (0x00010000, 'POSITION_LIMIT_VIOLATION', 'Position exceeded the limit. Check the position limit settings.'),
    (0x01000000, 'WATCHDOG_TIMER_EXPIRED', 'The axis watchdog expired. Check that control commands keep arriving over CAN.'),
    (0x02000000, 'ESTOP_REQUESTED', 'An emergency stop was requested over CAN. Release the E-stop and clear the errors.'),
    (0x04000000, 'SPINOUT_DETECTED', 'Spinout detected (encoder and motor disagree). Check the encoder mounting and the phase wiring.'),
    (0x08000000, 'BRAKE_RESISTOR_DISARMED', 'The brake resistor is disarmed. Check brake_resistance and the resistor wiring.'),
    (0x10000000, 'THERMISTOR_DISCONNECTED', 'The motor thermistor is disconnected. Check the thermistor wiring or disable it in the config.'),
    (0x40000000, 'CALIBRATION_ERROR', 'Calibration failed. Re-run the calibration sequence and check the procedure result.'),
]

AXIS_STATES = {
    0: 'UNDEFINED', 1: 'IDLE', 2: 'STARTUP_SEQUENCE', 3: 'FULL_CALIBRATION_SEQUENCE',
    4: 'MOTOR_CALIBRATION', 6: 'ENCODER_INDEX_SEARCH', 7: 'ENCODER_OFFSET_CALIBRATION',
    8: 'CLOSED_LOOP_CONTROL', 9: 'LOCKIN_SPIN', 10: 'ENCODER_DIR_FIND', 11: 'HOMING',
    12: 'ENCODER_HALL_POLARITY_CALIBRATION', 13: 'ENCODER_HALL_PHASE_CALIBRATION',
    14: 'ANTICOGGING_CALIBRATION',
}

PROCEDURE_RESULTS = {
    0: 'SUCCESS', 1: 'BUSY', 2: 'CANCELLED', 3: 'DISARMED', 4: 'NO_RESPONSE',
    5: 'POLE_PAIR_CPR_MISMATCH', 6: 'PHASE_RESISTANCE_OUT_OF_RANGE',
    7: 'PHASE_INDUCTANCE_OUT_OF_RANGE', 8: 'UNBALANCED_PHASES', 9: 'INVALID_MOTOR_TYPE',
    10: 'ILLEGAL_HALL_STATE', 11: 'TIMEOUT', 12: 'HOMING_WITHOUT_ENDSTOP',
    13: 'INVALID_STATE', 14: 'NOT_CALIBRATED', 15: 'NOT_CONVERGING',
}


def decode_errors(value):
    """エラービットフィールドを (名前一覧, 対処一覧) に展開する。"""
    names, hints = [], []
    known = 0
    for bit, name, hint in ODRIVE_ERRORS:
        if value & bit:
            names.append('%s (0x%08X)' % (name, bit))
            hints.append(hint)
            known |= bit
    rest = value & ~known
    if rest:
        names.append('UNKNOWN (0x%08X)' % rest)
    return names, hints


# (フルノード名, 深刻度, ヒント)
NODES = [
    ('/chassis_driver_node', 'error',
     'Check that chassis_driver.launch.py is running on the DellPC.'),
    ('/socketcan_interface_node', 'error',
     'Check that socketcan_interface_node is running and that binding to the CAN interface did not fail (see the startup log).'),
    (ODRV_NODE, 'error',
     'Check that odrive_can_launch.yaml is included (launch.odrive must be true in chassis_driver_params.yaml).'),
]

# (トピック, 型, 種別, 期待Hz, 下限Hz, 上限Hz, 深刻度, ヒント)
#   種別 rate   : 周波数まで判定する
#   種別 event  : 到着したことだけを判定する (発生契機が不定のもの)
#   種別 latched: transient_local。1 サンプル受け取れれば良い
CAN_MSG = 'socketcan_interface_msg/msg/SocketcanIF'

TOPICS = [
    # --- Jetson から届く指令 (DellPC が購読する側なので確認対象) ---
    ('/cmd_vel', 'steered_drive_msg/msg/SteeredDrive', 'rate', 20.0, 14.0, None, 'error',
     'Failed to receive velocity commands. This topic is published by the Jetson; check that controller_server_node is running there and check the DDS connectivity between the two machines.'),
    ('/vectornav/velocity_body', 'geometry_msgs/msg/TwistWithCovarianceStamped', 'rate', 20.0, 14.0, None, 'error',
     'Failed to receive body velocity. This topic is published by the Jetson; check that vectornav and vn_sensor_msgs are running there, and check the DDS connectivity between the two machines.'),
    ('/restart', 'std_msgs/msg/Empty', 'event', None, None, None, 'warn',
     'Failed to receive the restart command. It is not published until restart is sent from the gamepad, so this is normal before driving. If it never arrives, check the DDS connectivity between the two machines.'),

    # --- DellPC が出版する足回りの状態 ---
    ('/caster_data', 'std_msgs/msg/Float64MultiArray', 'rate', 500.0, 250.0, None, 'warn',
     'Failed to receive caster states. chassis_driver_node does not publish while in stop mode. Send restart from the gamepad and check that /cmd_vel is arriving.'),
    ('/caster_odom', 'nav_msgs/msg/Odometry', 'rate', 500.0, 250.0, None, 'warn',
     'Failed to receive caster odometry. chassis_driver_node does not publish it until the caster rotation encoder (/can_rx_013) is received at least once. Check the CAN wiring. Note: nothing currently subscribes to this topic.'),

    # --- CAN トピック ---
    ('/can_tx', CAN_MSG, 'rate', 500.0, 250.0, None, 'error',
     'Failed to receive CAN TX frames. Check that chassis_driver_node is running (interval_ms: 2 = 500 Hz) and check the executor log.'),
    ('/can_rx_012', CAN_MSG, 'event', None, None, None, 'error',
     'Failed to receive the caster steering angle encoder (CAN ID 0x012). socketcan_interface_node only creates topics for IDs it has received. Check the encoder board power, CAN wiring, and termination resistors.'),
    ('/can_rx_013', CAN_MSG, 'event', None, None, None, 'error',
     'Failed to receive the caster rotation encoder (CAN ID 0x013). Check the encoder board power, CAN wiring, and termination resistors.'),
    ('/can_rx_712', CAN_MSG, 'event', None, None, None, 'warn',
     'Failed to receive the emergency stop board (CAN ID 0x712). Check the board power and CAN wiring.'),

    # --- ODrive の周期メッセージ ---
    #   odrive_can_node は 7 種のうち必要なものが全て揃うまで status を出版しない
    #   (ODriveStatus: error/temperature/bus_vi, ControllerStatus: heartbeat/encoder/iq/torque)
    #   ため、どれが欠けているかを個別に見えるようにする。
    (can_topic(0x01), CAN_MSG, 'event', None, None, None, 'error',
     'Failed to receive the ODrive heartbeat. Check the ODrive power, the node_id setting (%d), and that the CAN bitrate matches.' % ODRV_NODE_ID),
    (can_topic(0x03), CAN_MSG, 'event', None, None, None, 'error',
     'Failed to receive the ODrive error message. ODriveStatus (errors / temperature / bus voltage) is never published without it. Set axis0.config.can.error_msg_rate_ms to a non-zero value (e.g. 100) with odrivetool and run save_configuration().'),
    (can_topic(0x09), CAN_MSG, 'event', None, None, None, 'warn',
     'Failed to receive ODrive encoder estimates. Set axis0.config.can.encoder_msg_rate_ms to a non-zero value; ControllerStatus is not published without it.'),
    (can_topic(0x14), CAN_MSG, 'event', None, None, None, 'warn',
     'Failed to receive the ODrive Iq message. Set axis0.config.can.iq_msg_rate_ms to a non-zero value; ControllerStatus is not published without it.'),
    (can_topic(0x15), CAN_MSG, 'event', None, None, None, 'error',
     'Failed to receive the ODrive temperature message, so FET / motor temperature cannot be monitored. Set axis0.config.can.temperature_msg_rate_ms to a non-zero value (e.g. 100) and run save_configuration().'),
    (can_topic(0x17), CAN_MSG, 'event', None, None, None, 'error',
     'Failed to receive the ODrive bus voltage/current message. Set axis0.config.can.bus_vi_msg_rate_ms to a non-zero value (e.g. 100) and run save_configuration().'),
    (can_topic(0x1C), CAN_MSG, 'event', None, None, None, 'warn',
     'Failed to receive the ODrive torque message. Set axis0.config.can.torque_msg_rate_ms to a non-zero value; ControllerStatus is not published without it.'),

    ('%s/control_message' % ODRV_NS, 'odrive_can/msg/ControlMessage', 'rate', 500.0, 250.0, None, 'warn',
     'Failed to receive ODrive control commands. chassis_driver_node does not publish while in stop mode. Check that restart was sent from the gamepad.'),
]

REL_NAME = {ReliabilityPolicy.RELIABLE: 'reliable', ReliabilityPolicy.BEST_EFFORT: 'best_effort'}
DUR_NAME = {DurabilityPolicy.VOLATILE: 'volatile', DurabilityPolicy.TRANSIENT_LOCAL: 'transient_local'}

rclpy.init(args=None)
node = rclpy.create_node('aiformula_system_check')
executor = SingleThreadedExecutor()
executor.add_node(node)


def spin_for(seconds):
    end = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)


# --- ディスカバリ待ち ---
spin_for(DISCOVERY_WAIT)

# =========================== 1. ノード =====================================
emit('SEC', 'Nodes')

live_nodes = set()
for name, namespace in node.get_node_names_and_namespaces():
    ns = namespace if namespace.endswith('/') else namespace + '/'
    live_nodes.add(ns + name)

for full_name, severity, hint in NODES:
    if full_name in live_nodes:
        emit('RES', 'OK', full_name, 'running')
    else:
        emit('RES', 'NG' if severity == 'error' else 'WARN', full_name,
             'Node %s was not found. %s' % (full_name, hint))

# =========================== 2. トピック ===================================
emit('SEC', 'Topics / rates (measuring %.1f s)' % DURATION)

graph_types = dict(node.get_topic_names_and_types())

targets = []   # (spec, 状態, 付加情報)
subs = []
stats = {}     # topic -> [count, first_t, last_t]

for spec in TOPICS:
    topic, expected_type, kind, expect_hz, min_hz, max_hz, severity, hint = spec

    if topic not in graph_types:
        targets.append((spec, 'absent', None))
        continue

    actual_types = graph_types[topic]
    if expected_type not in actual_types:
        targets.append((spec, 'type', '/'.join(actual_types)))
        continue

    pub_infos = node.get_publishers_info_by_topic(topic)
    if not pub_infos:
        targets.append((spec, 'nopub', None))
        continue

    durability = DurabilityPolicy.VOLATILE
    if all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in pub_infos):
        durability = DurabilityPolicy.TRANSIENT_LOCAL

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=durability,
    )

    stats[topic] = [0, None, None]

    def make_cb(key):
        def cb(_msg):
            s = stats[key]
            now = time.monotonic()
            s[0] += 1
            if s[1] is None:
                s[1] = now
            s[2] = now
        return cb

    try:
        msg_cls = get_message(expected_type)
    except Exception as exc:  # noqa: BLE001
        targets.append((spec, 'notype', str(exc)))
        continue

    subs.append(node.create_subscription(msg_cls, topic, make_cb(topic), qos, raw=True))

    # QoS 不整合 (BEST_EFFORT 出版 x RELIABLE 購読 / VOLATILE 出版 x TRANSIENT_LOCAL 購読)
    mismatch = []
    for s in node.get_subscriptions_info_by_topic(topic):
        if s.node_name == node.get_name():
            continue
        for p in pub_infos:
            if (p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                    and s.qos_profile.reliability == ReliabilityPolicy.RELIABLE):
                mismatch.append('reliability of %s is reliable (publisher is best_effort)' % s.node_name)
            if (p.qos_profile.durability == DurabilityPolicy.VOLATILE
                    and s.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL):
                mismatch.append('durability of %s is transient_local (publisher is volatile)' % s.node_name)

    targets.append((spec, 'measuring', sorted(set(mismatch))))

# ------------------------------------------------------------------
# ODrive の内部状態は同じ計測窓で拾う (温度・電圧は最大/最小、エラーは論理和)
# ------------------------------------------------------------------
ODRV_STATUS_TOPIC = '%s/odrive_status' % ODRV_NS
CTRL_STATUS_TOPIC = '%s/controller_status' % ODRV_NS

odrv = {
    'odrive_count': 0, 'ctrl_count': 0,
    'fet_max': None, 'motor_max': None,
    'vbus_min': None, 'vbus_max': None,
    'ibus_min': None, 'ibus_max': None,
    'errors': 0, 'disarm': 0, 'axis_errors': 0,
    'axis_state': None, 'procedure_result': None,
    'odrive_missing': None, 'ctrl_missing': None,
}


def _minmax(key_min, key_max, value):
    if odrv[key_min] is None or value < odrv[key_min]:
        odrv[key_min] = value
    if odrv[key_max] is None or value > odrv[key_max]:
        odrv[key_max] = value


def odrive_status_cb(msg):
    odrv['odrive_count'] += 1
    if odrv['fet_max'] is None or msg.fet_temperature > odrv['fet_max']:
        odrv['fet_max'] = msg.fet_temperature
    if odrv['motor_max'] is None or msg.motor_temperature > odrv['motor_max']:
        odrv['motor_max'] = msg.motor_temperature
    _minmax('vbus_min', 'vbus_max', msg.bus_voltage)
    _minmax('ibus_min', 'ibus_max', msg.bus_current)
    odrv['errors'] |= msg.active_errors
    odrv['disarm'] |= msg.disarm_reason


def controller_status_cb(msg):
    odrv['ctrl_count'] += 1
    odrv['axis_errors'] |= msg.active_errors
    odrv['axis_state'] = msg.axis_state
    odrv['procedure_result'] = msg.procedure_result


def subscribe_status(topic, type_name, callback, key):
    """状態トピックを購読する。購読できない理由は key に控えて後で報告する。"""
    if topic not in graph_types:
        odrv[key] = 'the topic does not exist'
        return
    if type_name not in graph_types[topic]:
        odrv[key] = 'the type is %s, not %s' % ('/'.join(graph_types[topic]), type_name)
        return
    if not node.get_publishers_info_by_topic(topic):
        odrv[key] = 'the topic has no publisher'
        return
    try:
        msg_cls = get_message(type_name)
    except Exception as exc:  # noqa: BLE001
        odrv[key] = 'failed to load %s (%s)' % (type_name, exc)
        return
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    subs.append(node.create_subscription(msg_cls, topic, callback, qos))


subscribe_status(ODRV_STATUS_TOPIC, 'odrive_can/msg/ODriveStatus',
                 odrive_status_cb, 'odrive_missing')
subscribe_status(CTRL_STATUS_TOPIC, 'odrive_can/msg/ControllerStatus',
                 controller_status_cb, 'ctrl_missing')

# --- 計測 ---
spin_for(DURATION)

for spec, state, extra in targets:
    topic, expected_type, kind, expect_hz, min_hz, max_hz, severity, hint = spec
    ng = 'NG' if severity == 'error' else 'WARN'

    if state == 'absent':
        emit('RES', ng, topic, 'Topic %s does not exist. %s' % (topic, hint))
        continue
    if state == 'type':
        emit('RES', ng, topic,
             'Unexpected message type (expected %s / actual %s). Check the message definition of the publisher.'
             % (expected_type, extra))
        continue
    if state == 'nopub':
        emit('RES', ng, topic, 'Topic %s has no publisher. Check that the publishing node is running.' % topic)
        continue
    if state == 'notype':
        emit('RES', ng, topic,
             'Failed to load message type %s (%s). Check that the msg package is built.' % (expected_type, extra))
        continue

    count, first_t, last_t = stats[topic]
    hz = 0.0
    if count >= 2 and last_t is not None and first_t is not None and last_t > first_t:
        hz = (count - 1) / (last_t - first_t)

    pub_n = node.count_publishers(topic)

    if count == 0:
        emit('RES', ng, topic, 'No data received on %s (publishers: %d). %s' % (topic, pub_n, hint))
        continue

    if kind == 'latched':
        emit('RES', 'OK', topic, 'latched: %d sample(s) (transient_local, pub=%d)' % (count, pub_n))
    elif kind == 'event':
        emit('RES', 'OK', topic, '%d msg(s) received / %.1f Hz (pub=%d)' % (count, hz, pub_n))
    else:
        lo = min_hz if min_hz is not None else expect_hz * 0.7
        hi = max_hz
        if hz < lo:
            emit('RES', ng, topic,
                 'Rate too low (measured %.1f Hz / expected %.1f Hz / minimum %.1f Hz). %s'
                 % (hz, expect_hz, lo, hint))
        elif hi is not None and hz > hi:
            emit('RES', 'WARN', topic,
                 'Rate too high (measured %.1f Hz / expected %.1f Hz / maximum %.1f Hz). Check that the publisher is not running twice.'
                 % (hz, expect_hz, hi))
        else:
            emit('RES', 'OK', topic, '%.1f Hz (expected %.1f Hz, pub=%d)' % (hz, expect_hz, pub_n))

    if extra:
        for m in extra:
            emit('RES', 'WARN', topic + ' (QoS)',
                 'QoS mismatch: %s. Align the QoS settings of publisher and subscriber.' % m)

# =========================== 3. ODrive =====================================
emit('SEC', 'ODrive (node_id %d, %s)' % (ODRV_NODE_ID, ODRV_NS))

CLEAR_HINT = ('After removing the cause, clear the errors with '
              '`ros2 service call %s/clear_errors std_srvs/srv/Empty {}`.' % ODRV_NS)

# --- ODriveStatus: エラー / 温度 / バス電圧・電流 ---
if odrv['odrive_count'] == 0:
    reason = odrv['odrive_missing'] or 'no message arrived within %.0f s' % DURATION
    emit('RES', 'NG', ODRV_STATUS_TOPIC,
         'ODrive errors, temperature and bus voltage cannot be checked because %s. '
         'odrive_can_node publishes ODriveStatus only after ALL of 0x%03X (errors), 0x%03X (temperature) '
         'and 0x%03X (bus voltage/current) have arrived, so a single disabled cyclic message silences the whole topic. '
         'Check the three /can_rx_* results above and enable the missing rates with odrivetool.'
         % (reason, ODRV_BASE | 0x03, ODRV_BASE | 0x15, ODRV_BASE | 0x17))
else:
    emit('RES', 'OK', ODRV_STATUS_TOPIC, '%d msg(s) in %.0f s' % (odrv['odrive_count'], DURATION))

    # ODrive 本体のエラー (計測期間中に一度でも立ったビットの論理和)
    names, hints = decode_errors(odrv['errors'])
    if not names:
        emit('RES', 'OK', 'ODrive active errors', 'none (0x00000000)')
    else:
        emit('RES', 'NG', 'ODrive active errors',
             'The ODrive reports active errors: %s. %s %s'
             % (', '.join(names), ' '.join(hints), CLEAR_HINT))

    # disarm_reason は最後に disarm したときの理由なので、過去の履歴でも残る
    names, hints = decode_errors(odrv['disarm'])
    if not names:
        emit('RES', 'OK', 'ODrive disarm reason', 'none (0x00000000)')
    else:
        emit('RES', 'WARN', 'ODrive disarm reason',
             'The ODrive disarmed at least once for: %s. This is the reason of the last disarm, so it can be a past event, '
             'but the axis will not enter CLOSED_LOOP_CONTROL until it is cleared. %s %s'
             % (', '.join(names), ' '.join(hints), CLEAR_HINT))

    # インバータ (FET) 温度
    fet = odrv['fet_max']
    if fet >= FET_NG:
        emit('RES', 'NG', 'ODrive FET temperature',
             'The inverter is too hot (max %.1f degC >= %.1f degC). Stop driving and let it cool down, improve the cooling, '
             'and lower the current limit. INVERTER_OVER_TEMP will disarm the axis.' % (fet, FET_NG))
    elif fet >= FET_WARN:
        emit('RES', 'WARN', 'ODrive FET temperature',
             'The inverter is getting hot (max %.1f degC >= %.1f degC). Watch the temperature during long runs.' % (fet, FET_WARN))
    else:
        emit('RES', 'OK', 'ODrive FET temperature',
             'max %.1f degC (warn %.1f / NG %.1f)' % (fet, FET_WARN, FET_NG))

    # モータ温度 (サーミスタ未設定だと 0.0 が返る)
    motor = odrv['motor_max']
    if motor >= MOTOR_NG:
        emit('RES', 'NG', 'ODrive motor temperature',
             'The motor is too hot (max %.1f degC >= %.1f degC). Stop driving and let it cool down. '
             'MOTOR_OVER_TEMP will disarm the axis.' % (motor, MOTOR_NG))
    elif motor >= MOTOR_WARN:
        emit('RES', 'WARN', 'ODrive motor temperature',
             'The motor is getting hot (max %.1f degC >= %.1f degC). Watch the temperature during long runs.' % (motor, MOTOR_WARN))
    elif motor <= 0.5:
        emit('RES', 'WARN', 'ODrive motor temperature',
             'The reported motor temperature is %.1f degC, which usually means the motor thermistor is not configured. '
             'Motor over-temperature protection is therefore inactive; enable motor_thermistor in the ODrive config or '
             'accept that only the FET temperature is monitored.' % motor)
    else:
        emit('RES', 'OK', 'ODrive motor temperature',
             'max %.1f degC (warn %.1f / NG %.1f)' % (motor, MOTOR_WARN, MOTOR_NG))

    # バス電圧 (負荷時の落ち込みを見るため最小値で判定する)
    vmin, vmax = odrv['vbus_min'], odrv['vbus_max']
    if vmin < VBUS_MIN:
        emit('RES', 'NG', 'ODrive bus voltage',
             'The bus voltage dropped to %.1f V (minimum %.1f V). Check the battery charge, the main switch and the power wiring. '
             'DC_BUS_UNDER_VOLTAGE will disarm the axis.' % (vmin, VBUS_MIN))
    elif vmax > VBUS_MAX:
        emit('RES', 'NG', 'ODrive bus voltage',
             'The bus voltage rose to %.1f V (maximum %.1f V). This is usually regenerative braking; check the brake resistor '
             'and dc_max_negative_current. DC_BUS_OVER_VOLTAGE will disarm the axis.' % (vmax, VBUS_MAX))
    else:
        emit('RES', 'OK', 'ODrive bus voltage',
             '%.1f - %.1f V (allowed %.1f - %.1f V)' % (vmin, vmax, VBUS_MIN, VBUS_MAX))

    # バス電流 (負値は回生)
    imin, imax = odrv['ibus_min'], odrv['ibus_max']
    ipeak = max(abs(imin), abs(imax))
    if ipeak > IBUS_WARN:
        emit('RES', 'WARN', 'ODrive bus current',
             'The bus current reached %.1f A (threshold %.1f A, range %.1f - %.1f A, negative means regeneration). '
             'Check the mechanical load and the current limit.' % (ipeak, IBUS_WARN, imin, imax))
    else:
        emit('RES', 'OK', 'ODrive bus current',
             '%.1f - %.1f A (peak %.1f A, threshold %.1f A)' % (imin, imax, ipeak, IBUS_WARN))

# --- ControllerStatus: heartbeat 由来の軸の状態 ---
if odrv['ctrl_count'] == 0:
    reason = odrv['ctrl_missing'] or 'no message arrived within %.0f s' % DURATION
    emit('RES', 'NG', CTRL_STATUS_TOPIC,
         'The axis state and the heartbeat errors cannot be checked because %s. '
         'odrive_can_node publishes ControllerStatus only after ALL of 0x%03X (heartbeat), 0x%03X (encoder), '
         '0x%03X (Iq) and 0x%03X (torques) have arrived. Check the four /can_rx_* results above.'
         % (reason, ODRV_BASE | 0x01, ODRV_BASE | 0x09, ODRV_BASE | 0x14, ODRV_BASE | 0x1C))
else:
    emit('RES', 'OK', CTRL_STATUS_TOPIC, '%d msg(s) in %.0f s' % (odrv['ctrl_count'], DURATION))

    names, hints = decode_errors(odrv['axis_errors'])
    if not names:
        emit('RES', 'OK', 'ODrive axis errors (heartbeat)', 'none (0x00000000)')
    else:
        emit('RES', 'NG', 'ODrive axis errors (heartbeat)',
             'The heartbeat reports active errors: %s. %s %s'
             % (', '.join(names), ' '.join(hints), CLEAR_HINT))

    state = odrv['axis_state']
    state_name = AXIS_STATES.get(state, 'UNKNOWN')
    if state == 8:
        emit('RES', 'OK', 'ODrive axis state', '%s (%d)' % (state_name, state))
    elif state == 1:
        emit('RES', 'WARN', 'ODrive axis state',
             'The axis is IDLE (1). This is normal before restart is sent from the gamepad; chassis_driver_node requests '
             'CLOSED_LOOP_CONTROL (8) when /restart arrives. If it stays IDLE while driving, the ODrive disarmed - '
             'check the disarm reason and the errors above.')
    else:
        emit('RES', 'WARN', 'ODrive axis state',
             'The axis is in %s (%d) instead of CLOSED_LOOP_CONTROL (8). Wait for the procedure to finish, then check '
             'the procedure result below.' % (state_name, state))

    result = odrv['procedure_result']
    result_name = PROCEDURE_RESULTS.get(result, 'UNKNOWN')
    if result == 0:
        emit('RES', 'OK', 'ODrive procedure result', '%s (%d)' % (result_name, result))
    elif result == 1:
        emit('RES', 'WARN', 'ODrive procedure result',
             'A procedure is still running (BUSY). Wait for calibration or the state transition to finish and check again.')
    else:
        emit('RES', 'NG', 'ODrive procedure result',
             'The last procedure ended with %s (%d). The axis will not enter CLOSED_LOOP_CONTROL until this is resolved; '
             'check the motor and encoder wiring and the calibration settings. %s' % (result_name, result, CLEAR_HINT))

executor.shutdown()
node.destroy_node()
rclpy.shutdown()
PY
}

dispatch_ros_checks() {
  local kind status name msg
  local produced=0
  while IFS=$'\t' read -r kind status name msg; do
    produced=1
    case "$kind" in
      SEC)  section "$status" ;;
      RES)  report "$status" "$name" "$msg" ;;
      INFO) info "$status" ;;
    esac
  done < <(run_ros_checks)

  if [[ "$produced" -eq 0 ]]; then
    section "Nodes / Topics / ODrive"
    report NG "rclpy check" "Failed to query the ROS 2 graph. Check that install/setup.bash is sourced and that python3-yaml is installed."
  fi
}

dispatch_ros_checks

# ---------------------------------------------------------------------------
# 5. CAN
# ---------------------------------------------------------------------------
section "CAN (${CAN_IF})"

check_can() {
  if ! command -v ip >/dev/null 2>&1; then
    report NG "ip command" "ip command not found. Check that iproute2 is installed."
    return
  fi

  local link
  if ! link="$(ip -details link show "$CAN_IF" 2>/dev/null)"; then
    report NG "interface ${CAN_IF}" "CAN interface ${CAN_IF} not found. Check that the USB-CAN adapter is connected, that the udev rule renaming it to ${CAN_IF} is installed (/etc/udev/rules.d), and that \`sudo ip link set ${CAN_IF} up type can bitrate 1000000\` was executed."
    info "Run \`ip -br link\` to list the interfaces that actually exist."
    return
  fi
  report OK "interface ${CAN_IF}" "present"

  if grep -q 'link/can' <<< "$link"; then
    report OK "interface type" "CAN"
  else
    report NG "interface type" "${CAN_IF} exists but is not a CAN device. Check the udev rule and the adapter."
    return
  fi

  if grep -qE '\bstate UP\b' <<< "$link"; then
    report OK "link state" "UP"
  else
    report NG "link state" "CAN interface ${CAN_IF} is not UP. Run \`sudo ip link set ${CAN_IF} up type can bitrate 1000000\`."
    return
  fi

  local bitrate
  bitrate="$(grep -oP '\bbitrate \K[0-9]+' <<< "$link" | head -n1)"
  if [[ -z "$bitrate" ]]; then
    report WARN "bitrate" "Failed to read the bitrate. Check that this is not a virtual CAN (vcan) and that the interface was brought up with a bitrate."
  elif (( bitrate == 1000000 )); then
    report OK "bitrate" "1000000 bps"
  else
    report WARN "bitrate" "Bitrate is ${bitrate} bps (expected 1000000 bps). It must match every board on the bus; if this value is intentional, ignore this warning."
  fi

  local can_state
  can_state="$(grep -oP 'can state \K[A-Z-]+' <<< "$link" | head -n1)"
  case "$can_state" in
    ERROR-ACTIVE)
      report OK "CAN controller state" "$can_state"
      ;;
    ERROR-WARNING|ERROR-PASSIVE)
      report NG "CAN controller state" "CAN controller is in an error state (${can_state}). Check the termination resistors (120 ohm at both ends), CAN_H / CAN_L wiring, and bitrate match."
      ;;
    BUS-OFF)
      report NG "CAN controller state" "CAN is BUS-OFF. Check for short circuits and bitrate mismatch, then recover with \`sudo ip link set ${CAN_IF} down && sudo ip link set ${CAN_IF} up type can bitrate ${bitrate:-1000000}\`."
      ;;
    "")
      report WARN "CAN controller state" "Failed to read the CAN controller state. This may be a virtual CAN (vcan)."
      ;;
    *)
      report WARN "CAN controller state" "Unexpected state (${can_state}). Check dmesg for driver errors."
      ;;
  esac

  local berr_tx berr_rx
  berr_tx="$(grep -oP 'berr-counter tx \K[0-9]+' <<< "$link" | head -n1)"
  berr_rx="$(grep -oP 'berr-counter tx [0-9]+ rx \K[0-9]+' <<< "$link" | head -n1)"
  if [[ -n "$berr_tx" && -n "$berr_rx" ]]; then
    if (( berr_tx == 0 && berr_rx == 0 )); then
      report OK "bus error counters" "tx=0 rx=0"
    else
      report NG "bus error counters" "CAN bus error counters are increasing (tx=${berr_tx} rx=${berr_rx}). Check the termination resistors, wiring, and noise countermeasures."
    fi
  fi

  # 500 Hz 送信では txqueuelen が小さいと ENOBUFS でフレーム落ちしやすい
  local qlen
  qlen="$(grep -oP '\bqlen \K[0-9]+' <<< "$link" | head -n1)"
  if [[ -n "$qlen" ]]; then
    if (( qlen >= 100 )); then
      report OK "txqueuelen" "$qlen"
    else
      report WARN "txqueuelen" "txqueuelen is ${qlen}. With 500 Hz TX a small queue can drop frames (ENOBUFS); consider \`sudo ip link set ${CAN_IF} txqueuelen 1000\`."
    fi
  fi

  # RX / TX パケットの増分でバスが生きているかを見る (can-utils 不要)
  local stats_before stats_after
  local rx0 tx0 rx1 tx1 rx_delta tx_delta
  stats_before="$(ip -s link show "$CAN_IF" 2>/dev/null)"
  rx0="$(awk '/RX:/{getline; print $2}' <<< "$stats_before")"
  tx0="$(awk '/TX:/{getline; print $2}' <<< "$stats_before")"
  sleep 1
  stats_after="$(ip -s link show "$CAN_IF" 2>/dev/null)"
  rx1="$(awk '/RX:/{getline; print $2}' <<< "$stats_after")"
  tx1="$(awk '/TX:/{getline; print $2}' <<< "$stats_after")"

  if [[ -n "$rx0" && -n "$rx1" ]]; then
    rx_delta=$((rx1 - rx0))
    if (( rx_delta > 0 )); then
      report OK "CAN RX" "${rx_delta} frames/s"
    else
      report NG "CAN RX" "No CAN frame received within 1 second. Check the power of the encoder boards and the ODrive, CAN wiring, and termination resistors."
    fi
  else
    report WARN "CAN RX" "Failed to read RX statistics. Check the output format of ip -s link show ${CAN_IF}."
  fi

  # chassis_driver は停止モード中も 0x210 を 500 Hz で送るため、TX はほぼ 500 f/s になるはず
  if [[ -n "$tx0" && -n "$tx1" ]]; then
    tx_delta=$((tx1 - tx0))
    if (( tx_delta == 0 )); then
      report NG "CAN TX" "No CAN frame sent within 1 second. Check that chassis_driver_node and socketcan_interface_node are running and that /can_tx is published."
    elif (( tx_delta < 250 )); then
      report WARN "CAN TX" "TX rate is low (${tx_delta} frames/s, expected ~500 from chassis_driver at interval_ms: 2). Check the CPU load and the socketcan TX queue."
    else
      report OK "CAN TX" "${tx_delta} frames/s (expected ~500)"
    fi
  else
    report WARN "CAN TX" "Failed to read TX statistics. Check the output format of ip -s link show ${CAN_IF}."
  fi

  # エラー / ドロップカウンタ (RX: bytes packets errors dropped ... / TX: 同様)
  local rx_err rx_drop tx_err tx_drop
  rx_err="$(awk '/RX:/{getline; print $3; exit}' <<< "$stats_after")"
  rx_drop="$(awk '/RX:/{getline; print $4; exit}' <<< "$stats_after")"
  tx_err="$(awk '/TX:/{getline; print $3; exit}' <<< "$stats_after")"
  tx_drop="$(awk '/TX:/{getline; print $4; exit}' <<< "$stats_after")"
  if [[ "$rx_err$rx_drop$tx_err$tx_drop" =~ ^[0-9]+$ ]]; then
    if (( rx_err + rx_drop + tx_err + tx_drop == 0 )); then
      report OK "error/drop counters" "rx errors=0 dropped=0 / tx errors=0 dropped=0"
    else
      report WARN "error/drop counters" "Frames are being lost (rx errors=${rx_err} dropped=${rx_drop} / tx errors=${tx_err} dropped=${tx_drop}). Check the wiring, termination resistors, bus load, and txqueuelen."
    fi
  fi

  # candump があれば実際に流れている CAN ID をフレーム数まで確認する
  if ! command -v candump >/dev/null 2>&1; then
    report WARN "candump" "candump not found. Install can-utils (\`sudo apt install can-utils\`) to check individual CAN IDs."
    return
  fi

  local dump ids
  dump="$(timeout 2 candump "$CAN_IF" 2>/dev/null)"
  ids="$(awk '{print $2}' <<< "$dump" | sort -u | tr '\n' ' ')"
  if [[ -z "${ids// /}" ]]; then
    report NG "received CAN IDs" "candump captured no frame in 2 seconds. Check the power of the devices on the bus and the termination resistors."
    return
  fi
  report OK "received CAN IDs" "$ids"

  # ID ごとのフレーム数 (2 秒間) を判定する
  #   count_id <hex_id> : dump 中の該当 ID のフレーム数を返す
  count_id() { awk -v id="$1" '$2==id{c++} END{print c+0}' <<< "$dump"; }

  # 0x210: chassis_driver -> MD の速度指令 (500 Hz)。速度指令入力から CAN 送信までの
  # 経路が生きていることを示す、DellPC の役目の最終出力
  local n210
  n210="$(count_id 210)"
  if (( n210 == 0 )); then
    report NG "CAN ID 0x210 (drive cmd to MD)" "Velocity command frames (0x210) are not on the bus. chassis_driver_node sends them at 500 Hz even in stop mode; check that chassis_driver_node and socketcan_interface_node are running on ${CAN_IF}."
  elif (( n210 < 500 )); then
    report WARN "CAN ID 0x210 (drive cmd to MD)" "TX rate is low (~$((n210 / 2)) frames/s, expected ~500). Check the CPU load and the socketcan TX queue (txqueuelen)."
  else
    report OK "CAN ID 0x210 (drive cmd to MD)" "~$((n210 / 2)) frames/s (expected ~500)"
  fi

  #   check_id <hex_id> <NG|WARN> <ラベル> <ヒント>
  check_id() {
    local id="$1" sev="$2" label="$3" hint="$4" n
    n="$(count_id "$id")"
    if (( n > 0 )); then
      report OK "$label" "$n frames in 2 s (~$((n / 2))/s)"
    else
      report "$sev" "$label" "$hint"
    fi
  }

  check_id 012 NG   "CAN ID 0x012 (caster steering enc)" "Caster steering angle encoder (0x012) not received. Check the encoder board power, CAN wiring, and termination resistors."
  check_id 013 NG   "CAN ID 0x013 (caster rotation enc)" "Caster rotation encoder (0x013) not received. Check the encoder board power, CAN wiring, and termination resistors."
  check_id 712 WARN "CAN ID 0x712 (emergency stop)" "Emergency stop board (0x712) not received. Check the board power and CAN wiring."

  # ODrive の周期メッセージ。1 つでも欠けると odrive_can_node が status を出版しないため、
  # 「どの周期メッセージが無効か」を CAN レベルで切り分けられるようにする。
  check_id "$ID_HEARTBEAT" NG "CAN ID 0x${ID_HEARTBEAT} (ODrive heartbeat)" \
    "ODrive heartbeat (0x${ID_HEARTBEAT}) not received. Check the ODrive power, the node_id setting (${ODRIVE_NODE_ID}), and bitrate match."
  check_id "$ID_ERROR" NG "CAN ID 0x${ID_ERROR} (ODrive errors)" \
    "ODrive error message (0x${ID_ERROR}) not received, so ODrive errors and disarm reasons cannot be read. Set axis0.config.can.error_msg_rate_ms to a non-zero value (e.g. 100) with odrivetool and run save_configuration()."
  check_id "$ID_ENCODER" WARN "CAN ID 0x${ID_ENCODER} (ODrive encoder est.)" \
    "ODrive encoder estimates (0x${ID_ENCODER}) not received. Set axis0.config.can.encoder_msg_rate_ms to a non-zero value."
  check_id "$ID_IQ" WARN "CAN ID 0x${ID_IQ} (ODrive Iq)" \
    "ODrive Iq message (0x${ID_IQ}) not received. Set axis0.config.can.iq_msg_rate_ms to a non-zero value."
  check_id "$ID_TEMP" NG "CAN ID 0x${ID_TEMP} (ODrive temperature)" \
    "ODrive temperature message (0x${ID_TEMP}) not received, so FET and motor temperature cannot be monitored. Set axis0.config.can.temperature_msg_rate_ms to a non-zero value (e.g. 100) and run save_configuration()."
  check_id "$ID_BUS_VI" NG "CAN ID 0x${ID_BUS_VI} (ODrive bus V/I)" \
    "ODrive bus voltage/current message (0x${ID_BUS_VI}) not received. Set axis0.config.can.bus_vi_msg_rate_ms to a non-zero value (e.g. 100) and run save_configuration()."
  check_id "$ID_TORQUE" WARN "CAN ID 0x${ID_TORQUE} (ODrive torques)" \
    "ODrive torque message (0x${ID_TORQUE}) not received. Set axis0.config.can.torque_msg_rate_ms to a non-zero value."
}

check_can

# ---------------------------------------------------------------------------
# 6. Jetson (上位機) との接続
# ---------------------------------------------------------------------------
section "${PEER_NAME} connection (${PEER_IP})"

check_network() {
  local subnet="${PEER_IP%.*}."
  local local_addrs
  local_addrs="$(ip -4 -brief addr show 2>/dev/null | awk '{for(i=3;i<=NF;i++) print $1" "$i}')"

  if grep -qF " ${OWN_IP}/" <<< "$local_addrs"; then
    report OK "own IP" "${OWN_IP} (${OWN_NAME})"
  else
    local own_addr
    own_addr="$(grep -F " ${subnet}" <<< "$local_addrs" | head -n1)"
    report NG "own IP" "This machine does not have ${OWN_IP} (current: ${own_addr:-no address in ${subnet}0/24}). Check that the static IP of the ${OWN_NAME} is ${OWN_IP}/24."
  fi

  if ping -c 3 -W 1 "$PEER_IP" >/dev/null 2>&1; then
    local rtt
    rtt="$(ping -c 3 -W 1 "$PEER_IP" 2>/dev/null | tail -n1 | awk -F'/' '{print $5}')"
    report OK "ping ${PEER_IP}" "avg RTT ${rtt:-?} ms to ${PEER_NAME}"
  else
    report NG "ping ${PEER_IP}" "Failed to reach the ${PEER_NAME} (${PEER_IP}). Check the LAN cable, the power of the ${PEER_NAME}, and its static IP (${PEER_IP}/24)."
    info "Also check that both machines are in the same subnet (${subnet}0/24)."
    return
  fi

  local neigh
  neigh="$(ip neigh show "$PEER_IP" 2>/dev/null | head -n1)"
  if [[ -n "$neigh" ]]; then
    report OK "ARP entry" "$neigh"
  else
    report WARN "ARP entry" "Failed to get the ARP entry. L2 reachability may be unstable."
  fi

  if timeout 2 bash -c "echo > /dev/tcp/${PEER_IP}/22" 2>/dev/null; then
    report OK "SSH (22/tcp)" "reachable"
  else
    report WARN "SSH (22/tcp)" "Failed to connect to the SSH port of the ${PEER_NAME}. If you need remote log access, check that sshd is running and check the firewall."
  fi

  # マルチキャストが通らないと DDS のディスカバリが成立しない
  if ping -c 2 -W 1 224.0.0.1 2>/dev/null | grep -q "$PEER_IP"; then
    report OK "multicast" "response from the ${PEER_NAME} (DDS discovery possible)"
  else
    report WARN "multicast" "No multicast (224.0.0.1) response from the ${PEER_NAME}. ROS 2 discovery relies on multicast; check IGMP snooping on the switch and the firewall."
  fi

  report OK "ROS_DOMAIN_ID consistency" "This machine uses ${ROS_DOMAIN_ID:-0}. Check that the ${PEER_NAME} uses the same value."
}

check_network

# ---------------------------------------------------------------------------
# サマリ
# ---------------------------------------------------------------------------
TOTAL=$((OK_COUNT + NG_COUNT + WARN_COUNT + SKIP_COUNT))

printf "\n%s===== Summary =====%s\n" "$C_SEC" "$C_OFF"
printf "  Total %d checks : %sOK %d%s / %sNG %d%s / %sWARN %d%s / %sSKIP %d%s\n\n" \
  "$TOTAL" \
  "$C_OK" "$OK_COUNT" "$C_OFF" \
  "$C_NG" "$NG_COUNT" "$C_OFF" \
  "$C_WARN" "$WARN_COUNT" "$C_OFF" \
  "$C_SKIP" "$SKIP_COUNT" "$C_OFF"

if (( NG_COUNT > 0 )); then
  printf "  %sThere are %d NG items. See the -> hints above.%s\n\n" "$C_NG" "$NG_COUNT" "$C_OFF"
  exit 1
fi

if (( WARN_COUNT > 0 )); then
  printf "  %sNo fatal NG. Depending on the situation, the %d WARN item(s) may be normal.%s\n\n" "$C_WARN" "$WARN_COUNT" "$C_OFF"
fi

exit 0
