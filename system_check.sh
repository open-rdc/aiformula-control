#!/usr/bin/env bash
#
# aiformula 実機システム状態チェック
#
# main_exec.launch.py を起動した状態で実行し、以下をまとめて確認する。
#
#   1. ROS 2 実行環境
#   2. ノードの生存
#   3. トピックの出版状況と周波数
#   4. TF
#   5. CAN バス
#   6. もう一方のマシン (Jetson / DellPC) との接続
#
# 実機専用。main_params.yaml の launch.sim が true のときは非対応として終了する。
#
# ROLE="dell" のとき (aiformula-control リポジトリで動く DellPC 向け):
#   - main_executor が無いため main_params.yaml のパラメータ確認をスキップする
#   - 日本語ロケールが無いためログはすべて英語で出力する
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

# 実行ロール
#   "jetson" : 知覚 / 自己位置 / 計画 / TF を確認する
#   "dell"   : chassis_driver / CAN / ODrive を確認する
readonly ROLE="dell"

# 各マシンの固定 IP
readonly JETSON_IP="192.168.10.10"
readonly DELL_PC_IP="192.168.10.20"

# 周波数の計測時間 [秒]
readonly MEASURE_DURATION=10.0

# ロールから自機と相手機を決める
if [[ "$ROLE" == "dell" ]]; then
  readonly OWN_NAME="DellPC"  OWN_IP="$DELL_PC_IP"
  readonly PEER_NAME="Jetson" PEER_IP="$JETSON_IP"
else
  readonly OWN_NAME="Jetson"  OWN_IP="$JETSON_IP"
  readonly PEER_NAME="DellPC" PEER_IP="$DELL_PC_IP"
fi

# ROLE=dell のログは英語で出す (DellPC は日本語ロケール非対応のため)
#   t <日本語> <英語> : ロールに応じた文字列を返す
t() {
  if [[ "$ROLE" == "dell" ]]; then printf '%s' "$2"; else printf '%s' "$1"; fi
}

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

# 端末上の表示幅を数える (日本語などの全角文字は 2 桁として扱う)
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
#   それ以外: 「〇〇に失敗しました。××を確認してください。」を次行にインデントして出す
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
    [[ -n "$msg" ]] && printf "       %s→ %s%s\n" "$color" "$msg" "$C_OFF"
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
section "$(t "実行環境" "Environment")"

if ! command -v ros2 >/dev/null 2>&1; then
  report NG "$(t "ros2 コマンド" "ros2 command")" "$(t "ros2 コマンドの検出に失敗しました。/opt/ros/humble/setup.bash と install/setup.bash を source したか確認してください。" "ros2 command not found. Check that /opt/ros/humble/setup.bash and install/setup.bash are sourced.")"
  fatal "$(t "ROS 2 環境が読み込まれていないため中断します。" "Aborting because the ROS 2 environment is not loaded.")"
fi
report OK "$(t "ros2 コマンド" "ros2 command")" "$(command -v ros2)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  report NG "ROS_DISTRO" "$(t "ROS_DISTRO の取得に失敗しました。/opt/ros/humble/setup.bash を source したか確認してください。" "ROS_DISTRO is not set. Check that /opt/ros/humble/setup.bash is sourced.")"
else
  report OK "ROS_DISTRO" "$ROS_DISTRO"
fi

report OK "ROS_DOMAIN_ID" "$(t "${ROS_DOMAIN_ID:-0 (未設定)} ※Jetson と DellPC で一致させること" "${ROS_DOMAIN_ID:-0 (unset)} (must match between Jetson and DellPC)")"
report OK "RMW_IMPLEMENTATION" "$(t "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (既定)} ※2 台で一致させること" "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (default)} (must match on both machines)")"
if [[ "$ROLE" == "dell" ]]; then
  report OK "role" "dell (chassis control PC, aiformula-control)"
else
  report OK "実行ロール" "jetson (高レイヤ制御機)"
fi

# パラメータ既定値 (dell はパラメータ確認をしないため既定値をそのまま使う)
#   CAN_IF は aiformula-control の chassis_driver_params.yaml (if_name: "can_main") に合わせる
SIM="false"; JOY="false"; CAN_IF="can_main"

if [[ "$ROLE" == "dell" ]]; then
  # aiformula-control リポジトリには main_executor が無いため、パラメータ確認は行わない
  report SKIP "main_params.yaml" "Parameter check is skipped on the DellPC (the aiformula-control repository has no main_executor)."
else
  # main_params.yaml を探す (install 側を優先。--symlink-install なら src と同一実体)
  #   2 台分離後に実行機パッケージが main_executor 以外になっても拾えるよう、
  #   install / src 配下の総当たりまで含めて探す。マッチしないグロブは -f で弾かれる
  PARAMS_YAML=""
  for candidate in \
    "${SCRIPT_DIR}/../install/main_executor/share/main_executor/config/main_params.yaml" \
    "${SCRIPT_DIR}/main_executor/config/main_params.yaml" \
    "${SCRIPT_DIR}"/../install/*/share/*/config/main_params.yaml \
    "${SCRIPT_DIR}"/*/config/main_params.yaml
  do
    if [[ -f "$candidate" ]]; then
      PARAMS_YAML="$(realpath "$candidate" 2>/dev/null || echo "$candidate")"
      break
    fi
  done

  if [[ -z "$PARAMS_YAML" ]]; then
    report NG "main_params.yaml" "main_params.yaml の読み込みに失敗しました。src/main_executor/config/main_params.yaml が存在するか確認してください。"
    fatal "パラメータファイルが見つからないため中断します。"
  fi
  report OK "main_params.yaml" "$PARAMS_YAML"

  # launch セクションと CAN インターフェース名を取り出す
  PARAM_DUMP="$(python3 - "$PARAMS_YAML" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as f:
    doc = yaml.safe_load(f) or {}

launch = (doc.get('launch') or {}).get('ros__parameters', {}) or {}
can = (doc.get('socketcan_interface_node') or {}).get('ros__parameters', {}) or {}

print('SIM=%s' % ('true' if launch.get('sim', False) else 'false'))
print('JOY=%s' % ('true' if launch.get('joy', False) else 'false'))
print('CAN_IF=%s' % (can.get('if_name', 'can_main')))
PY
)"

  if [[ $? -ne 0 || -z "$PARAM_DUMP" ]]; then
    report NG "main_params.yaml の解析" "main_params.yaml の解析に失敗しました。YAML の書式と python3-yaml の導入を確認してください。"
    fatal "パラメータを解釈できないため中断します。"
  fi

  while IFS='=' read -r key value; do
    case "$key" in
      SIM)    SIM="$value" ;;
      JOY)    JOY="$value" ;;
      CAN_IF) CAN_IF="$value" ;;
    esac
  done <<< "$PARAM_DUMP"

  if [[ "$SIM" == "true" ]]; then
    report NG "launch.sim" "main_params.yaml の sim が true です。サポートしていません。"
    info "このスクリプトは実機構成 (sim: false) 専用です。"
    info "${PARAMS_YAML} の launch: セクションで sim: false に変更してから実行してください。"
    printf "\n"
    exit 1
  fi
  report OK "launch.sim" "false (実機構成)"
  report OK "launch.joy" "$JOY"
  report OK "CAN インターフェース名" "$CAN_IF"
fi

# ---------------------------------------------------------------------------
# 1〜4. ノード / トピック / 周波数 / TF (rclpy でまとめて計測)
# ---------------------------------------------------------------------------
run_ros_checks() {
  python3 - "$MEASURE_DURATION" "$ROLE" "$JOY" <<'PY' 2>/dev/null
import sys
import time

DURATION = float(sys.argv[1])
ROLE = sys.argv[2]
JOY_ENABLED = sys.argv[3] == 'true'

DISCOVERY_WAIT = 2.0

JA = ROLE != 'dell'  # dell は日本語ロケール非対応のため英語で出力する


def tr(ja, en):
    return ja if JA else en


def hint_text(h):
    """ヒントは str (単一言語) または {'ja': ..., 'en': ...} の辞書。"""
    if isinstance(h, dict):
        return h['ja' if JA else 'en']
    return h


try:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer, TransformListener
except Exception as exc:  # noqa: BLE001
    print('RES\tNG\t%s\t%s' % (
        tr('rclpy のインポート', 'rclpy import'),
        tr('rclpy の読み込みに失敗しました (%s)。install/setup.bash を source したか確認してください。',
           'Failed to import rclpy (%s). Check that install/setup.bash is sourced.') % exc))
    sys.exit(1)


def emit(kind, *fields):
    print('\t'.join([kind] + [str(f) for f in fields]), flush=True)


# dell は足回り制御のみ担当 (速度指令の入力 → CAN で MD へ送信まで)。
# 上層ソフトウェアの項目は "This is jetson role" として飛ばす。
SKIP_REASON = ('This is jetson role'
               if ROLE == 'dell'
               else 'DellPC 側の担当です。DellPC 上のスクリプトで ROLE="dell" にして確認してください。')


def want(layer):
    if ROLE == 'dell':
        return layer in ('low', 'both')
    return layer in ('high', 'both')


# (フルノード名, レイヤ, 深刻度, ヒント)
#   ヒントの言語: high 層は Jetson (日本語) でしか表示されないため日本語、
#   low 層は DellPC (英語) でしか表示されないため英語で書く。
#   both 層は両方で表示されるため {'ja': ..., 'en': ...} の辞書にする。
NODES = [
    ('/controller_node',           'high', 'error', 'main_exec が起動しているか、colcon build --packages-up-to main_executor が通っているか確認してください。'),
    ('/vectormap_server_node',     'high', 'error', 'main_exec のログに map_path の読み込みエラーが出ていないか確認してください。'),
    ('/lane_line_publisher_node',  'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/vectormap_visualizer_node', 'high', 'warn',  'main_exec が起動しているか確認してください。'),
    ('/pose_estimater_node',       'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/ekf_localizer_node',        'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/odom_tf_node',              'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/map_odom_tf_node',          'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/mission_planner_node',      'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/local_planner_server_node', 'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/controller_server_node',    'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/zed_wrapper_node',          'high', 'error', 'ZED SDK が導入されているか (未検出だと zed_wrapper はビルドをスキップします)、ZED カメラが USB3 で接続されているか確認してください。'),
    ('/road_detector_node',        'high', 'error', 'road_detector (rclpy) が起動しているか、colcon build --packages-select road_detector が通っているか確認してください。'),
    ('/vectornav',                 'high', 'error', 'vectornav が起動しているか、VN-100 が /dev/ttyUSB0 に見えて読み書き権限があるか確認してください。'),
    ('/vn_sensor_msgs',            'high', 'error', 'vectornav の launch が読み込まれているか確認してください。'),
    ('/robot_state_publisher',     'high', 'error', 'robot_state_publisher が起動しているか、URDF (simulator/models/ai_car1/model.urdf) が読めているか確認してください。'),
    ('/chassis_driver_node',       'low',  'error', 'Check that the executor including chassis_driver is running on the DellPC.'),
    ('/socketcan_interface_node',  'low',  'error', 'Check that socketcan_interface_node is running and that binding to the CAN interface did not fail (see the startup log).'),
    ('/odrive_axis0/can_node',     'low',  'error', 'Check that odrive_can_launch.yaml is loaded on the DellPC.'),
]

if JOY_ENABLED:
    NODES.insert(0, ('/joy_node', 'high', 'error',
                     'joy パッケージが導入されているか、コントローラが /dev/input/js0 に接続されているか確認してください。'))

# (トピック, 型, 種別, 期待Hz, 下限Hz, 上限Hz, レイヤ, 深刻度, ヒント)
#   種別 rate   : 周波数まで判定する
#   種別 event  : 到着したことだけを判定する (発生契機が不定のもの)
#   種別 latched: transient_local。1 サンプル受け取れれば良い
TOPICS = [
    # --- センサ ---
    ('/vectornav/imu', 'sensor_msgs/msg/Imu', 'rate', 20.0, 14.0, None, 'high', 'error',
     'IMU データの受信に失敗しました。vectornav が起動しているか、vectornav.yaml の AsyncDataOutputFrequency (20 Hz) と /dev/ttyUSB0 の接続を確認してください。'),
    ('/vectornav/gnss', 'sensor_msgs/msg/NavSatFix', 'rate', 20.0, 14.0, None, 'high', 'error',
     'GNSS データの受信に失敗しました。vectornav が起動しているか、GNSS アンテナが接続され屋外で測位できているか確認してください。'),
    # 2 台構成では Jetson が出版し DellPC の chassis_driver も購読する
    ('/vectornav/velocity_body', 'geometry_msgs/msg/TwistWithCovarianceStamped', 'rate', 20.0, 14.0, None, 'both', 'error',
     {'ja': '車体速度の受信に失敗しました。vectornav の INS グループ出力が有効か、vn_sensor_msgs が起動しているか確認してください。2 台構成では Jetson 側で出版されるため、DellPC で見えない場合は 2 台間の DDS 疎通を確認してください。',
      'en': 'Failed to receive body velocity. This topic is published by the Jetson; check that vectornav and vn_sensor_msgs are running there, and check the DDS connectivity between the two machines.'}),
    ('/joy', 'sensor_msgs/msg/Joy', 'event', None, None, None, 'high', 'error',
     'コントローラ入力の受信に失敗しました。joy_node が起動しているか、コントローラが /dev/input/js0 として認識されているか確認してください。'),

    # --- カメラ ---
    ('/zed/zed_node/rgb/image_rect_color', 'sensor_msgs/msg/Image', 'rate', 15.0, 10.0, None, 'high', 'error',
     'カメラ画像の受信に失敗しました。ZED SDK が導入され zed_wrapper が ENABLE_ZED 付きでビルドされているか、カメラが USB3 ポートに接続されているか確認してください。'),
    ('/zed/zed_node/point_cloud', 'sensor_msgs/msg/PointCloud2', 'rate', 15.0, 10.0, None, 'high', 'warn',
     '点群の受信に失敗しました。zed_wrapper_node の depth mode 設定と GPU メモリの空きを確認してください。'),

    # --- 知覚 ---
    ('/perception/lane_mask', 'sensor_msgs/msg/Image', 'rate', 15.0, 8.0, None, 'high', 'error',
     '白線マスクの受信に失敗しました。road_detector_node が起動しているか、入力 /zed/zed_node/rgb/image_rect_color が出ているか、GPU 推論でエラーが出ていないか確認してください。'),
    ('/perception/lane_mask_visualize', 'sensor_msgs/msg/Image', 'rate', 15.0, 8.0, None, 'high', 'warn',
     '白線マスク可視化の受信に失敗しました。road_detector_node のログを確認してください。'),
    ('/perception/lane_line_points', 'sensor_msgs/msg/PointCloud2', 'rate', 15.0, 8.0, None, 'high', 'error',
     '白線点群の受信に失敗しました。lane_line_publisher_node が /perception/lane_mask を受け取れているか、mask_threshold (既定 128) が高すぎないか確認してください。'),
    ('/perception/lane_line', 'visualization_msgs/msg/MarkerArray', 'rate', 15.0, 8.0, None, 'high', 'warn',
     '白線マーカの受信に失敗しました。lane_line_publisher_node のログを確認してください。'),
    ('/perception/vectormap_visualize', 'sensor_msgs/msg/Image', 'rate', 15.0, 8.0, None, 'high', 'warn',
     'ベクターマップ重畳画像の受信に失敗しました。vectormap_visualizer_node が TF base_link->map を引けているか確認してください。'),
    ('/perception/objects', 'object_detection_msgs/msg/ObjectInfoArray', 'event', None, None, None, 'high', 'warn',
     '障害物情報の受信に失敗しました。object_detector_node は main_executor/src/main.cpp でコメントアウトされています。障害物回避を使う場合は有効化してください。'),

    # --- 地図 ---
    ('/vector_map', 'vectormap_msgs/msg/VectorMap', 'latched', None, None, None, 'high', 'error',
     'ベクターマップの受信に失敗しました。vectormap_server_node が起動しているか、main_params.yaml の map_path (aiformula_course.osm) が map パッケージに存在するか確認してください。'),
    ('/vector_map/visualize', 'visualization_msgs/msg/MarkerArray', 'latched', None, None, None, 'high', 'warn',
     'ベクターマップ可視化の受信に失敗しました。vectormap_server_node のログを確認してください。'),

    # --- 自己位置 ---
    ('/localization/pf_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', 'rate', 50.0, 35.0, None, 'high', 'error',
     'パーティクルフィルタ推定値の受信に失敗しました。pose_estimater_node が /vector_map と /perception/lane_line_points を受け取れているか確認してください。'),
    ('/localization/particle', 'geometry_msgs/msg/PoseArray', 'rate', 50.0, 35.0, None, 'high', 'warn',
     'パーティクル分布の受信に失敗しました。pose_estimater_node のログを確認してください。'),
    ('/localization/pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', 'rate', 50.0, 35.0, None, 'high', 'error',
     'EKF 推定姿勢の受信に失敗しました。ekf_localizer_node が /localization/pf_pose と /vectornav/velocity_body を受け取れているか確認してください。'),
    ('/localization/odom', 'nav_msgs/msg/Odometry', 'rate', 50.0, 35.0, None, 'high', 'error',
     'オドメトリの受信に失敗しました。odom_tf_node が /vectornav/imu と /vectornav/velocity_body を受け取れているか確認してください。'),

    # --- 計画 ---
    ('/planner/global_path', 'nav_msgs/msg/Path', 'rate', 10.0, 7.0, None, 'high', 'error',
     'グローバル経路の受信に失敗しました。mission_planner_node が /vector_map と /localization/pose を受け取れているか、自車が経路上のレーンレット近傍にいるか確認してください。'),
    ('/planner/local_path', 'nav_msgs/msg/Path', 'rate', 10.0, 7.0, None, 'high', 'error',
     'ローカル経路の受信に失敗しました。local_planner_server_node が /planner/global_path を受け取れているか、local_planner_plugin の名前が正しいか確認してください。'),

    # --- 制御 ---
    ('/cmd_vel', 'steered_drive_msg/msg/SteeredDrive', 'rate', 20.0, 14.0, None, 'both', 'error',
     {'ja': '速度指令の受信に失敗しました。controller_server_node が起動しているか、controller_plugin の名前が正しいか確認してください。',
      'en': 'Failed to receive velocity commands. This topic is published by the Jetson; check that controller_server_node is running there and check the DDS connectivity between the two machines.'}),
    ('/autonomous', 'std_msgs/msg/Bool', 'event', None, None, None, 'high', 'warn',
     '自律走行フラグの受信に失敗しました。コントローラの Share ボタンで自律走行に切り替えたか確認してください。'),
    ('/planning/nav_cmd', 'std_msgs/msg/String', 'event', None, None, None, 'high', 'warn',
     '進路指令の受信に失敗しました。controller_node がコントローラ入力を受け取れているか確認してください。'),
    # 2 台構成では Jetson の controller_node が出版し DellPC の chassis_driver が購読する
    ('/restart', 'std_msgs/msg/Empty', 'event', None, None, None, 'both', 'warn',
     {'ja': '再起動指令の受信に失敗しました。コントローラで再起動を送るまでは出版されないため、走行前であれば正常です。DellPC 側で見えない場合は 2 台間の DDS 疎通を確認してください。',
      'en': 'Failed to receive the restart command. It is not published until restart is sent from the gamepad, so this is normal before driving. If it never arrives, check the DDS connectivity between the two machines.'}),
    # 2 台構成では DellPC が出版し Jetson の controller_server が購読する
    ('/caster_data', 'std_msgs/msg/Float64MultiArray', 'rate', 500.0, 250.0, None, 'both', 'warn',
     {'ja': 'キャスタ状態の受信に失敗しました。chassis_driver_node は停止モード中は出版しません。コントローラで再起動 (restart) を送り、/cmd_vel が届いているか確認してください。Jetson 側で見えない場合は 2 台間の DDS 疎通も確認してください。',
      'en': 'Failed to receive caster states. chassis_driver_node does not publish while in stop mode. Send restart from the gamepad and check that /cmd_vel is arriving.'}),
    ('/caster_odom', 'nav_msgs/msg/Odometry', 'rate', 500.0, 250.0, None, 'low', 'warn',
     'Failed to receive caster odometry. chassis_driver_node does not publish it until the caster rotation encoder (/can_rx_013) is received at least once. Check the CAN wiring. Note: nothing currently subscribes to this topic.'),

    # --- CAN トピック ---
    ('/can_tx', 'socketcan_interface_msg/msg/SocketcanIF', 'rate', 500.0, 250.0, None, 'low', 'error',
     'Failed to receive CAN TX frames. Check that chassis_driver_node is running (interval_ms: 2 = 500 Hz) and check the executor log.'),
    ('/can_rx_012', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'error',
     'Failed to receive the caster steering angle encoder (CAN ID 0x012). socketcan_interface_node only creates topics for IDs it has received. Check the encoder board power, CAN wiring, and termination resistors.'),
    ('/can_rx_013', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'error',
     'Failed to receive the caster rotation encoder (CAN ID 0x013). Check the encoder board power, CAN wiring, and termination resistors.'),
    ('/can_rx_712', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'warn',
     'Failed to receive the emergency stop board (CAN ID 0x712). Check the board power and CAN wiring.'),
    ('/can_rx_301', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'error',
     'Failed to receive the ODrive heartbeat (CAN ID 0x301 = node_id 24). Check the ODrive power, node_id setting (24), and that the CAN bitrate matches.'),
    ('/can_rx_309', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'warn',
     'Failed to receive ODrive encoder estimates (CAN ID 0x309). Check the ODrive cyclic message settings (encoder_estimates rate).'),
    ('/odrive_axis0/control_message', 'odrive_can/msg/ControlMessage', 'rate', 500.0, 250.0, None, 'low', 'warn',
     'Failed to receive ODrive control commands. chassis_driver_node does not publish while in stop mode. Check that restart was sent from the gamepad.'),
    ('/odrive_axis0/controller_status', 'odrive_can/msg/ControllerStatus', 'event', None, None, None, 'low', 'warn',
     'Failed to receive ODrive controller status. Check that /can_rx_301 and /can_rx_309 are arriving.'),
    ('/odrive_axis0/odrive_status', 'odrive_can/msg/ODriveStatus', 'event', None, None, None, 'low', 'warn',
     'Failed to receive ODrive status. Check that /can_rx_303 (errors) and /can_rx_317 (bus voltage) are arriving.'),
]

# (親, 子, 静的か, 許容遅延[s], ヒント)
TF_PAIRS = [
    ('map', 'odom', False, 0.5,
     'map->odom の取得に失敗しました。map_odom_tf_node が /localization/pose を受け取れているか、odom->base_link が先に出ているか確認してください。'),
    ('odom', 'base_link', False, 0.2,
     'odom->base_link の取得に失敗しました。odom_tf_node が /vectornav/imu と /vectornav/velocity_body を受け取れているか確認してください。'),
    ('map', 'base_link', False, 0.5,
     'map->base_link の取得に失敗しました。map->odom と odom->base_link のどちらかが欠けています。上 2 項目の結果を確認してください。'),
    ('base_link', 'camera_link', True, None,
     'base_link->camera_link (静的 TF) の取得に失敗しました。robot_state_publisher が起動しているか、URDF に camera_joint があるか確認してください。'),
    ('base_link', 'imu_link', True, None,
     'base_link->imu_link (静的 TF) の取得に失敗しました。robot_state_publisher が起動しているか確認してください。'),
    ('base_link', 'gps_link', True, None,
     'base_link->gps_link (静的 TF) の取得に失敗しました。robot_state_publisher が起動しているか確認してください。'),
]

REL_NAME = {ReliabilityPolicy.RELIABLE: 'reliable', ReliabilityPolicy.BEST_EFFORT: 'best_effort'}
DUR_NAME = {DurabilityPolicy.VOLATILE: 'volatile', DurabilityPolicy.TRANSIENT_LOCAL: 'transient_local'}

rclpy.init(args=None)
node = rclpy.create_node('aiformula_system_check')
executor = SingleThreadedExecutor()
executor.add_node(node)

tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
tf_listener = TransformListener(tf_buffer, node, spin_thread=False)


def spin_for(seconds):
    end = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)


# --- ディスカバリ待ち ---
spin_for(DISCOVERY_WAIT)

# =========================== 1. ノード =====================================
emit('SEC', tr('ノード', 'Nodes'))

live_nodes = set()
for name, namespace in node.get_node_names_and_namespaces():
    ns = namespace if namespace.endswith('/') else namespace + '/'
    live_nodes.add(ns + name)

for full_name, layer, severity, hint in NODES:
    if not want(layer):
        emit('RES', 'SKIP', full_name, SKIP_REASON)
        continue
    if full_name in live_nodes:
        emit('RES', 'OK', full_name, tr('起動中', 'running'))
    else:
        emit('RES', 'NG' if severity == 'error' else 'WARN', full_name,
             tr('%s の検出に失敗しました。%s', 'Node %s was not found. %s') % (full_name, hint_text(hint)))

# =========================== 2. トピック ===================================
emit('SEC', tr('トピック / 周波数 (計測 %.1f 秒)', 'Topics / rates (measuring %.1f s)') % DURATION)

graph_types = dict(node.get_topic_names_and_types())

targets = []   # (spec, 状態)
subs = []
stats = {}     # topic -> [count, first_t, last_t]

for spec in TOPICS:
    topic, expected_type, kind, expect_hz, min_hz, max_hz, layer, severity, hint = spec

    if not want(layer):
        targets.append((spec, 'skip', SKIP_REASON))
        continue

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
                mismatch.append(tr('%s の reliability が reliable (出版側は best_effort)',
                                   'reliability of %s is reliable (publisher is best_effort)') % s.node_name)
            if (p.qos_profile.durability == DurabilityPolicy.VOLATILE
                    and s.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL):
                mismatch.append(tr('%s の durability が transient_local (出版側は volatile)',
                                   'durability of %s is transient_local (publisher is volatile)') % s.node_name)

    targets.append((spec, 'measuring', sorted(set(mismatch))))

# --- 計測 ---
spin_for(DURATION)

for spec, state, extra in targets:
    topic, expected_type, kind, expect_hz, min_hz, max_hz, layer, severity, hint = spec
    ng = 'NG' if severity == 'error' else 'WARN'

    if state == 'skip':
        emit('RES', 'SKIP', topic, extra)
        continue
    if state == 'absent':
        emit('RES', ng, topic, tr('トピック %s が存在しません。%s',
                                  'Topic %s does not exist. %s') % (topic, hint_text(hint)))
        continue
    if state == 'type':
        emit('RES', ng, topic, tr('型が期待と異なります (期待 %s / 実際 %s)。出版ノードのメッセージ定義を確認してください。',
                                  'Unexpected message type (expected %s / actual %s). Check the message definition of the publisher.') % (expected_type, extra))
        continue
    if state == 'nopub':
        emit('RES', ng, topic, tr('%s の出版者が 0 です。出版ノードが起動しているか確認してください。',
                                  'Topic %s has no publisher. Check that the publishing node is running.') % topic)
        continue
    if state == 'notype':
        emit('RES', ng, topic, tr('メッセージ型 %s の読み込みに失敗しました (%s)。該当 msg パッケージがビルド済みか確認してください。',
                                  'Failed to load message type %s (%s). Check that the msg package is built.') % (expected_type, extra))
        continue

    count, first_t, last_t = stats[topic]
    hz = 0.0
    if count >= 2 and last_t is not None and first_t is not None and last_t > first_t:
        hz = (count - 1) / (last_t - first_t)

    pub_n = node.count_publishers(topic)

    if count == 0:
        emit('RES', ng, topic,
             tr('%s のデータ受信に失敗しました (出版者 %d)。%s',
                'No data received on %s (publishers: %d). %s') % (topic, pub_n, hint_text(hint)))
        continue

    if kind == 'latched':
        emit('RES', 'OK', topic, tr('ラッチ受信 %d 件 (transient_local, pub=%d)',
                                    'latched: %d sample(s) (transient_local, pub=%d)') % (count, pub_n))
    elif kind == 'event':
        emit('RES', 'OK', topic, tr('%d 件受信 / %.1f Hz (pub=%d)',
                                    '%d msg(s) received / %.1f Hz (pub=%d)') % (count, hz, pub_n))
    else:
        lo = min_hz if min_hz is not None else expect_hz * 0.7
        hi = max_hz
        if hz < lo:
            emit('RES', ng, topic,
                 tr('周波数が不足しています (実測 %.1f Hz / 期待 %.1f Hz / 下限 %.1f Hz)。%s',
                    'Rate too low (measured %.1f Hz / expected %.1f Hz / minimum %.1f Hz). %s') % (hz, expect_hz, lo, hint_text(hint)))
        elif hi is not None and hz > hi:
            emit('RES', 'WARN', topic,
                 tr('周波数が過大です (実測 %.1f Hz / 期待 %.1f Hz / 上限 %.1f Hz)。出版者が二重に起動していないか確認してください。',
                    'Rate too high (measured %.1f Hz / expected %.1f Hz / maximum %.1f Hz). Check that the publisher is not running twice.') % (hz, expect_hz, hi))
        else:
            emit('RES', 'OK', topic, tr('%.1f Hz (期待 %.1f Hz, pub=%d)',
                                        '%.1f Hz (expected %.1f Hz, pub=%d)') % (hz, expect_hz, pub_n))

    if extra:
        for m in extra:
            emit('RES', 'WARN', topic + ' (QoS)',
                 tr('QoS が不整合です: %s。出版側と購読側の QoS 設定を合わせてください。',
                    'QoS mismatch: %s. Align the QoS settings of publisher and subscriber.') % m)

# =========================== 3. TF =========================================
emit('SEC', 'TF')

if ROLE == 'dell':
    for parent, child, is_static, max_delay, hint in TF_PAIRS:
        emit('RES', 'SKIP', '%s -> %s' % (parent, child), SKIP_REASON)
else:
    now = node.get_clock().now()
    for parent, child, is_static, max_delay, hint in TF_PAIRS:
        label = '%s -> %s' % (parent, child)
        try:
            tf = tf_buffer.lookup_transform(parent, child, Time())
        except Exception as exc:  # noqa: BLE001
            emit('RES', 'NG', label, '%s (%s)' % (hint, type(exc).__name__))
            continue

        t = tf.transform.translation
        pose = 'x=%.3f y=%.3f z=%.3f' % (t.x, t.y, t.z)

        if is_static or max_delay is None:
            emit('RES', 'OK', label, '%s (静的 TF)' % pose)
            continue

        stamp = Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds == 0:
            # /tf_static で配信されていると lookup 時刻が 0 のまま返る
            emit('RES', 'WARN', label,
                 '%s はタイムスタンプが 0 です。動的 TF のはずが /tf_static で配信されていないか確認してください。' % label)
            continue

        delay = (now - stamp).nanoseconds / 1e9
        if delay > max_delay:
            emit('RES', 'NG', label,
                 'TF が古くなっています (遅延 %.2f 秒 / 許容 %.2f 秒)。%s' % (delay, max_delay, hint))
        else:
            emit('RES', 'OK', label, '%s (遅延 %.3f 秒)' % (pose, delay))

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
    section "$(t "ノード / トピック / TF" "Nodes / Topics / TF")"
    report NG "$(t "rclpy チェック" "rclpy check")" "$(t "ROS 2 の状態取得に失敗しました。install/setup.bash を source したか、python3-yaml と tf2_ros が導入されているか確認してください。" "Failed to query the ROS 2 graph. Check that install/setup.bash is sourced and that python3-yaml and tf2_ros are installed.")"
  fi
}

dispatch_ros_checks

# ---------------------------------------------------------------------------
# 5. CAN
# ---------------------------------------------------------------------------
section "CAN (${CAN_IF})"

check_can() {
  if [[ "$ROLE" == "jetson" ]]; then
    report SKIP "CAN バス" "DellPC 側の担当です。DellPC 上のスクリプトで ROLE=\"dell\" にして確認してください。"
    return
  fi

  # 以降は ROLE=dell でのみ実行されるため、メッセージは英語のみ
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
  # 経路が生きていることを示す、dell の役目の最終出力
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
  check_id 301 NG   "CAN ID 0x301 (ODrive heartbeat)" "ODrive heartbeat (0x301) not received. Check the ODrive power, node_id setting (24), and bitrate match."
  check_id 309 WARN "CAN ID 0x309 (ODrive encoder est.)" "ODrive encoder estimates (0x309) not received. Check the ODrive cyclic message settings (encoder_estimates rate)."
  check_id 712 WARN "CAN ID 0x712 (emergency stop)" "Emergency stop board (0x712) not received. Check the board power and CAN wiring."
}

check_can

# ---------------------------------------------------------------------------
# 6. DellPC (足回り制御機) との接続
# ---------------------------------------------------------------------------
section "$(t "${PEER_NAME} 接続 (${PEER_IP})" "${PEER_NAME} connection (${PEER_IP})")"

check_network() {
  local subnet="${PEER_IP%.*}."
  local local_addrs
  local_addrs="$(ip -4 -brief addr show 2>/dev/null | awk '{for(i=3;i<=NF;i++) print $1" "$i}')"

  if grep -qF " ${OWN_IP}/" <<< "$local_addrs"; then
    report OK "$(t "自機の IP" "own IP")" "${OWN_IP} (${OWN_NAME})"
  else
    local own_addr
    own_addr="$(grep -F " ${subnet}" <<< "$local_addrs" | head -n1)"
    report NG "$(t "自機の IP" "own IP")" "$(t "自機に ${OWN_IP} が設定されていません (現在: ${own_addr:-${subnet}0/24 に該当なし})。${OWN_NAME} の固定 IP が ${OWN_IP}/24 になっているか、ROLE の設定が実機と合っているか確認してください。" "This machine does not have ${OWN_IP} (current: ${own_addr:-no address in ${subnet}0/24}). Check that the static IP of the ${OWN_NAME} is ${OWN_IP}/24 and that ROLE matches the actual machine.")"
  fi

  if ping -c 3 -W 1 "$PEER_IP" >/dev/null 2>&1; then
    local rtt
    rtt="$(ping -c 3 -W 1 "$PEER_IP" 2>/dev/null | tail -n1 | awk -F'/' '{print $5}')"
    report OK "ping ${PEER_IP}" "$(t "${PEER_NAME} まで 平均 RTT ${rtt:-?} ms" "avg RTT ${rtt:-?} ms to ${PEER_NAME}")"
  else
    report NG "ping ${PEER_IP}" "$(t "${PEER_NAME} (${PEER_IP}) への疎通に失敗しました。LAN ケーブルの接続、${PEER_NAME} の電源、固定 IP 設定 (${PEER_IP}/24) を確認してください。" "Failed to reach the ${PEER_NAME} (${PEER_IP}). Check the LAN cable, the power of the ${PEER_NAME}, and its static IP (${PEER_IP}/24).")"
    info "$(t "自機と ${PEER_NAME} が同一サブネット (${subnet}0/24) にいるかも確認してください。" "Also check that both machines are in the same subnet (${subnet}0/24).")"
    return
  fi

  local neigh
  neigh="$(ip neigh show "$PEER_IP" 2>/dev/null | head -n1)"
  if [[ -n "$neigh" ]]; then
    report OK "$(t "ARP エントリ" "ARP entry")" "$neigh"
  else
    report WARN "$(t "ARP エントリ" "ARP entry")" "$(t "ARP エントリの取得に失敗しました。L2 の到達性が不安定な可能性があります。" "Failed to get the ARP entry. L2 reachability may be unstable.")"
  fi

  if timeout 2 bash -c "echo > /dev/tcp/${PEER_IP}/22" 2>/dev/null; then
    report OK "SSH (22/tcp)" "$(t "接続可" "reachable")"
  else
    report WARN "SSH (22/tcp)" "$(t "${PEER_NAME} の SSH ポートへの接続に失敗しました。遠隔でログを見る場合は sshd の起動とファイアウォールを確認してください。" "Failed to connect to the SSH port of the ${PEER_NAME}. If you need remote log access, check that sshd is running and check the firewall.")"
  fi

  # マルチキャストが通らないと DDS のディスカバリが成立しない
  if ping -c 2 -W 1 224.0.0.1 2>/dev/null | grep -q "$PEER_IP"; then
    report OK "$(t "マルチキャスト" "multicast")" "$(t "${PEER_NAME} から応答あり (DDS ディスカバリ可)" "response from the ${PEER_NAME} (DDS discovery possible)")"
  else
    report WARN "$(t "マルチキャスト" "multicast")" "$(t "マルチキャスト (224.0.0.1) への ${PEER_NAME} からの応答確認に失敗しました。ROS 2 のディスカバリはマルチキャストを使うため、スイッチの IGMP スヌーピングとファイアウォールを確認してください。" "No multicast (224.0.0.1) response from the ${PEER_NAME}. ROS 2 discovery relies on multicast; check IGMP snooping on the switch and the firewall.")"
  fi

  report OK "$(t "ROS_DOMAIN_ID の整合" "ROS_DOMAIN_ID consistency")" "$(t "自機は ${ROS_DOMAIN_ID:-0}。${PEER_NAME} 側でも同じ値になっているか確認してください。" "This machine uses ${ROS_DOMAIN_ID:-0}. Check that the ${PEER_NAME} uses the same value.")"
}

check_network

# ---------------------------------------------------------------------------
# サマリ
# ---------------------------------------------------------------------------
TOTAL=$((OK_COUNT + NG_COUNT + WARN_COUNT + SKIP_COUNT))

printf "\n%s===== %s =====%s\n" "$C_SEC" "$(t "結果" "Summary")" "$C_OFF"
if [[ "$ROLE" == "dell" ]]; then
  printf "  Total %d checks : %sOK %d%s / %sNG %d%s / %sWARN %d%s / %sSKIP %d%s\n\n" \
    "$TOTAL" \
    "$C_OK" "$OK_COUNT" "$C_OFF" \
    "$C_NG" "$NG_COUNT" "$C_OFF" \
    "$C_WARN" "$WARN_COUNT" "$C_OFF" \
    "$C_SKIP" "$SKIP_COUNT" "$C_OFF"
else
  printf "  合計 %d 項目 : %sOK %d%s / %sNG %d%s / %sWARN %d%s / %sSKIP %d%s\n\n" \
    "$TOTAL" \
    "$C_OK" "$OK_COUNT" "$C_OFF" \
    "$C_NG" "$NG_COUNT" "$C_OFF" \
    "$C_WARN" "$WARN_COUNT" "$C_OFF" \
    "$C_SKIP" "$SKIP_COUNT" "$C_OFF"
fi

if (( NG_COUNT > 0 )); then
  if [[ "$ROLE" == "dell" ]]; then
    printf "  %sThere are %d NG items. See the → hints above.%s\n\n" "$C_NG" "$NG_COUNT" "$C_OFF"
  else
    printf "  %sNG が %d 件あります。上の → の指示を確認してください。%s\n\n" "$C_NG" "$NG_COUNT" "$C_OFF"
  fi
  exit 1
fi

if (( WARN_COUNT > 0 )); then
  if [[ "$ROLE" == "dell" ]]; then
    printf "  %sNo fatal NG. Depending on the situation, the %d WARN item(s) may be normal.%s\n\n" "$C_WARN" "$WARN_COUNT" "$C_OFF"
  else
    printf "  %s致命的な NG はありません。WARN %d 件は運用状況によっては正常です。%s\n\n" "$C_WARN" "$WARN_COUNT" "$C_OFF"
  fi
fi

exit 0
