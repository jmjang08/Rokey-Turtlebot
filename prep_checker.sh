#!/usr/bin/env bash
set -euo pipefail

############### 인자 파싱 ###############

NAME_SPACE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--n|--ns|--namespace)
      ROBOT_ID="$2"
      NAME_SPACE="/robot${ROBOT_ID}"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: ./prep_checker --n <robot_number>"
      exit 1
      ;;
  esac
done

# 기본값 (옵션을 입력하지 않은 경우)
if [[ -z "${NAME_SPACE}" ]]; then
  NAME_SPACE="/robot1"
fi

echo "Using namespace: ${NAME_SPACE}"

############### 설정값 (필요하면 여기만 수정) ###############

ODOM_FRAME=${ODOM_FRAME:-odom}
BASE_LINK_FRAME=${BASE_LINK_FRAME:-base_link}
LASER_FRAME=${LASER_FRAME:-rplidar_link}
SCAN_TOPIC=${SCAN_TOPIC:-/scan}
MAP_TOPIC=${MAP_TOPIC:-/map}
TIMEOUT=${TIMEOUT:-10}

##############################################################

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

ok()   { echo -e "${GREEN}[OK]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
fail() { echo -e "${RED}[FAIL]${NC} $*"; exit 1; }

echo "========== Localization 선행조건 체크 시작 =========="
echo "ODOM_FRAME     = ${ODOM_FRAME}"
echo "BASE_LINK_FRAME= ${BASE_LINK_FRAME}"
echo "LASER_FRAME    = ${LASER_FRAME}"
echo "SCAN_TOPIC     = ${SCAN_TOPIC}"
echo "MAP_TOPIC      = ${MAP_TOPIC}"
echo "TIMEOUT        = ${TIMEOUT}s"
echo "===================================================="
echo

# 0. ROS 환경 대략 체크
if ! command -v ros2 >/dev/null 2>&1; then
  fail "ros2 명령어를 찾을 수 없습니다. ROS2 환경이 설정되지 않았습니다."
fi

if ! ros2 node list >/dev/null 2>&1; then
  warn "현재 ros2 node list 가 비어있거나 ROS master와 통신이 안 될 수 있습니다."
fi

echo

# 1. /tf, /tf_static 토픽 존재 여부
echo "1) TF 토픽 존재 확인 (/tf, /tf_static)..."

TF_LIST=$(ros2 topic list)

# TF 토픽 패턴 검사: /tf 또는 */tf
if ! echo "${TF_LIST}" | grep -E "/tf(\s|$)" >/dev/null && \
   ! echo "${TF_LIST}" | grep -E ".*/tf(\s|$)" >/dev/null; then
  fail "TF 토픽이 없습니다. (/tf 혹은 <namespace>/tf)"
fi

# TF_STATIC 검사
if ! echo "${TF_LIST}" | grep -E "/tf_static(\s|$)" >/dev/null && \
   ! echo "${TF_LIST}" | grep -E ".*/tf_static(\s|$)" >/dev/null; then
  fail "TF_STATIC 토픽이 없습니다. (/tf_static 혹은 <namespace>/tf_static)"
fi

ok "TF 토픽과 TF_STATIC 토픽이 네임스페이스에서 정상적으로 발견됨."

echo

# 2. odom → base_link TF 가 실제로 들어오는지 확인
echo "2) TF: ${ODOM_FRAME} → ${BASE_LINK_FRAME} 변환 확인 (오도메트리 TF)..."

TMP_ODOM_BASE_TF=$(mktemp)
set +e
REMAP_ARGS=()
if [[ -n "${NAME_SPACE}" ]]; then
  REMAP_ARGS=(--ros-args -r tf:=${NAME_SPACE}/tf -r tf_static:=${NAME_SPACE}/tf_static)
fi

timeout "${TIMEOUT}" \
  ros2 run tf2_ros tf2_echo "${ODOM_FRAME}" "${BASE_LINK_FRAME}" \
    --ros-args -r tf:=${NAME_SPACE}/tf -r tf_static:=${NAME_SPACE}/tf_static \
  | head -n 20 > "${TMP_ODOM_BASE_TF}" 2>&1
set -e

if ! grep -q "At time" "${TMP_ODOM_BASE_TF}"; then
  echo "  - tf2_echo 실행 로그:"
  sed -e 's/^/    /' "${TMP_ODOM_BASE_TF}" | head -n 10
  rm -f "${TMP_ODOM_BASE_TF}"
  fail "${ODOM_FRAME} → ${BASE_LINK_FRAME} TF가 유효하지 않습니다."
fi

rm -f "${TMP_ODOM_BASE_TF}"
ok "${ODOM_FRAME} → ${BASE_LINK_FRAME} TF 정상: 오도메트리와 base_link 프레임이 연결되었습니다."

echo

# 3. 라이다 토픽 및 TF 확인
echo "3) LaserScan 토픽 및 라이다 프레임 TF 확인..."

# 3-1) 스캔 토픽 존재
if ! ros2 topic list | grep -qx "${NAME_SPACE}${SCAN_TOPIC}"; then
  fail "LaserScan 토픽 ${NAME_SPACE}${SCAN_TOPIC} 이(가) 존재하지 않습니다. 라이다 드라이버/브링업을 확인하세요."
fi
ok "LaserScan 토픽 ${NAME_SPACE}${SCAN_TOPIC} 발견."

# 3-2) 토픽 타입 확인 (sensor_msgs/msg/LaserScan)
FULL_SCAN_TOPIC="${NAME_SPACE}${SCAN_TOPIC}"

SCAN_TYPE=$(ros2 topic type "${FULL_SCAN_TOPIC}" 2>/dev/null)

if [[ "${SCAN_TYPE}" != "sensor_msgs/msg/LaserScan" ]]; then
  warn "토픽 ${FULL_SCAN_TOPIC} 타입이 LaserScan이 아닙니다: ${SCAN_TYPE}"
else
  ok "토픽 ${FULL_SCAN_TOPIC} 타입이 sensor_msgs/msg/LaserScan 입니다."
fi

# 3-3) base_link → laser TF 확인
echo "  - TF: ${BASE_LINK_FRAME} → ${LASER_FRAME} 확인..."
TMP_BASE_LASER_TF=$(mktemp)
set +e
timeout "${TIMEOUT}" \
  ros2 run tf2_ros tf2_echo "${BASE_LINK_FRAME}" "${LASER_FRAME}" \
    --ros-args -r tf:=${NAME_SPACE}/tf -r tf_static:=${NAME_SPACE}/tf_static \
  | head -n 20 > "${TMP_BASE_LASER_TF}" 2>&1

set -e

if ! grep -q "At time" "${TMP_BASE_LASER_TF}"; then
  echo "  - tf2_echo 실행 로그:"
  sed -e 's/^/    /' "${TMP_BASE_LASER_TF}" | head -n 10
  rm -f "${TMP_BASE_LASER_TF}"
  fail "${BASE_LINK_FRAME} → ${LASER_FRAME} TF 를 ${TIMEOUT}s 안에 가져오지 못했습니다. URDF 또는 static TF 설정을 확인하세요."
fi

rm -f "${TMP_BASE_LASER_TF}"

echo

# 5. 최종 요약
echo "================= 체크 완료 ================="
ok "모든 필수 선행조건을 만족했습니다."
echo "이 상태라면 AMCL/localization 노드를 실행했을 때,"
echo "- TF(odom ↔ base_link, base_link ↔ laser)"
echo "- LaserScan 입력"
echo "이 모두 준비된 상태이므로, 설정/파라미터만 올바르다면 localization은 정상적으로 동작할 가능성이 매우 높습니다."
echo "============================================="
