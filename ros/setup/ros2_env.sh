#!/usr/bin/env bash
# ROS 2 Jazzy 환경 설정 — 개발새발 팀 공용
# ~/.ros2_env.sh 로 배치되고 ~/.bashrc 에서 source 됩니다.
# 대상: Ubuntu 24.04 LTS + ROS 2 Jazzy Jalisco

# ── 1. ROS 2 배포판 ──────────────────────────────────────────────
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "[ros2_env] /opt/ros/jazzy 를 찾을 수 없습니다. ROS 2 Jazzy 설치를 확인하세요."
fi

# ── 2. 도메인 ID ─────────────────────────────────────────────────
# 같은 네트워크에서 값이 같으면 노드가 서로 보입니다.
# 팀원과 협업할 땐 같은 값, 각자 실습할 땐 다른 값을 쓰세요. (0~101 권장)
export ROS_DOMAIN_ID=42

# ── 3. 디스커버리 범위 ───────────────────────────────────────────
# SUBNET      : 같은 서브넷의 노드까지 발견 (기본값, 팀 협업용)
# LOCALHOST   : 내 PC 안에서만 (강의 실습 중 남의 노드와 섞이기 싫을 때)
# OFF         : 디스커버리 끔
# ※ Jazzy에서는 구버전의 ROS_LOCALHOST_ONLY 대신 이 변수를 씁니다.
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# ── 4. 미들웨어 구현체 ───────────────────────────────────────────
# Jazzy 기본값. 팀원 간 값이 다르면 통신이 안 되니 통일해두세요.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ── 5. 콘솔 로그 포맷 ────────────────────────────────────────────
# 디버깅할 때 어느 노드의 몇 번째 줄인지 바로 보이게
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}'

# ── 6. 개인 워크스페이스 자동 source ─────────────────────────────
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi

# ── 7. colcon 자동완성 / 이동 ────────────────────────────────────
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi
if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
  source /usr/share/colcon_cd/function/colcon_cd.sh
  export _colcon_cd_root=/opt/ros/jazzy/
fi

# ── 8. 자주 쓰는 별칭 ────────────────────────────────────────────
alias cw='cd ~/ros2_ws'
alias cs='cd ~/ros2_ws/src'
alias cb='cd ~/ros2_ws && colcon build --symlink-install && source ~/ros2_ws/install/setup.bash'
alias cbp='cd ~/ros2_ws && colcon build --symlink-install --packages-select'
alias cbc='cd ~/ros2_ws && rm -rf build install log'
alias rosdi='rosdep install --from-paths ~/ros2_ws/src --ignore-src -r -y'

alias rt='ros2 topic list'
alias rte='ros2 topic echo'
alias rti='ros2 topic info'
alias rn='ros2 node list'
alias rni='ros2 node info'
alias rs='ros2 service list'
alias rp='ros2 param list'
alias rdoctor='ros2 doctor --report'

# 현재 ROS 환경 한눈에 보기
rosinfo() {
  echo "ROS_DISTRO                    : ${ROS_DISTRO:-(unset)}"
  echo "ROS_DOMAIN_ID                 : ${ROS_DOMAIN_ID:-(unset)}"
  echo "ROS_AUTOMATIC_DISCOVERY_RANGE : ${ROS_AUTOMATIC_DISCOVERY_RANGE:-(unset)}"
  echo "RMW_IMPLEMENTATION            : ${RMW_IMPLEMENTATION:-(unset)}"
  echo "워크스페이스                   : $([ -f "$HOME/ros2_ws/install/setup.bash" ] && echo "~/ros2_ws (source됨)" || echo "없음")"
}
