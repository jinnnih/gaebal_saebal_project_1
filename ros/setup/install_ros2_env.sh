#!/usr/bin/env bash
# ros2_env.sh 를 홈에 설치하고 ~/.bashrc 에 연결합니다.
# Ubuntu 24.04 (ROS 2 Jazzy) 머신에서 실행하세요.
#
#   bash install_ros2_env.sh
#
# 여러 번 실행해도 ~/.bashrc 에 중복으로 쌓이지 않습니다.

set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ros2_env.sh"
DEST="$HOME/.ros2_env.sh"
BASHRC="$HOME/.bashrc"
BEGIN="# >>> ros2 env (gaebal-saebal) >>>"
END="# <<< ros2 env (gaebal-saebal) <<<"

[ -f "$SRC" ] || { echo "오류: $SRC 가 없습니다."; exit 1; }

# 1. 환경 스크립트 배치
cp "$SRC" "$DEST"
echo "설치: $DEST"

# 2. ~/.bashrc 백업 (기존 파일이 있을 때만)
if [ -f "$BASHRC" ]; then
  BACKUP="$BASHRC.bak.$(date +%Y%m%d%H%M%S)"
  cp "$BASHRC" "$BACKUP"
  echo "백업: $BACKUP"
else
  touch "$BASHRC"
  echo "생성: $BASHRC (기존 파일 없음)"
fi

# 3. 기존 블록 제거 후 재삽입 (중복 방지)
if grep -qF "$BEGIN" "$BASHRC"; then
  sed -i "/$(printf '%s' "$BEGIN" | sed 's/[][\.*^$/]/\\&/g')/,/$(printf '%s' "$END" | sed 's/[][\.*^$/]/\\&/g')/d" "$BASHRC"
  echo "기존 블록 제거됨 (갱신)"
fi

{
  echo ""
  echo "$BEGIN"
  echo '[ -f "$HOME/.ros2_env.sh" ] && source "$HOME/.ros2_env.sh"'
  echo "$END"
} >> "$BASHRC"
echo "연결: $BASHRC 에 source 라인 추가"

# 4. 문법 검사
bash -n "$DEST" && echo "문법 검사 통과"

echo ""
echo "완료. 아래 명령으로 적용하세요:"
echo "  source ~/.bashrc && rosinfo"
