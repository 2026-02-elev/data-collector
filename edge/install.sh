#!/bin/sh
# CellGuard edge 설치/갱신 (Pi의 pi 계정에서 실행, sudo 불필요)
#   필요: python3-paho-mqtt, python3-numpy, pi가 dialout 그룹, loginctl linger=yes (부팅 시 자동 시작)
#   이 폴더: cellguard_edge.py, cg_ai.py, cellguard-edge.service, (선택) ca.crt, config.json, models/*.zip
set -eu
cd "$(dirname "$0")"
DEST="$HOME/cellguard-edge"
UNIT_DIR="$HOME/.config/systemd/user"

mkdir -p "$DEST" "$UNIT_DIR" "$HOME/.local/bin"
chmod 700 "$DEST"
install -m 755 cellguard_edge.py "$DEST/"
install -m 644 cg_ai.py "$DEST/"
[ -f ca.crt ] && install -m 644 ca.crt "$DEST/"
[ -f config.json ] && [ ! -f "$DEST/config.json" ] && install -m 644 config.json "$DEST/"
install -m 644 cellguard-edge.service "$UNIT_DIR/"
ln -sf "$DEST/cellguard_edge.py" "$HOME/.local/bin/cellguard-edge"

[ -f "$DEST/secrets.env" ] || echo "경고: $DEST/secrets.env 없음 (MQTT_USERNAME=, MQTT_PASSWORD=, EDGE_TOKEN=) - MQTT 전송 꺼짐"
grep -q '^EDGE_TOKEN=' "$DEST/secrets.env" 2>/dev/null || echo "알림: secrets.env 에 EDGE_TOKEN= 이 없음 - MLOps OTA는 서버가 거절할 수 있음 (오프라인 설치: cellguard-edge ota-install)"
[ -f "$DEST/ca.crt" ] || echo "경고: $DEST/ca.crt 없음 - TLS 연결 불가"
python3 -c 'import numpy' 2>/dev/null || echo "경고: numpy 없음 - AI 판정 꺼짐 (sudo apt install python3-numpy)"
id -nG | grep -qw dialout || echo "경고: $(id -un) 가 dialout 그룹이 아님 - /dev/serial0 접근 불가"
[ "$(loginctl show-user "$(id -un)" -p Linger --value 2>/dev/null)" = yes ] || \
  echo "경고: linger 꺼짐 - 부팅 시 자동 시작하려면: sudo loginctl enable-linger $(id -un)"

# 예전 GPT 수집기 감시자(crontab @reboot)는 새 서비스와 직렬포트를 다투므로 끈다
if crontab -l 2>/dev/null | grep -q cellguard-watchdog.py; then
  crontab -l > "$DEST/crontab.backup.$(date +%Y%m%d%H%M%S)"
  crontab -l | grep -v cellguard-watchdog.py | crontab -
  echo "예전 cellguard-watchdog crontab 항목 제거 (백업: $DEST/crontab.backup.*)"
fi
pgrep -f '[c]ellguard-watchdog.py' >/dev/null && pkill -f '[c]ellguard-watchdog.py' || true

# 함께 온 모델 패키지(models/NAME-VER.zip)는 서버 없이 설치한다. 검증(체크섬, 테스트 벡터)을 통과해야 들어간다
if ls models/*.zip >/dev/null 2>&1; then
  for z in models/*.zip; do
    name=$(basename "$z" .zip); name=${name%-*}
    python3 "$DEST/cellguard_edge.py" ota-install "$name" "$z" || echo "모델 $z 설치 실패 (지금 모델 유지)"
  done
fi

systemctl --user daemon-reload
systemctl --user enable cellguard-edge.service
systemctl --user restart cellguard-edge.service
sleep 3
systemctl --user --no-pager status cellguard-edge.service | head -12
