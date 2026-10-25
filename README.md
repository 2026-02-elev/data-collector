# CellGuard 데이터 수집기 (Raspberry Pi)

- 작성/구현: **Yongho Ahn** <nanhosoft@gmail.com>
- 역할: Arduino Nano(CellGuard 펌웨어, UART 프로토콜 v1.0) 제어·텔레메트리 수집, Pi I²C INA219 팩 전류 수집, 시간정합 후 `normalized_sample_v1` 저장, 서버 업로드용 outbox 관리
- 대상 장비: Raspberry Pi Zero 2 WH + Arduino Nano + PCB1×4 + PCB2. **현재 실물 펌웨어 `0.1.6-LOAD-VERIFY`**(RAMP/DRAIN 시운전 빌드)와 저장소 펌웨어 `0.2.0` 을 모두 지원한다 (아래 "펌웨어 호환성").
- 연계 저장소: 펌웨어 `2026-02-elev/arudino-firmware` (프로토콜 정의 `docs/uart_protocol.md`, 호스트 시뮬레이터 `tests/host`), 합성 데이터 `2026-02-elev/data-generator` (같은 스키마)

> **안전 고지.** 액추에이션(ARM/CHARGE/DISCHARGE/ESR/LEAK/HEAT/RESET/BENCH)은 `--i-am-supervising` 이 없으면 수집기가 거부하며,
> 펌웨어는 실물 검증 게이트가 잠겨 있으면 `ERR,HARDWARE_LOCKED` 로 별도 거부한다. ACK `OK` 는 "명령 수락" 이지
> 릴레이·MOSFET·히터가 물리적으로 동작했다는 뜻이 아니다. 이 프로그램은 무인 충방전/히터 운용을 승인하지 않는다.
> 이 저장소의 시험은 모두 시뮬레이터(펌웨어 소스를 PC 용으로 빌드한 것)로 했으며 **실물 장비에서는 실행하지 않았다.**

## 펌웨어 호환성

`SYNC` 응답의 `CGV` 버전 문자열로 펌웨어 계열을 판별하고(`cg_collector/firmware.py`), **그 펌웨어가 지원하지 않는 명령은
아예 보내지 않는다**(`UnsupportedCommand`, UART 로 나가지 않음). 버전을 모르면(CGV 전, 또는 알 수 없는 버전) 읽기 전용 명령만 보낸다.

| 항목 | `0.1.6-LOAD-VERIFY` (현재 실물) | `0.2.0` (arudino-firmware) |
|---|---|---|
| 충전 | `RAMP,<mV/셀>` (150–1000, 휴지 전압 목표). `CHARGE,1` → 펌웨어 `ERR,USE_RAMP`, 수집기는 **보내지 않음** | `CHARGE,1` / `CHARGE,0` |
| 방전 | `DRAIN,<mV/셀>` (100–950). `DISCHARGE,1` → `ERR,USE_DRAIN`, **보내지 않음** | `DISCHARGE,1` / `DISCHARGE,0` |
| 정지 이벤트 | `CHARGE_STOP` / `DRAIN_START`·`DRAIN_STOP` (사유 `TARGET`, `DRAIN_TARGET`, `LOAD_TAP_UNSTABLE`, `TIMEOUT`, `BUDGET`, 한계, `TRIP`, `HOST`…) | `CHARGE_*` / `DISCHARGE_*` |
| 단계 종료 후 | 자동 무장 해제 → 단계마다 새 `ARM` 필요 | ARMED 유지 |
| 부팅 자체점검 `SELFTEST` | **없음** → 부팅 점검에 `UNSUPPORTED` 로 표시 (보내지 않음) | 있음 (`CGE SELF_TEST`) |
| 진단 프레임 | `CGY` (부하 중 탭 역전 의심값), `CGX` (트립 원인 측정값) | 없음 |
| 추가 trip 코드 | 16 `CELL_IMBALANCE`, 17 `DISCHARGE_NO_RESPONSE` | – |
| 기준전압 | REF3040 4.096 V (CGK `aref_mv=4096`) | REF3025 2.5 V |
| ESR / LEAK / HEAT | 펌웨어에서 잠김 → 수집기도 보내지 않음 | 게이트에 따름 |
| 수집기 명령 | `selftest`, `monitor`, **`cycle`**, `query`, `import-cycle-log` | `selftest`, `monitor`, `run`, `shell`, `query` |

근거: 실물에 올린 `CellGuardNano_LoadVerify` 0.1.6 소스(변경 없이 `tests/fw016/` 에 보관, `SHA256SUMS`)와 `arudino-firmware`.
`tests/fw016/sim016` 은 그 0.1.6 소스를 그대로 PC 용으로 빌드하고 펌웨어에 동봉된 `load_verify_smoke.cpp` 의 플랜트 모델을 붙인
**시뮬레이터**다(CGV 에 HOST_SIM_BUILD 표시 → 기록은 `source="synthetic"`). 전기적 모델이 아니다.

## 구조

```text
cg_collector/
  protocol.py   프레임/CRC16/메시지 파서 (CGA CGT CGS CGD CGE CGV CGK)
  transport.py  SerialTransport(115200 8N1) / SimTransport(펌웨어 호스트 시뮬레이터)
  session.py    NanoSession: seq·같은 seq 재전송·ACK 매칭·하트비트, 모든 수신 프레임을 구독자에 전달,
                FaultTracker: 요청(requested) / 명령(commanded) / 물리(미검증) 상태 분리
  current.py    INA219 드라이버 + CurrentSampler(백그라운드 10Hz, 이력 버퍼, 시간정합)
  fusion.py     CGT+CGS+CGD+전류 → normalized_sample_v1 (1Hz 슬롯, 결측 슬롯, 실험 경과시간), 메모리 상한
  events.py     명령/CGE → fault_event_v1 (REQUESTED/FAULT_REJECTED/COMMANDED/TRIPPED/RESET/BOOT)
  replay.py     raw UART 로그(수집기 raw-*.jsonl 또는 생성기 nano_uart.log) → samples/events 재생성
  storage.py    RotatingWriter(spool → outbox, 크기/시간 회전, 비정상 종료 .part 검증 복구), Outbox + Uploader 인터페이스
  runner.py     실험 시퀀스 실행기(한 세션 유지), 대화형 shell 용 Pump 스레드
  preflight.py  부팅 점검(테스트 모드): 링크·펌웨어·Nano 자체점검·상태·텔레메트리·측정·ADC 경로·INA219·시계·저장소
  firmware.py   펌웨어 계열별 지원 명령/기능 (0.1.6-LOAD-VERIFY, 0.2.x, 0.1.0, unknown)
  cycle.py      감독하 단계 충방전(RAMP/DRAIN) — cellguard_cycle.py 의 검증된 절차를 수집 세션 안에서 실행
  diagnostics.py CGY/CGX·0.1.6 진단 이벤트·단계 판정 → nano_diagnostic_v1
  cyclelog.py   cellguard_cycle_*.log 가져오기 → raw/samples/events/diagnostics
  cli.py        cg-collect selftest | monitor | cycle | run | shell | query | outbox | replay | import-cycle-log
schemas/        normalized_sample_v1, fault_event_v1 (합성기와 공통 계약), nano_diagnostic_v1 (진단 전용)
examples/       plan_charge_esr.json (감독하 실험 계획 예시), cellguard-collector.service (부팅 시 자동 시작)
tests/          단위시험 + 펌웨어 호스트 시뮬레이터 통합시험 (0.2.0: ../arudino-firmware, 0.1.6: tests/fw016)
tests/fw016/    실물 0.1.6 펌웨어 소스(무수정) + sim016.cpp(PC 시뮬레이터) + 로그 픽스처 생성기
```

## 설치 / 시험

```bash
pip install -r requirements-dev.txt        # Pi 에서 INA219 사용 시: pip install smbus2
pip install -e .                           # cg-collect 명령 설치 (또는 python3 -m cg_collector ...)

# 통합시험은 펌웨어 호스트 시뮬레이터를 빌드해 사용한다 (g++ 필요).
#   0.2.0: ../arudino-firmware (다른 위치면 CELLGUARD_FIRMWARE_DIR), 0.1.6: tests/fw016 (자동 빌드)
python3 -m pytest -q        # 89 passed (2026-10-25)
```

## 실물 장비 (0.1.6-LOAD-VERIFY) 사용

> 아래 명령 중 출력을 구동하는 것은 `cycle` 하나뿐이며, **사람이 장비 옆에서 감독할 때만** 실행한다.
> `cellguard_cycle.py` 와 **동시에 실행하지 말 것** — 둘 다 `/dev/serial0` 을 연다. `cycle` 이 같은 일을 하면서 데이터도 기록한다.

```bash
# 1) Pi 에 배포 (Python 3.9+, 실행 의존성은 pyserial 뿐; 시험용 pytest/jsonschema 는 requirements-dev.txt)
sudo apt install -y python3-venv python3-serial git
git clone -b claude/lucid-dijkstra-7lpdtl https://github.com/2026-02-elev/data-collector.git ~/data-collector
cd ~/data-collector && python3 -m venv --system-site-packages .venv && . .venv/bin/activate
pip install -e .
sudo usermod -aG dialout $USER          # /dev/serial0 접근 (재로그인 후 sudo 없이 실행)
sudo mkdir -p /var/lib/cellguard && sudo chown $USER /var/lib/cellguard

# 2) 출력 동작 없이 수집기만 점검 (cellguard_cycle.py --probe-only 에 해당: SYNC/PING/STATUS/RAW/CAL 만 보냄)
cg-collect --port /dev/serial0 --data-dir /var/lib/cellguard selftest
cg-collect --port /dev/serial0 --data-dir /var/lib/cellguard --run-id idle-001 monitor --seconds 60   # 수동 기록만

# 3) 감독하 충방전 + 데이터 수집 (cellguard_cycle.py --charge-to 400 --step 50 --discharge-to 100 과 같은 절차)
cg-collect --port /dev/serial0 --data-dir /var/lib/cellguard --run-id cycle-001 \
           cycle --charge-to 400 --step 50 --discharge-to 100 --i-am-supervising

# 4) 기존 cellguard_cycle_*.log 를 표준 데이터로 변환 (장비 불필요, 로그의 시각대 지정)
cg-collect --device-id cellguard-001 import-cycle-log ~/cellguard_cycle_20261025_132840.log \
           --log-tz Asia/Seoul --out /var/lib/cellguard/imported/cycle_20261025_132840
```

`cycle` 의 동작 (`cellguard_cycle.py` 0.1.6 과 동일한 규칙, `tests/test_fw016.py` 가 원본 함수와 대조):

1. 부팅 점검을 **항상** 먼저 한다 (`--boot-check` 와 무관). FAIL 이 있거나, 0.1.6 처럼 Nano 자체점검이 없는데
   Pi 측 대체 점검(nano_status·telemetry·measurement·adc_path) 중 하나라도 PASS/WARN 이 아니면 **아무것도 구동하지 않고 종료**.
2. `SYNC`(모든 출력 OFF) → `CAL` → 첫 CGT 확인 → Nano READY·릴레이 0 확인 → 초기 셀 전압 검사(최저 < −20 mV 또는 편차 > 80 mV 면 중단).
3. 단계 계획: 가장 높은 셀 기준으로 `--step` 간격 목표(예: 최고 셀 142 mV, `--charge-to 400 --step 50` → 200, 250, 300, 350, 400 mV).
4. 단계마다 **새 `ARM`** → `RAMP,<목표>` (방전은 `DRAIN,<목표>`) → 1 s 마다 `PING`(ACK 필수) →
   `CHARGE_STOP,TARGET`(방전 `DRAIN_STOP,DRAIN_TARGET`) 다음 `RELAY_OFF` 와 CGS 릴레이 0 확인 →
   **최소 2.3 s 휴지 + 새 CGT 2개 이상** → 셀 편차 ≤ 80 mV, 목표 ±40 mV, 모든 셀이 올바른 방향으로 변했는지 확인.
5. 시간 제한: RAMP 32 s, DRAIN 37 s (Nano 자체 25 s / 30 s 와 별도).
6. 다른 정지 사유(`LOAD_TAP_UNSTABLE`, `TIMEOUT`, `BUDGET`, 한계), `TRIP`, 무응답, 시간 초과, Ctrl-C → **즉시 `OFF`**, 실패(`ABORTED`) 종료.
   자동 재시도·자동 `RESET`·자동 재가동 없음. `LOAD_TAP_VERIFY,REST_OK` 는 성공이 아니다(무부하 전압이 정상으로 돌아왔다는 뜻일 뿐).
7. 종료 코드: 0 = PASS, 1 = ABORTED/거부, 2 = 인자·감독 플래그 오류, 130 = Ctrl-C.

원본 스크립트와 다른 점(모두 더 보수적이거나 기록용): PING 은 ACK 를 확인하며 1.2 s 안에 ACK 가 없으면 통신 단절로 보고 중단한다.
무효 셀 값(-32768)이 든 CGT 는 원본처럼 전압 검사에서 실패한다. 데이터 기록을 위해 CGT 마다 `RAW` 를 한 번 요청하는데,
원본과 똑같은 UART 트래픽을 원하면 `--no-raw` 를 붙인다(그러면 `nano_adc_*_raw` 는 null).

생성 파일 (`--data-dir /var/lib/cellguard` 기준, 회전 완료 후 `outbox/`):

```text
outbox/selftest-<UTC>-00001.jsonl     부팅 점검 보고서 (selftest_report_v1, actuation 허용 여부 포함)
outbox/raw-<UTC>-00001.jsonl          UART 원문 양방향 (첫 줄 RUN 헤더, 마지막 줄 END)
outbox/samples-<UTC>-00001.jsonl      normalized_sample_v1 (1 Hz)
outbox/events-<UTC>-00001.jsonl       fault_event_v1 (REQUESTED RAMP/DRAIN, COMMANDED *_START/*_STOP, TRIPPED …)
outbox/diagnostics-<UTC>-00001.jsonl  nano_diagnostic_v1 (CGY, CGX, LOAD_TAP_VERIFY, 단계 정지, cycle 판정)
spool/*.jsonl.part                    기록 중인 파일 (비정상 종료 시 다음 실행에서 검증 복구)
```

예시 (sim016 에서 생성 — 그래서 `source` 가 `synthetic`; 실물 0.1.6 은 `measured_normal`):

```json
{"schema_version":"normalized_sample_v1","source":"synthetic","run_id":"rig-0001","t_sim_s":1.823,"record_status":"OK",
 "cell_voltage_v":[0.134,0.143,0.15,0.151],"cumulative_tap_voltage_v":[0.134,0.277,0.427,0.578],
 "cell_temperature_c":[25.03,25.03,25.03,25.03],"pack_current_a":null,"current_quality":"INA219_UNAVAILABLE",
 "charge_commanded":false,"pack_relay_commanded":true,"power_requested":{"charge":true,"discharge":false},
 "power_stage_requested":{"command":"RAMP","direction":"CHARGE","target_mv_per_cell":200,"since_utc":"2026-10-14T00:00:05.300Z"},
 "power_path_reported":{"charge":true,"discharge":false},"controller_state":"ACTIVE","trip_reason":"NONE",
 "nano_adc_tap_raw":[17,35,53,71],"firmware_version":"0.1.6-LOAD-VERIFY","raw_log_ref":"raw-20261014T000004000000Z-00001.jsonl"}
{"schema_version":"fault_event_v1","event_type":"COMMANDED","fault_type":"NONE","requested_value":1,"commanded_value":0,
 "evidence":{"cge":{"cge_code":"CHARGE_STOP","reason":"TARGET","nano_millis":7830},
             "stage_stop":{"stage":"RAMP","stop_class":"TARGET_REACHED","nano_target_reached":true,"abort_reason":null}}}
{"schema_version":"nano_diagnostic_v1","kind":"LOAD_TAP_SUSPECT_SNAPSHOT","frame":"$CGY,16275,2,16,33,73,67,134,277,612,561,275*F478",
 "detail":{"direction":"DISCHARGE","measured_under_load":true,"rested_cell_voltage":false,"tap_mv_under_load":[134,277,612,561]}}
{"schema_version":"nano_diagnostic_v1","kind":"REST_VERIFY","detail":{"reason":"REST_OK","rested_voltage_ok":true,"stage_successful":false}}
```

`import-cycle-log` 출력 폴더: `raw.jsonl`(줄마다 `dir` RX→`N>P`/TX→`P>N`, 원본 `log_local_time`, `source_log`, `source_line`,
CRC/형식 판정 `error`), `samples.jsonl`, `events.jsonl`, `diagnostics.jsonl`, `import_report.json`(줄 수, RX/TX, CRC 오류,
프레임 종류, 펌웨어 버전, 원본 SHA-256). 모든 레코드의 `raw_log_ref` 는 `<로그 파일>:<줄 번호>`. 로그에 INA219 가 없으므로
전류는 null(`INA219_UNAVAILABLE`), 깨진 CGT 의 슬롯은 `MISSING_TELEMETRY` 로 남고 추정값을 채우지 않는다.

## 사용

```bash
# 수동 모니터링 (액추에이션 없음): SYNC, CAL, 1s 하트비트, 데이터 저장
cg-collect --port /dev/serial0 --data-dir /var/lib/cellguard --run-id run-001 monitor
# INA219 전류 포함 (션트값·설치 방향은 실측값으로)
cg-collect --port /dev/serial0 --ina219 --shunt-ohm 0.1 --current-sign 1 monitor

# 실험 계획 실행: 한 세션에서 SYNC → ARM → 명령/대기, 대기 중에도 하트비트·수집, 종료/오류 시 항상 OFF
cg-collect --port /dev/serial0 run examples/plan_charge_esr.json --i-am-supervising

# 대화형 제어: 세션 유지, 백그라운드 하트비트/수집
cg-collect --port /dev/serial0 shell --i-am-supervising
#   > arm / charge 1 / esr 2 1 / status / selftest / off / quit   (selftest = Nano 자체점검 재실행, 출력 OFF 상태에서만)

# 단발 조회(읽기 전용): ping | status | raw | cal
cg-collect --port /dev/serial0 query status

# raw 로그에서 samples/events 재생성 (파서 수정 후 재처리, 생성기 출력 검증)
cg-collect replay --from-dir /var/lib/cellguard/outbox --out rebuilt/           # raw-*.jsonl + ina219-*.jsonl
cg-collect replay --from-raw raw-*.jsonl --ina219-log ina219-*.jsonl --out rebuilt/
cg-collect replay --from-generator ../data-generator/demo_outputs/runs/F-01_demo_esr_cell2__r00 --out rebuilt/

# 하드웨어 없이 (SIMULATED DEVICE — 기록은 source="synthetic")
cg-collect --sim ../arudino-firmware/tests/host/build/nano_host_sim --sim-gates unlocked \
           run examples/plan_charge_esr.json --i-am-supervising
```

Pi 준비: `raspi-config` → Interface Options → Serial Port (login shell **No**, serial hardware **Yes**), I²C 활성화(INA219), 40핀 리본 1번 방향·도통 확인 후 연결.

## 부팅 점검 (테스트 모드)

`monitor` / `run` / `shell` 은 **시작 전에 항상 부팅 점검을 한다** (`--boot-check strict`, 기본).
FAIL 이 하나라도 있으면 아무것도 기록·구동하지 않고 종료(코드 1)한다. `--boot-check warn` 은 보고만 하고 시작, `off` 는 생략.
점검만 하려면 테스트 모드:

```bash
cg-collect --port /dev/serial0 --ina219 --shunt-ohm 0.1 selftest          # 표 출력, PASS/WARN → 0, FAIL → 1
cg-collect --port /dev/serial0 selftest --json --fail-on-warn               # 전체 보고서 JSON, WARN 도 1
cg-collect --sim ../arudino-firmware/tests/host/build/nano_host_sim selftest  # 하드웨어 없이
```

출력 형식 예시 (기본 빌드 + INA219 를 가정한 예시이며 실물에서 측정한 값이 아니다):

```text
[PASS] link            SYNC OK, PING 3.1 ms
[PASS] firmware        0.2.0 / protocol 1.0 / PCB2-20260918-R2
[WARN] nano_self_test  warnings: NTC_SATURATED, CAL_NOMINAL, HW_LOCKED
[PASS] nano_status     LOCKED, locks POWER_LOCKED,ESR_LOCKED,LEAK_LOCKED,HEAT_LOCKED
[PASS] telemetry       3 CGT in 3.0 s
[WARN] measurement     NTC 1,2,3,4 saturated: <= ~25 degC or open (heaters stay blocked)
[WARN] adc_path        raw -> taps agree (0 mV); NOMINAL (unmeasured) calibration
[PASS] ina219          10 reads VALID, 0.4 mA at rest
[PASS] clock           system clock NTP-synchronised
[PASS] storage         28734 MB free
BOOT CHECK: WARN
```

| 점검 | FAIL (시작 안 함) | WARN (기록하고 시작) |
|---|---|---|
| `link` | SYNC/PING 응답 없음 (전원, TX/RX 교차, 115200, 시리얼 콘솔) | 응답 중앙값 > 100 ms |
| `firmware` | 프로토콜 ≠ 1.0, 하네스 맵 ≠ `--expect-map`, 실제 포트에 시뮬레이터 펌웨어 | 벤치 빌드 |
| `nano_self_test` | Nano 자체점검(POST) 실패 비트, SELFTEST 미지원(펌웨어 < 0.2.0), 결과 없음 | 경고 비트 (0V 팩, NTC 포화, 공칭 교정, 게이트 잠김) |
| `nano_status` | TRIPPED (원인 확인 후 RESET) | 직전 리셋이 워치독/브라운아웃, RAM 부족, 루프 지연 |
| `telemetry` | 3 s 동안 CGT 없음, 프레임 오류 > 20% | 프레임 오류·seq 공백·CGT 부족 |
| `measurement` | AREF/ADC 포화/탭 역전/STALE, NTC 단락·변환불가 | 팩 ≈ 0 V(빈 팩 또는 하네스 미연결), NTC 포화, 정지 중 셀 값 흔들림 > 50 mV |
| `adc_path` | CGD raw 로 다시 계산한 탭과 CGT 탭 차이 > 30 mV | > 10 mV, 공칭 교정 |
| `ina219` (`--ina219`) | 장치 없음/설정 레지스터 불일치/I2C 오류/범위 밖 | 모든 출력 OFF 인데 \|I\| > 10 mA (오프셋·션트 배선 H4_1·부호) |
| `clock` | 시스템 연도 < 2025 (RTC 없는 Pi 가 NTP 전) | NTP 미동기 |
| `storage` | 데이터 디렉터리 쓰기 불가, 여유 < 100 MB | 여유 < 1 GB, 비정상 종료로 남은 `.part` |

- 보고서 `outbox/selftest-<UTC>-<n>.jsonl` (`selftest_report_v1`: 결과, 점검별 상태·근거 데이터, 펌웨어 정보, 점검 중 UART 원문)
  → 데이터 파일과 함께 서버로 간다. 장치가 왜 시작했는지/못 했는지 서버에서 볼 수 있다.
- Nano 쪽 자체점검 내용과 한계는 펌웨어 `docs/self_test.md`. **출력(릴레이·MOSFET·히터)은 되읽기 회로가 없어 점검하지 않는다**
  — 점검은 아무 출력도 켜지 않는다.
- 부팅 시 자동 시작: `examples/cellguard-collector.service` (systemd). 점검 FAIL 이면 종료 → 120 s 후 재시도, 매 시도 보고서 기록.

## 데이터 저장과 서버 전송

```text
<data-dir>/spool/samples-<시작UTC>-<n>.jsonl.part   기록 중 (레코드마다 flush)
<data-dir>/spool/raw-<시작UTC>-<n>.jsonl.part       첫 줄 RUN 헤더(run_start_utc 등) + 송수신 원본 프레임(양방향, dir=N>P|P>N) + Pi UTC(µs) + 파싱 오류
<data-dir>/spool/ina219-<시작UTC>-<n>.jsonl.part    (--ina219) INA219 읽기마다 1줄: ina219_raw_v1
<data-dir>/spool/diagnostics-<시작UTC>-<n>.jsonl.part  nano_diagnostic_v1 (CGY/CGX/0.1.6 진단 이벤트/cycle 판정)
<data-dir>/spool/events-<시작UTC>-<n>.jsonl.part    fault_event_v1 이벤트
<data-dir>/outbox/*.jsonl                            회전 완료 → 서버 업로드 대기
<data-dir>/sent/*.jsonl                              업로더가 성공 확인한 파일
<data-dir>/quarantine/                               비정상 종료 복구 보고서(.recovery.json)와 거부된 불완전 줄(.rejected)
```

- 기본 회전: 50 MB 또는 1시간 (`--rotate-mb`, `--rotate-s`).
- 비정상 종료로 남은 `.part` 는 다음 실행 시 **줄 단위로 검증**한다: 완결된 JSON 객체 줄만 outbox 로 옮기고,
  잘린 마지막 줄·깨진 줄은 버리지 않고 `quarantine/<name>.rejected` 에 원본 바이트 그대로 보관, 결과는 `<name>.recovery.json`.
- **서버 전송: 미구현(보류).** `storage.Uploader.upload(path) -> bool` 인터페이스와 `Outbox.flush(uploader)` 만 있다.
  HTTP/MQTT 업로더는 백엔드 API(엔드포인트, 인증, 재시도·멱등 키, 압축) 확정 후 구현한다. 업로드 성공 확인 전에는 파일을 지우거나 옮기지 않는다.
- 샘플의 `raw_log_ref` 는 해당 시점의 raw 파일 이름 → 원본 프레임으로 추적 가능.
- `ina219_raw_v1` (생성기 `ina219_raw.jsonl` 과 같은 형식):
  `{"schema_version":"ina219_raw_v1","sampled_utc","available_utc","current_a","quality","raw":{shunt_register,shunt_lsb_v,shunt_ohm,sign_correction}}`
  — `sampled_utc` 는 I²C 읽기 중간 시각, `available_utc` 는 읽기 완료(Pi 가 값을 알게 된) 시각, `current_a` 는 부호 보정 후 값(VALID 가 아니면 null).

### 재처리 (replay)

- raw UART 로그와 INA219 raw 로그를 **줄 단위 스트리밍**으로 읽어 시각순 병합(`heapq.merge`)한다 — 파일 전체를 메모리에 올리지 않는다(7일치 로그도 메모리 일정).
- INA219 읽기는 `available_utc` 시점에 샘플러에 들어가므로, 실시간 실행과 같은 전류가 선택된다 → 실측 로그 재처리로 전류 필드까지 동일하게 복원(`test_replay_of_own_raw_log_reproduces_samples`).
- INA219 로그가 없거나 비어 있으면(센서 미설치·`--ina219` 미사용) 전류는 null + `INA219_UNAVAILABLE`.

## 데이터 의미 (normalized_sample_v1 — data-generator 와 같은 레코드 구조)

합성(data-generator)과 실측(이 수집기)은 **같은 규칙으로 같은 필드**를 만든다. 학습에 쓴 입력이 실측에도 그대로 있어야 하기 때문이다.
`tests/test_generator_parity.py` 가 생성기 데모의 UART 로그(`nano_uart.log`)와 INA219 로그(`ina219_raw.jsonl`)를 이 수집기 코드로 재처리해
생성기 출력과 필드 단위로 비교한다. 측정·RAW·명령·품질·전류 필드와 `t_sim_s` 는 **정확히 일치**, 요청 필드는 슬롯 경계 ±50 ms 안의 변화만,
전송 오류 카운터는 인접 레코드로의 1건 이동만 허용한다 (생성기 전체 시나리오 27종으로도 확인: `CELLGUARD_GENERATOR_DIR=<runs 의 상위의 상위>`).

| 필드 | 의미 |
|---|---|
| `t_sim_s` | **실험 경과시간**[s]: 런 시작(수집 시작 = 첫 SYNC 직전) 기준, Nano 샘플 시점 추정치. 합성 데이터에서는 시나리오 시작 기준 — 같은 의미. Nano millis 를 Pi 시계에 대응(최소 오프셋 추정). 슬롯은 다음 CGT 수신 또는 CGT 후 0.6 s 에 기록(그때까지 오프셋 보정) |
| `timestamp_utc` | 런 시작 UTC + `t_sim_s` |
| `nano_millis`, `nano_boot_epoch`, `telemetry_seq`, `pi_received_utc` | Nano 가동시간(ms), 재부팅 구간 번호, CGT seq, Pi 수신 시각 (원시 근거) |
| `record_status` | `OK` 또는 `MISSING_TELEMETRY`. **1Hz 슬롯마다 1레코드** — CGT 가 없는 슬롯(손실·CRC 오류·통신 단절)은 측정값 null 의 결측 레코드로 남는다(0 채움 없음). 결측 슬롯은 텔레메트리 재개 시/종료 시 기록(`--live-missing-s` 로 실시간 기록 가능) |
| `cell_voltage_v`, `cumulative_tap_voltage_v` | Nano 누적 탭 차분 (무효 = null) |
| `nano_adc_tap_raw`, `nano_adc_ntc_raw` | **CGT 직후 요청한 RAW(CGD)** 의 A0..A3 / A4..A7 원시값. CGT 수신 후 0.19 s 안에 온 CGD 만 같은 샘플로 결합, 아니면 null (`--no-raw` 로 끔) |
| `cell_temperature_c`, `temperature_quality` | NTC 포화/단선 구분 불가 → null + `SATURATED_OR_OPEN` |
| `pack_current_a`, `current_quality`, `ina219_sampled_utc`, `current_age_s` | INA219: Nano 샘플 시점보다 5 ms 이상 앞서 측정되고 CGT 수신 시각까지 Pi 가 받은 읽기 중 최신. 나이(Nano 샘플 시점 기준) 0.5 s 초과 시 `MISSING`. 미설치 = null + `INA219_UNAVAILABLE` |
| `power_stage_requested` (확장) | 열려 있는 충방전 요청과 명령: `{"command":"RAMP"\|"DRAIN"\|"CHARGE"\|"DISCHARGE","direction","target_mv_per_cell"(RAMP/DRAIN 의 휴지 목표, 그 외 null),"since_utc"}` 또는 null |
| `power_path_reported` (확장) | Nano 가 **보고한** 전력 경로 활성 구간(CGE `CHARGE_START`/`DRAIN_START`·`DISCHARGE_START` ~ `*_STOP`). 0.1.6 에서 `charge_commanded`/`discharge_commanded` 는 CGT 비트 그대로 = 120 ms ON / 80 ms OFF 펄스의 **순간** MOSFET 상태이고, 측정은 OFF 구간에 하므로 대개 false 다. 단계 진행 여부는 이 필드를 본다 |
| `fault_requested`, `power_requested`, `fault_request_last_ended` | Pi 가 요청해 아직 유효한 것(슬롯 시각 기준). 호스트 정지·OFF/TRIP/SYNC·Nano 자체 정지(수신한 CGE: TIMEOUT/TRIP/한계)·재부팅(`NANO_RESET`)·거절(`REJECTED_<사유>`)로 종료 |
| `fault_commanded`, `*_commanded`, `command_state_source` | Nano 가 출력에 명령한 상태와 그 출처(`CGT+CGS`, `CGT+LAST_KNOWN_CGS`, 결측 슬롯은 `LAST_KNOWN`) |
| `fault_modeled_applied`, `fault_physical_observed` | 실측에는 모델이 없고 물리 피드백도 없으므로 항상 null. `physical_fault_verified=false` |
| `controller_state`, `controller_locks`, `trip_reason`, `trip_code` | Nano CGS/CGT |
| `voltage_quality`, `voltage_quality_bits`, `temperature_quality_mask` | 품질 코드와 원본 비트 |
| `transport` | 직전 레코드 이후 CRC 오류·중복·형식 오류 수 |
| `source`, `scenario_domain` | 실측: `measured_normal` / `measured_fault_injection`, `measured_hardware`. 펌웨어 호스트 시뮬레이터: `synthetic`, `simulation_only` |

합성 쪽에만 있는 필드: `generator_version`, `seed`, `config_hash`, `hardware_operable`(모델 분류), `fault_modeled_applied` 값.
실측 쪽에만 있는 필드: `firmware_version`, `hardware_revision`, `raw_log_ref`.

## 이벤트 (fault_event_v1)

0.1.6 반영 (스키마 필드 추가 없음, `evidence` 안에만 확장):
- `REQUESTED` + `evidence.command` = `RAMP`/`DRAIN`, `requested_value` = 휴지 목표 mV/셀 (`evidence.unit="mV_per_cell"`). 거절은 `FAULT_REJECTED`.
- `DRAIN_START`/`DRAIN_STOP` 는 `COMMANDED`. 단계 정지(`CHARGE_STOP`/`DRAIN_STOP`)에는 `evidence.stage_stop`
  = `{stage, stop_class: TARGET_REACHED|ABORTED|LIMIT|TRIP|HOST_STOP, nano_target_reached, abort_reason}`.
  `LOAD_TAP_UNSTABLE`·`TIMEOUT`·`BUDGET` → `ABORTED` (실패). Nano 가 목표에 도달했어도 성공 판정은 Pi 의 휴지 전압 확인(`CYCLE_STAGE`)까지다.
- `TRIPPED` 에는 직전 `CGX` 가 같은 trip 이면 `evidence.trip_snapshot`(트립을 일으킨 원시 ADC·탭 전압·품질 비트).

## 진단 (nano_diagnostic_v1, `diagnostics-*.jsonl`)

측정값도 고장 이벤트도 아닌 것을 따로 둔다 (`schemas/nano_diagnostic_v1.schema.json`):

| kind | 출처·의미 |
|---|---|
| `LOAD_TAP_SUSPECT_SNAPSHOT` | `CGY`: MOSFET ON 상태에서 역전처럼 보인 탭 값. `rested_cell_voltage=false` — **셀 전압으로 쓰지 않는다**(samples 에 들어가지 않음) |
| `TRIP_SNAPSHOT` | `CGX`: 실제 하드웨어 trip 의 원인 측정값 (`hardware_trip=true`) |
| `REST_VERIFY` | `LOAD_TAP_VERIFY`: `REST_OK` = 무부하 전압 정상 복귀, **단계는 실패**(`stage_successful=false`) / `REST_INVERTED` = 무부하에서도 역전 → TRIP |
| `NANO_EVENT` | `LOAD_TAP_SUSPECT`, `LIVE_GUARD`, `CHECKPOINT`, `PULSE_TOTAL`, `RAMP_TARGET`, `DRAIN_TARGET_SET` |
| `STAGE_STOP` | 단계 정지와 분류 (위 `stage_stop` 과 같음) |
| `CYCLE_STAGE` / `CYCLE_RESULT` | `cycle` 의 단계별·전체 판정 (PASS / ABORTED, 전후 셀 전압, 사유) |

CRC 가 맞는 `CGY`/`CGX` 는 정상 프레임이다(오류로 세지 않음). 파서가 모르는 종류라도 CRC 가 맞으면 오류가 아니라
`unknown_frames` 로만 세고 원문은 raw 로그에 남긴다.

`events-*.jsonl`: `REQUESTED`(이 Pi 가 보낸 명령), `FAULT_REJECTED`(CGA ERR), `COMMANDED`(CGE *_START/_STOP/_SET, RELAY_*, ARM/DISARM),
`TRIPPED`, `RESET`, `BOOT`. 시각은 Nano 이벤트 시각 추정치(`t_sim_s`), 근거는 `evidence.cge`.
링크에서 잃은 CGE 는 기록할 수 없으므로 eventSeq 공백을 세어 다음 이벤트의 `evidence.cge.events_lost_before` 로 남긴다.
중복 수신 CGE 는 한 번만 기록. `APPLIED/RELEASED/OBSERVABLE` 은 생성기 모델 전용(실측에는 없음).
생성기 `events.jsonl` 은 가상 Nano 내부 이벤트 기반 라벨 기록이라 프레임 손실 시나리오에서는 이 파일보다 이벤트가 많을 수 있다.

## 메모리·장시간 운용

- 수신 프레임은 구독자(수집기·raw 로그)로 즉시 전달 후 최근 256개만 유지, 샘플은 파일에 쓰고 최근 32개만 유지 → 1Hz × 7일(약 60만 샘플)에도 메모리 일정.
- 명령의 ACK 를 기다리는 동안 들어온 텔레메트리도 수집기로 전달된다 (누락 없음 — `test_no_telemetry_lost_while_commands_wait`).

## 제한 사항

- **실물 장비 시험은 하지 않았다.** 0.1.6 관련 시험은 실물 펌웨어 소스를 PC 로 빌드한 `sim016` 으로만 했고,
  그 플랜트 모델(5 F 셀, 22 Ω 충전 경로, 10 Ω 방전 저항, 부하 중 탭 오프셋)은 전기적 검증 자료가 아니다.
  첫 실물 사용은 `selftest` → `monitor` → 낮은 목표의 감독하 `cycle` 순서로 한다.
- `tests/fixtures/cellguard_cycle_20261025_132800.log` 는 sim016 으로 만든 로그다(첫 줄에 표시). 실제 Pi 로그로는 아직 변환해 보지 않았다.
  CGY 실측값 `352,696,612,955`(2026-10-25 벤치)는 파서 시험에만 썼다.
- `run`(계획 실행)·`shell` 은 0.1.6 의 RAMP/DRAIN 을 지원하지 않는다 — 단계 확인 절차가 있는 `cycle` 만 쓴다
  (`CHARGE 1` 등은 수집기가 거부하고 보내지 않는다).
- 0.1.6 은 부팅 자체점검이 없다. 부팅 점검은 이를 `UNSUPPORTED` 로 표시하고(전체 결과 PASS 불가), 구동 허용은 Pi 측 대체 점검이 모두 통과할 때만이다.

- INA219 모듈은 구매 목록에 없음. 션트값·부호·정격 실측 전에는 `--ina219` 를 쓰지 않는다 (데이터는 null 로 남음).
- 물리 고장 적용 여부는 이 수집기로 검증할 수 없다(명령 ≠ 물리). 실측 검증 절차는 펌웨어 저장소 `docs/calibration_and_bench_checklist.md`.
- 서버 업로더는 인터페이스만 제공.
