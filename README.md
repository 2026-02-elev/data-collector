# CellGuard 데이터 수집기 (Raspberry Pi)

- 작성/구현: **Yongho Ahn** <nanhosoft@gmail.com>
- 역할: Arduino Nano(CellGuard 펌웨어, UART 프로토콜 v1.0) 제어·텔레메트리 수집, Pi I²C INA219 팩 전류 수집, 시간정합 후 `normalized_sample_v1` 저장, 서버 업로드용 outbox 관리
- 연계 저장소: 펌웨어 `2026-02-elev/arudino-firmware` (프로토콜 정의 `docs/uart_protocol.md`, 호스트 시뮬레이터 `tests/host`), 합성 데이터 `2026-02-elev/data-generator` (같은 스키마)

> **안전 고지.** 액추에이션(ARM/CHARGE/DISCHARGE/ESR/LEAK/HEAT/RESET/BENCH)은 `--i-am-supervising` 이 없으면 수집기가 거부하며,
> 펌웨어는 실물 검증 게이트가 잠겨 있으면 `ERR,HARDWARE_LOCKED` 로 별도 거부한다. ACK `OK` 는 "명령 수락" 이지
> 릴레이·MOSFET·히터가 물리적으로 동작했다는 뜻이 아니다. 이 프로그램은 무인 충방전/히터 운용을 승인하지 않는다.

## 구조

```text
cg_collector/
  protocol.py   프레임/CRC16/메시지 파서 (CGA CGT CGS CGD CGE CGV CGK)
  transport.py  SerialTransport(115200 8N1) / SimTransport(펌웨어 호스트 시뮬레이터)
  session.py    NanoSession: seq·같은 seq 재전송·ACK 매칭·하트비트, 모든 수신 프레임을 구독자에 전달,
                FaultTracker: 요청(requested) / 명령(commanded) / 물리(미검증) 상태 분리
  current.py    INA219 드라이버 + CurrentSampler(백그라운드 10Hz, 이력 버퍼, 시간정합)
  fusion.py     CGT+CGS+전류 → normalized_sample_v1, Collector(즉시 기록, 메모리 상한)
  storage.py    RotatingWriter(spool → outbox, 크기/시간 회전), Outbox + Uploader 인터페이스
  runner.py     실험 시퀀스 실행기(한 세션 유지), 대화형 shell 용 Pump 스레드
  cli.py        cg-collect monitor | run | shell | query | outbox
schemas/        normalized_sample_v1, fault_event_v1 (합성기와 공통 계약)
examples/       plan_charge_esr.json (감독하 실험 계획 예시)
tests/          단위시험 + 펌웨어 호스트 시뮬레이터 통합시험
```

## 설치 / 시험

```bash
pip install -r requirements-dev.txt        # Pi 에서 INA219 사용 시: pip install smbus2
pip install -e .                           # cg-collect 명령 설치 (또는 python3 -m cg_collector ...)

# 통합시험은 펌웨어 저장소의 호스트 시뮬레이터를 빌드해 사용한다 (g++ 필요).
# 기본 위치: ../arudino-firmware  (다른 위치면 CELLGUARD_FIRMWARE_DIR 지정)
python3 -m pytest -q
```

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
#   > arm / charge 1 / esr 2 1 / status / off / quit

# 단발 조회(읽기 전용): ping | status | raw | cal
cg-collect --port /dev/serial0 query status

# 하드웨어 없이 (SIMULATED DEVICE — 기록은 source="synthetic")
cg-collect --sim ../arudino-firmware/tests/host/build/nano_host_sim --sim-gates unlocked \
           run examples/plan_charge_esr.json --i-am-supervising
```

Pi 준비: `raspi-config` → Interface Options → Serial Port (login shell **No**, serial hardware **Yes**), I²C 활성화(INA219), 40핀 리본 1번 방향·도통 확인 후 연결.

## 데이터 저장과 서버 전송

```text
<data-dir>/spool/samples-<시작UTC>-<n>.jsonl.part   기록 중 (레코드마다 flush)
<data-dir>/spool/raw-<시작UTC>-<n>.jsonl.part       수신 원본 프레임 + Pi 수신 UTC + 파싱 오류
<data-dir>/outbox/*.jsonl                            회전 완료 → 서버 업로드 대기
<data-dir>/sent/*.jsonl                              업로더가 성공 확인한 파일
```

- 기본 회전: 50 MB 또는 1시간 (`--rotate-mb`, `--rotate-s`). 비정상 종료로 남은 `.part` 는 다음 실행 시 outbox 로 이동.
- 서버 전송은 `storage.Uploader.upload(path) -> bool` 를 구현해 `Outbox.flush(uploader)` 로 호출한다 (HTTP/MQTT 백엔드는 미구현 — 서버 API 확정 후 추가). 업로드 성공 확인 전에는 파일을 지우거나 옮기지 않는다.
- 샘플의 `raw_log_ref` 는 해당 시점의 raw 파일 이름 → 원본 프레임으로 추적 가능.

## 데이터 의미 (normalized_sample_v1 + 확장 필드)

| 필드 | 의미 |
|---|---|
| `source` | 실측: `measured_normal` / `measured_fault_injection`(명령된 고장이 있을 때). 시뮬레이터: `synthetic` |
| `cell_voltage_v`, `cumulative_tap_voltage_v` | Nano 누적 탭 차분 (무효 = null) |
| `cell_temperature_c`, `temperature_quality` | NTC 포화/단선 구분 불가 → null + `SATURATED_OR_OPEN` |
| `pack_current_a`, `current_quality`, `ina219_sampled_utc` | INA219 값. 텔레메트리 수신 시각 **이전** 샘플 중 최신, 0.5s 초과 시 `MISSING`. 미설치 = null + `INA219_UNAVAILABLE` (0A 아님) |
| `fault_requested`, `power_requested` | Pi 가 요청해 아직 유효한 것. Nano 가 자체 종료(TIMEOUT/TRIP/한계)하면 이벤트 수신 즉시 해제 |
| `fault_request_last_ended` | 마지막으로 끝난 요청과 종료 사유 |
| `fault_commanded`, `*_commanded` | Nano 가 출력에 명령한 상태 |
| `fault_physical_observed` | 항상 null — 고장별 물리 피드백 없음. `physical_fault_verified=false` |
| `controller_state`, `controller_locks`, `trip_reason` | Nano CGS/CGT |
| `nano_millis`, `nano_telemetry_seq`, `pi_received_utc` | 시간 정합 근거 |

## 메모리·장시간 운용

- 수신 프레임은 구독자(수집기·raw 로그)로 즉시 전달 후 최근 256개만 유지, 샘플은 파일에 쓰고 최근 32개만 유지 → 1Hz × 7일(약 60만 샘플)에도 메모리 일정.
- 명령의 ACK 를 기다리는 동안 들어온 텔레메트리도 수집기로 전달된다 (누락 없음 — `test_no_telemetry_lost_while_commands_wait`).

## 제한 사항

- INA219 모듈은 구매 목록에 없음. 션트값·부호·정격 실측 전에는 `--ina219` 를 쓰지 않는다 (데이터는 null 로 남음).
- 물리 고장 적용 여부는 이 수집기로 검증할 수 없다(명령 ≠ 물리). 실측 검증 절차는 펌웨어 저장소 `docs/calibration_and_bench_checklist.md`.
- 서버 업로더는 인터페이스만 제공.
