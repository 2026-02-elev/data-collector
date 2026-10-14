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
  fusion.py     CGT+CGS+CGD+전류 → normalized_sample_v1 (1Hz 슬롯, 결측 슬롯, 실험 경과시간), 메모리 상한
  events.py     명령/CGE → fault_event_v1 (REQUESTED/FAULT_REJECTED/COMMANDED/TRIPPED/RESET/BOOT)
  replay.py     raw UART 로그(수집기 raw-*.jsonl 또는 생성기 nano_uart.log) → samples/events 재생성
  storage.py    RotatingWriter(spool → outbox, 크기/시간 회전, 비정상 종료 .part 검증 복구), Outbox + Uploader 인터페이스
  runner.py     실험 시퀀스 실행기(한 세션 유지), 대화형 shell 용 Pump 스레드
  cli.py        cg-collect monitor | run | shell | query | outbox | replay
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

# raw 로그에서 samples/events 재생성 (파서 수정 후 재처리, 생성기 출력 검증)
cg-collect replay --from-dir /var/lib/cellguard/outbox --out rebuilt/           # raw-*.jsonl + ina219-*.jsonl
cg-collect replay --from-raw raw-*.jsonl --ina219-log ina219-*.jsonl --out rebuilt/
cg-collect replay --from-generator ../data-generator/demo_outputs/runs/F-01_demo_esr_cell2__r00 --out rebuilt/

# 하드웨어 없이 (SIMULATED DEVICE — 기록은 source="synthetic")
cg-collect --sim ../arudino-firmware/tests/host/build/nano_host_sim --sim-gates unlocked \
           run examples/plan_charge_esr.json --i-am-supervising
```

Pi 준비: `raspi-config` → Interface Options → Serial Port (login shell **No**, serial hardware **Yes**), I²C 활성화(INA219), 40핀 리본 1번 방향·도통 확인 후 연결.

## 데이터 저장과 서버 전송

```text
<data-dir>/spool/samples-<시작UTC>-<n>.jsonl.part   기록 중 (레코드마다 flush)
<data-dir>/spool/raw-<시작UTC>-<n>.jsonl.part       첫 줄 RUN 헤더(run_start_utc 등) + 송수신 원본 프레임(양방향, dir=N>P|P>N) + Pi UTC(µs) + 파싱 오류
<data-dir>/spool/ina219-<시작UTC>-<n>.jsonl.part    (--ina219) INA219 읽기마다 1줄: ina219_raw_v1
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

`events-*.jsonl`: `REQUESTED`(이 Pi 가 보낸 명령), `FAULT_REJECTED`(CGA ERR), `COMMANDED`(CGE *_START/_STOP/_SET, RELAY_*, ARM/DISARM),
`TRIPPED`, `RESET`, `BOOT`. 시각은 Nano 이벤트 시각 추정치(`t_sim_s`), 근거는 `evidence.cge`.
링크에서 잃은 CGE 는 기록할 수 없으므로 eventSeq 공백을 세어 다음 이벤트의 `evidence.cge.events_lost_before` 로 남긴다.
중복 수신 CGE 는 한 번만 기록. `APPLIED/RELEASED/OBSERVABLE` 은 생성기 모델 전용(실측에는 없음).
생성기 `events.jsonl` 은 가상 Nano 내부 이벤트 기반 라벨 기록이라 프레임 손실 시나리오에서는 이 파일보다 이벤트가 많을 수 있다.

## 메모리·장시간 운용

- 수신 프레임은 구독자(수집기·raw 로그)로 즉시 전달 후 최근 256개만 유지, 샘플은 파일에 쓰고 최근 32개만 유지 → 1Hz × 7일(약 60만 샘플)에도 메모리 일정.
- 명령의 ACK 를 기다리는 동안 들어온 텔레메트리도 수집기로 전달된다 (누락 없음 — `test_no_telemetry_lost_while_commands_wait`).

## 제한 사항

- INA219 모듈은 구매 목록에 없음. 션트값·부호·정격 실측 전에는 `--ina219` 를 쓰지 않는다 (데이터는 null 로 남음).
- 물리 고장 적용 여부는 이 수집기로 검증할 수 없다(명령 ≠ 물리). 실측 검증 절차는 펌웨어 저장소 `docs/calibration_and_bench_checklist.md`.
- 서버 업로더는 인터페이스만 제공.
