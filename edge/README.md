# cellguard-edge v3 (Raspberry Pi): 운영용 엣지 프로그램

Pi Zero 2 W에서 상시 돌아가는 프로그램이다. Nano(펌웨어 `0.2.0-RUNTIME`, `0.1.9-RUNTIME`도 동작)와 UART로 통신하고, 측정값을 MQTT TLS로 서버(`mqtts://mqtt.misakachan.moe:8883`)에 올린다.

1. **AI 판정**: MLOps에서 학습한 모델(residual, classifier, load_esr)로 1초마다 셀 상태를 판정한다. 이상이 확정되면 서버에 보고한다.
2. **모델 OTA**: MLOps의 운영(production) 버전이 바뀌면 새 모델을 받아 검증한 뒤 교체한다.
3. **모드 전환**: 랜덤 고장 모드(단일·복합 고장)와 데이터 수집 모드를 오간다. 전환은 Pi에서 명령으로 하거나 서버에서 MQTT로 한다.

이 폴더(`edge/`)가 실물 장비에서 쓰는 프로그램이다. 저장소 상위의 `cg_collector/`는 예전 펌웨어(0.1.6/0.2.0)용 감독형 수집기다.

## 모드

| 모드 | 하는 일 |
| --- | --- |
| `collect` | 계통 수집 사이클을 돈다. 충전(600/700/800 mV) → 자극 → 균형 → 방전 순서다. 자극(control, ESR, 누설, 과열)과 셀을 차례로 바꾼다. 예전 `auto` 모드와 같다 |
| `fault` | 랜덤 고장 에피소드를 돈다. 구성은 고장 없음 15 %, 단일 고장 45 %, 복합 고장 40 %다. 에피소드가 끝나면 AI가 무엇을 맞혔는지 채점한다 (`EPISODE_RESULT`) |
| `passive` | 출력을 끄고 측정·전송·AI 판정만 한다 |

AI 판정은 세 모드 모두에서 돈다.

### 랜덤 고장 에피소드 (`fault`)

순서: SELFTEST/SESSION → 기준 측정 30 s → 충전 → 유지 20 s → 고장 주입 → 관찰 45 s → 열 식힘 → 균형 → 방전 → 휴지 30 s

- **복합 고장**은 종류 2개를 고른다(25 % 확률로 3개). 셀은 50 %는 한 셀에 모으고, 50 %는 셀마다 따로 고른다.
- **Nano `0.2.0-RUNTIME` 이상이면 복합 고장을 동시에 넣는다** (`fault_overlap`, 기본 켬).
  - 과열·누설을 함께 켜고, 둘 다 켜진 동안 ESR 부하 펄스를 넣는다.
  - 펄스는 `DISCHARGE 0` + `ESR n 0`으로 끝낸다. OFF는 다른 고장까지 모두 끄기 때문에 쓰지 않는다.
  - 과열·누설은 각자 정해진 시간에 `HEAT n 0` / `LEAK n 0`으로 끈다. 펌웨어도 `heat_s`·`leak_s`로 따로 끊는다.
  - 마지막에 OFF로 전부 끈다.
  - 동시 한도: ESR 2셀, LEAK 2셀, HEAT 1셀. 고장이 켜져 있는 동안 RAMP(충전)는 펌웨어가 거절한다.
  - 하트비트가 5 s 끊기면 전부 꺼진다.
- **`0.1.9-RUNTIME`은 액추에이터를 한 번에 하나만 허용한다.** 그래서 쉬는 시간 없이 연달아 넣어 효과가 겹치게 한다.
  - 과열을 맨 앞에 둔다. 히터를 끈 뒤에도 온도 상승이 수십 초 남아서, 이어지는 누설·ESR과 겹친다.
- `EPISODE_RESULT`의 `overlap`이 true면 동시 주입이다.
- 누설·과열은 펌웨어 한계 시간(`leak_s`, `heat_s`)의 50~100 % 중 무작위로 정한 시점에 끊는다.
- **ESR 주입은 짧은 부하 펄스로 한다.** ESR은 전류가 흐를 때만 전압 강하로 드러나기 때문이다.
  - 펄스 순서: ESR 켜기 → 접점 안정 0.35 s → `DRAIN` 1 s → OFF
  - 펄스 동안 Nano가 부하 측정(CGP)을 보내고, `load_esr` 모델이 이를 판정한다.
  - ESR 없는 기준 펄스(control_load)를 먼저 한 번 넣는다.
  - 1초 펄스는 펌웨어 방전 무반응 검사(4 s)보다 짧다.
- 출력과 안전 한계는 v2와 같다. 판단은 펌웨어가 맡고, Pi는 스프레드 가드와 셀 하한을 지킨다.

## AI 판정과 보고

| 모델 | 입력 | 맡는 고장 (실측 학습 v1) |
| --- | --- | --- |
| `residual` ② | 1초 측정 | 과열 (온도 상승 잔차). 실측 탐지율 100 %, 지연 약 19 s, 시간당 오경보 0 |
| `classifier` ③ | 최근 20 s 창 | 누설, 과열 (이벤트 단위 100 %). ESR 정밀도가 낮아 ESR은 보고하지 않는다 |
| `load_esr` | 부하 측정(CGP)과 직전 휴지 전압 | ESR (부하 재현율 98 %, 셀 식별 100 %, 오경보 0 %) |

**새 모델 (2026-11-23)**: MLOps가 실측 복합 고장 데이터셋(2026-11-18\~20, 772 에피소드)으로 다시 학습했다.
Pi 판정 규칙대로 에피소드를 재생해 비교한 test 결과다 (MLOps `docs/results/2026-11-23_fault_dataset.md`).

| 패키지 | 이 코드 이전 버전에서 | test 결과 (v1 대비) |
| --- | --- | --- |
| `residual-v5`, `load_esr-v2` | 설치됨 (형식 같음) | 내부저항 탐지 81 → 88 %, 오경보 104 → 65건 |
| 위 + `classifier-v3` (멀티레이블) | 이 버전부터 읽음 (이전 버전은 테스트 벡터에서 거절) | 오경보 104 → 12건, 정상 에피소드 오경보 0/27 |

- **멀티레이블 ③** (`classifier_type: multilabel`): ESR·LEAK·HEAT 독립 확률이고, 모델 파일의 `thresholds` 이상이면 그 고장이다.
  - 셀마다 `labels`(고장 목록)를 낸다. 한 셀의 누설+과열도 같이 낸다.
  - `label`은 셀마다 대표 하나다. 기존 단일 레이블 모델은 예전과 같다.

- **판정 확정 규칙**:
  - residual: 경보 3회 연속
  - classifier: 같은 고장이 3창 연속이고 확률 0.6 이상
    - 멀티레이블 모델은 셀·고장마다 따로 센다. 모델 임계값 이상이 3창 연속이면 확정하고, 한 셀에 둘 이상 확정될 수 있다.
  - load_esr: 1회
- **해제 규칙**: 경보 없음이 10회 이어지면 해제한다. 같은 (모델, 셀)은 해제 전까지 다시 보고하지 않는다.
- **추론 부담**: Pi Zero 2 W에서 1초 측정 한 번에 약 26 ms. 수신 스레드와 따로 돈다.
- **정답(ground truth)**: Pi는 자신이 무엇을 주입했는지 알고 있다. CGT·CGP의 액추에이터 마스크(esr, leak, heat)로 셀별 주입 시점을 기록하고, 판정마다 맞혔는지 붙인다. `match` 값은 다음과 같다.
  - `injected`: 주입한 고장을 맞힘
  - `balance`: 균형용 방전 셀을 누설로 본 경우
  - `none`: 근거 없는 판정 (오경보)
  - 인정 시간은 과열 180 s, 누설 60 s, ESR 30 s다.

**서버로 가는 메시지** (모두 `cellguard/v1/{device}/…`, QoS 1, 오프라인이면 outbox에 쌓였다가 보낸다)

| 토픽 | kind / event_type | 내용 |
| --- | --- | --- |
| `diagnostics` | `AI_DETECTION`, `AI_CLEARED` | 모델, 모델 버전, 셀, 고장 종류, 점수, `match`, ground_truth, 모드, 에피소드 |
| `events` | `OBSERVABLE` | ESR, LEAK, HEAT 판정. 같은 (종류, 셀)은 120 s에 한 번만 보낸다. `details.source = "edge_ai"` |
| `diagnostics` | `EPISODE_START`, `EPISODE_RESULT` | 에피소드 계획, 실제 주입한 고장별 탐지 여부·지연, 오경보 목록 |
| `diagnostics` | `OTA_CHECK`, `OTA_RESULT` | OTA 확인 결과와 적용, 실패, 되돌림 |
| `diagnostics` | `COMMAND_ACK`, `MODE_REQUEST`, `MODE_START` | 원격 명령 처리, 모드 전환 |
| `telemetry` | `normalized_sample_v1` | `scenario_id = edge_collect_v3`, `edge_fault_v3` 또는 `edge_passive_v3`. 에피소드 ID는 `cycle_id`(`…:e00012`)에 담긴다 |

## 모델 OTA

- **확인 주기**: 시작 30 s 뒤, 그다음 1시간마다 MLOps에 확인한다. `ota` 명령이나 MQTT로 바로 확인시킬 수도 있다.
  - 경로: `{mlops_url}/models/{name}/production` → `/models/{name}/{ver}/package`
- **설치 전 검증**:
  1. 패키지 SHA-256 (`X-Package-SHA256`)
  2. manifest의 파일별 체크섬
  3. `test_vectors.json`: 서버가 만든 입력과 기대 출력을 Pi 코드로 다시 계산해 같은 값인지 본다 (학습-서빙 불일치 방지)
- **설치**: `models/<name>/current`로 바꾸고, 직전 버전은 `previous`에 남긴다. 새 모델이 실제 설정으로 로드되지 않으면 되돌린다.
- **보고**: 결과는 MLOps `POST /edge/versions`와 MQTT `OTA_RESULT`로 보낸다.
- **서버 경로**: OCI nginx의 `/edge-api/`가 MLOps(127.0.0.1:8400)의 엣지용 경로 4개만 연다. 헤더 `X-Edge-Token`이 맞아야 한다 (`deploy/nginx-cellguard.conf`). Pi는 토큰을 `secrets.env`의 `EDGE_TOKEN=`에서 읽는다.
- **서버 없이 설치**: `cellguard-edge ota-install residual residual-v1.zip`. 같은 검증을 거친다. 설치하면 서비스가 바로 다시 읽는다.
  - 새 모델: `ota-install residual residual-v5.zip`, `ota-install load_esr load_esr-v2.zip`, `ota-install classifier classifier-v3.zip`

## 설치 / 갱신 (Pi의 pi 계정, sudo 불필요)

```
scp cellguard_edge.py cg_ai.py cellguard-edge.service install.sh cellguard:~/edge-src/
scp tests/fixtures/*-v1.zip cellguard:~/edge-src/models/      # (선택) 서버 없이 v1 모델 설치
scp tests/fixtures/residual-v5.zip tests/fixtures/load_esr-v2.zip tests/fixtures/classifier-v3.zip cellguard:~/edge-src/models/   # (선택) 새 모델
ssh cellguard 'sh ~/edge-src/install.sh'
```

- **필요 패키지**: `python3-paho-mqtt`, `python3-numpy` (Raspberry Pi OS 기본 저장소). numpy가 없으면 AI만 꺼지고 수집은 계속한다.
- **서비스**: systemd **user** 유닛 `cellguard-edge.service`
  - linger=yes라 부팅하면 자동으로 시작한다.
  - 죽으면 10초 뒤 재시작하고, watchdog은 120초다.
- **`~/cellguard-edge/secrets.env`** (600): `MQTT_USERNAME=`, `MQTT_PASSWORD=`, `EDGE_TOKEN=`. 레포에 넣지 않는다.
- **설정 변경**: `~/cellguard-edge/config.json`에 바꿀 키만 적는다. 기본값은 코드의 `DEFAULT_CONFIG`에 있다. 예:
  ```json
  {"fault_weights": {"none": 0.1, "single": 0.4, "compound": 0.5}, "ota_interval_s": 1800}
  ```

## 사용

```
cellguard-edge mode fault        # 랜덤 고장 모드
cellguard-edge mode collect      # 데이터 수집 모드
cellguard-edge pause             # 출력 OFF, 측정·AI만 (지금 모드 기억)
cellguard-edge resume            # pause 전 모드로 (자동 일시정지도 즉시 해제)
cellguard-edge ota               # MLOps 새 모델 지금 확인
cellguard-edge ota-install NAME FILE.zip
cellguard-edge status            # 상태 (모드, 에피소드, AI 모델 버전·최근 판정, OTA 결과, MQTT)
journalctl _SYSTEMD_USER_UNIT=cellguard-edge.service -f    # 로그 (journalctl --user 는 비어 보임)
```

ssh로 명령 하나만 실행할 때는 `~/.local/bin`이 PATH에 없으므로 `python3 ~/cellguard-edge/cellguard_edge.py mode fault`처럼 쓴다.

모드를 바꾸면 진행 중인 사이클이나 에피소드를 출력 OFF로 끊고, 새 모드를 바로 시작한다. 모드는 `data/control.json`에 남아 재부팅 뒤에도 유지된다.

**서버에서 원격 명령**: `cellguard/v1/cellguard-pi-01/command`에 JSON을 보낸다. 처리 결과는 diag `COMMAND_ACK`로 돌아온다.

```
{"command": "mode", "mode": "fault"}      # collect | fault | passive
{"command": "pause"}  {"command": "resume"}
{"command": "ota_check"}
{"command": "status"}
```

## 시험

```
python -m pytest edge/tests -q
```

| 파일 | 다루는 것 |
| --- | --- |
| `test_ai.py` | OTA 패키지 검증, 설치, 되돌리기, 테스트 벡터 불일치 거부, 판정 로직(과열 셀만 탐지, load_esr, 분류기 확정·해제), 새 패키지 테스트 벡터, 멀티레이블 ③(셀별 고장 목록, 잘못된 설정 거부, 셀·고장별 확정·해제) |
| `test_edge.py` | 모드 전환, 랜덤 고장 계획(복합·과열 먼저), 펌웨어 버전 판별, 동시 복합 고장 명령 순서(펄스는 OFF 없이, 고장별 정지), 판정과 정답 매칭·중복 억제, 균형 방전 구분, 에피소드 채점, CGT/CGP → AI 입력, 원격 명령 |

`tests/fixtures/*-v1.zip`은 MLOps가 실측 15세션(2026-11-07~10)으로 학습해 만든 실제 OTA 패키지다.
`residual-v5.zip`, `load_esr-v2.zip`, `classifier-v3.zip`은 실측 복합 고장 데이터셋으로 학습한 MLOps 서버의 패키지다 (② v5·부하 판정 v2는 MLOps 운영 버전, ③ v3는 엣지가 읽을 수 있게 되면 승격).

## 데이터 (Pi)

| 위치 | 내용 |
| --- | --- |
| `data/raw/uart-YYYYMMDD.log` | UART 원본 송수신 전부. 지난 날짜는 gzip으로 묶고 30일 보관한다 |
| `data/outbox.sqlite3` | 서버 PUBACK을 아직 못 받은 메시지. 최대 100만 건 |
| `data/status.json` | 상태 스냅샷. 2초마다 갱신한다 |
| `data/control.json` | 모드, OTA 요청, 모델 다시 읽기 요청 |
| `data/state.json` | 사이클·에피소드 번호 |
| `models/<name>/{current,previous}` | 설치된 모델 |

## 트립과 일시정지 (v2와 같음)

- **펌웨어 트립**: 출력 OFF → 30초 대기 → RESET → 다음 사이클을 SELFTEST/SESSION부터 다시 시작한다. 진행 중이던 에피소드는 `aborted`로 채점한다.
- **트립 반복**: 1시간 안에 트립이 3번 나면 1시간 동안 실험을 멈춘다 (`TRIP_PAUSE`).
- **충전 무반응** (`CHARGE_NO_RESPONSE`)이 2번 연속 나면 충전 경로가 끊긴 것으로 보고 30분 멈춘다.
  - 다시 실패하면 1시간, 2시간, 최대 4시간으로 늘린다.
  - 충전 경로를 확인한 뒤 `resume`하면 바로 다시 시도한다.
  - 충전 경로: 12 V → MP1584 → `+5V_Cell` → Q3 → R17 → H4(INA219 션트) → RLY2 → FH1 → 스택 최상단
