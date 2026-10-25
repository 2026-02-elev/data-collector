# 0.1.6-LOAD-VERIFY test harness (SIMULATED DEVICE)

작성: Yongho Ahn <nanhosoft@gmail.com>

- `CellGuardNano_LoadVerify/` — 실물 Nano 에 올라가 있는 펌웨어 `0.1.6-LOAD-VERIFY` 소스와 `cellguard_cycle.py`,
  `README_LOAD_VERIFY.md`. 사용자가 제공한 `CellGuardNano_LoadVerify_0.1.6.zip` 에서 **바이트 그대로** 복사했다
  (`SHA256SUMS`). 수정·업로드 대상이 아니라 시험의 기준(프로토콜·동작의 근거)이다.
- `sim016.cpp` — 위 컨트롤러 코드를 그대로 PC 용으로 컴파일하고(`-DCG_HOST_SIM_BUILD=1` → CGV flags bit9, 기록은
  `synthetic`), 펌웨어에 동봉된 `validation/load_verify_smoke.cpp` 의 플랜트 모델을 붙인 시뮬레이터.
  stdin 프로토콜은 `arudino-firmware/tests/host/nano_host_sim` 과 같아 `SimTransport` 로 그대로 쓴다.
  전기적 모델이 아니며 실물 검증을 대신하지 않는다.
- `make_cycle_log_fixture.py` — `tests/fixtures/cellguard_cycle_20261025_132800.log` 재생성 (sim016 + 방전 중 부하 탭 역전 주입).

빌드는 `tests/conftest.py` 의 `sim016_path` 픽스처가 자동으로 한다 (g++ 필요, 결과물 `build/` 는 git 제외).
