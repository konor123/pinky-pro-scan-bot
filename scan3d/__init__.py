"""스캔 PC 코드(docs/plan/PLAN.md).

공통
  protocol  스캔 PC ↔ 로봇 메시지(로봇 에이전트도 같은 파일을 쓴다: scripts/deploy.sh)
  clock     시계
  config    config/scan.yaml, 표식 판, 로봇 주소
  dataset   받은 원본 저장과 로봇별 Open3D 데이터셋
robot/  로봇 1대 단위(link, unit, drive, tagpose, align)
pair/   두 대 합침(coordinator, recon)
sim/    개발용 가짜(synth, sim2d, fake_agent)
station 주 프로그램(CLI)
"""
