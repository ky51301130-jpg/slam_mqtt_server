# 🚀 GitHub 저장소 생성 가이드

## 1. 로컬 Git 초기화

```bash
cd ~/ros2_ws/src/slam_mqtt_server

# Git 초기화
git init

# 모든 파일 추가
git add .

# 첫 커밋
git commit -m "🎉 Initial commit: SLAM MQTT Server

- unified_server: 맵 업로드 + 충돌 사진 + 네트워크 모니터링
- server_mqtt_bridge: ROS2 ↔ MQTT 양방향 통신
- nav2_map_builder: 8맵 ICP 정렬 + 과반수 투표 병합
- ai_vision_analyzer: ArUco + YOLO 장애물 감지
- Grafana/InfluxDB 모니터링 지원"
```

## 2. GitHub 저장소 생성

### 방법 A: GitHub 웹에서 생성

1. https://github.com/new 접속
2. Repository name: `slam_mqtt_server`
3. Description: `Pinky 로봇 서버 - MQTT 브릿지, 맵 병합, AI 비전, 모니터링`
4. **Private** 또는 **Public** 선택
5. **Create repository** 클릭
6. 생성 후 아래 명령어 실행:

```bash
git remote add origin https://github.com/ky51301130-jpg/slam_mqtt_server.git
git branch -M main
git push -u origin main
```

### 방법 B: GitHub CLI 사용

```bash
# GitHub CLI 설치 (이미 설치된 경우 생략)
sudo apt install gh

# 로그인
gh auth login

# 저장소 생성 및 푸시
gh repo create slam_mqtt_server --public --source=. --push \
  --description "Pinky 로봇 서버 - MQTT 브릿지, 맵 병합, AI 비전, 모니터링"
```

## 3. 저장소 설정 (선택)

### Topics 추가
GitHub 저장소 → Settings → About → Topics:
- `ros2`
- `mqtt`
- `slam`
- `navigation`
- `robotics`
- `python`
- `grafana`
- `yolo`

### README 배지 추가 (선택)

```markdown
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Python](https://img.shields.io/badge/Python-3.12+-yellow)
![License](https://img.shields.io/badge/License-MIT-green)
```

## 4. 로봇 저장소와 연동

기존 로봇 저장소 README에 링크 추가:

```markdown
## 🔗 관련 저장소

- 🖥️ **서버**: [slam_mqtt_server](https://github.com/ky51301130-jpg/slam_mqtt_server)
- 🤖 **로봇**: [slam_mqtt_project](https://github.com/ky51301130-jpg/slam_mqtt_project)
```

## 5. SSH 키 설정 (선택)

HTTPS 대신 SSH 사용 시:

```bash
# SSH 키 생성
ssh-keygen -t ed25519 -C "your_email@example.com"

# 공개키 복사
cat ~/.ssh/id_ed25519.pub

# GitHub → Settings → SSH Keys → New SSH Key에 붙여넣기

# remote 변경
git remote set-url origin git@github.com:ky51301130-jpg/slam_mqtt_server.git
```

---

## 📁 현재 디렉토리 구조

```
slam_mqtt_server/
├── slam_mqtt_server/         # Python 패키지
│   ├── __init__.py
│   ├── config.py             # 중앙 설정
│   ├── unified_server.py     # 통합 서버
│   ├── server_mqtt_bridge.py # MQTT 브릿지
│   ├── nav2_map_builder.py   # 맵 병합
│   └── ai_vision_analyzer.py # AI 비전
├── launch/                   # ROS2 런치 파일
│   ├── unified.launch.py
│   ├── slam_rviz.launch.py
│   └── nav2_rviz.launch.py
├── rviz/                     # RViz 설정
├── docs/                     # 문서
├── config/                   # 설정 파일
├── scripts/                  # 스크립트
├── resource/                 # ROS2 리소스
├── package.xml               # ROS2 패키지 정의
├── setup.py                  # Python 설정
├── setup.cfg
├── requirements.txt          # Python 의존성
├── .gitignore
├── README.md                 # 📌 메인 문서
└── GITHUB_SETUP.md           # 이 파일
```

---

**완료 후 확인:**
```bash
# 저장소 상태 확인
git status
git remote -v
git log --oneline

# 브라우저에서 확인
xdg-open https://github.com/ky51301130-jpg/slam_mqtt_server
```
