#!/bin/bash
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SLAM MQTT Server - 모니터링 스택 설치 스크립트
# InfluxDB v2 + Telegraf + Grafana
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

set -e

# 색상
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  SLAM MQTT Server - 모니터링 스택 설치${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

# 1. Mosquitto MQTT 브로커
echo -e "\n${YELLOW}[1/4] Mosquitto MQTT 브로커 설치...${NC}"
sudo apt update
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable mosquitto --now
echo -e "${GREEN}✅ Mosquitto 설치 완료${NC}"

# 2. InfluxDB v2
echo -e "\n${YELLOW}[2/4] InfluxDB v2 설치...${NC}"
if ! command -v influx &> /dev/null; then
    wget -q https://dl.influxdata.com/influxdb/releases/influxdb2-2.7.4-amd64.deb
    sudo dpkg -i influxdb2-2.7.4-amd64.deb
    rm influxdb2-2.7.4-amd64.deb
fi
sudo systemctl enable influxdb --now
echo -e "${GREEN}✅ InfluxDB 설치 완료${NC}"

# InfluxDB 초기 설정
echo -e "\n${YELLOW}InfluxDB 초기 설정...${NC}"
sleep 3
if ! influx bucket list --org slam_org &> /dev/null 2>&1; then
    influx setup \
        --username admin \
        --password adminadmin \
        --org slam_org \
        --bucket slam_data \
        --retention 7d \
        --force || echo "InfluxDB 이미 설정됨"
fi

# 토큰 생성/확인
INFLUX_TOKEN=$(influx auth list --org slam_org 2>/dev/null | grep "admin's Token" | awk '{print $2}' || echo "")
if [ -z "$INFLUX_TOKEN" ]; then
    INFLUX_TOKEN=$(influx auth create --org slam_org --description "Telegraf" --read-buckets --write-buckets 2>/dev/null | tail -1 | awk '{print $2}')
fi
echo -e "   토큰: ${INFLUX_TOKEN:0:20}..."

# 3. Telegraf
echo -e "\n${YELLOW}[3/4] Telegraf 설치...${NC}"
sudo apt install -y telegraf

# Telegraf 설정 복사
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(dirname "$SCRIPT_DIR")/config"
if [ -f "$CONFIG_DIR/telegraf.conf" ]; then
    # 토큰 치환 후 복사
    sudo sed "s/\$INFLUX_TOKEN/$INFLUX_TOKEN/g" "$CONFIG_DIR/telegraf.conf" > /tmp/telegraf.conf
    sudo cp /tmp/telegraf.conf /etc/telegraf/telegraf.conf
    rm /tmp/telegraf.conf
    echo -e "   설정 파일 복사 완료"
fi

sudo systemctl enable telegraf --now
sudo systemctl restart telegraf
echo -e "${GREEN}✅ Telegraf 설치 완료${NC}"

# 4. Grafana
echo -e "\n${YELLOW}[4/4] Grafana 설치...${NC}"
if ! command -v grafana-server &> /dev/null; then
    sudo apt install -y apt-transport-https software-properties-common
    wget -q -O - https://packages.grafana.com/gpg.key | sudo apt-key add -
    echo "deb https://packages.grafana.com/oss/deb stable main" | sudo tee /etc/apt/sources.list.d/grafana.list
    sudo apt update
    sudo apt install -y grafana
fi
sudo systemctl enable grafana-server --now
echo -e "${GREEN}✅ Grafana 설치 완료${NC}"

# 완료 메시지
echo -e "\n${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  ✅ 모니터링 스택 설치 완료!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "📊 ${YELLOW}Grafana${NC}:      http://localhost:3000  (admin/admin)"
echo -e "💾 ${YELLOW}InfluxDB${NC}:    http://localhost:8086  (admin/adminadmin)"
echo -e "📡 ${YELLOW}MQTT${NC}:        localhost:1883"
echo ""
echo -e "다음 단계:"
echo -e "  1. Grafana 접속 → Data Sources → InfluxDB 추가"
echo -e "  2. Import Dashboard: ${CONFIG_DIR}/grafana_dashboard.json"
echo -e "  3. ROS2 시스템 실행: ros2 launch slam_mqtt_server unified.launch.py"
echo ""
