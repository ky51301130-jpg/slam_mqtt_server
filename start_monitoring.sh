#!/bin/bash
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# SLAM MQTT Server - 모니터링 스택 시작
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 
# 포함: Mosquitto(MQTT) + InfluxDB + Telegraf + Grafana
# 
# 사용법:
#   ./start_monitoring.sh        # 시작
#   ./start_monitoring.sh stop   # 중지
#   ./start_monitoring.sh logs   # 로그 보기
#   ./start_monitoring.sh status # 상태 확인
#
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 색상
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}  🤖 SLAM MQTT Server - Monitoring Stack${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

check_docker() {
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}❌ Docker가 설치되지 않았습니다.${NC}"
        echo "   설치: https://docs.docker.com/engine/install/ubuntu/"
        exit 1
    fi
    
    if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
        echo -e "${RED}❌ Docker Compose가 설치되지 않았습니다.${NC}"
        echo "   설치: sudo apt install docker-compose-plugin"
        exit 1
    fi
}

start_stack() {
    print_header
    echo -e "${YELLOW}🚀 모니터링 스택 시작 중...${NC}"
    echo ""
    
    check_docker
    
    # Docker Compose 실행 (v2 또는 v1)
    if docker compose version &> /dev/null; then
        docker compose up -d
    else
        docker-compose up -d
    fi
    
    echo ""
    echo -e "${GREEN}✅ 모니터링 스택 시작 완료!${NC}"
    echo ""
    echo -e "  📊 ${GREEN}Grafana${NC}:    http://localhost:3000  (admin/admin)"
    echo -e "  📈 ${GREEN}InfluxDB${NC}:   http://localhost:8086  (admin/adminpassword)"
    echo -e "  📡 ${GREEN}MQTT${NC}:       localhost:1883"
    echo -e "  🌐 ${GREEN}WebSocket${NC}: localhost:9001"
    echo ""
    echo -e "${YELLOW}💡 Grafana 대시보드가 자동으로 설정됩니다.${NC}"
    echo -e "${YELLOW}   SLAM MQTT 폴더에서 대시보드를 확인하세요.${NC}"
}

stop_stack() {
    print_header
    echo -e "${YELLOW}⏹️  모니터링 스택 중지 중...${NC}"
    
    if docker compose version &> /dev/null; then
        docker compose down
    else
        docker-compose down
    fi
    
    echo -e "${GREEN}✅ 중지 완료${NC}"
}

show_logs() {
    if docker compose version &> /dev/null; then
        docker compose logs -f
    else
        docker-compose logs -f
    fi
}

show_status() {
    print_header
    echo -e "${YELLOW}📊 컨테이너 상태:${NC}"
    echo ""
    
    if docker compose version &> /dev/null; then
        docker compose ps
    else
        docker-compose ps
    fi
}

# 메인
case "${1:-start}" in
    start)
        start_stack
        ;;
    stop)
        stop_stack
        ;;
    restart)
        stop_stack
        sleep 2
        start_stack
        ;;
    logs)
        show_logs
        ;;
    status)
        show_status
        ;;
    *)
        echo "사용법: $0 {start|stop|restart|logs|status}"
        exit 1
        ;;
esac
