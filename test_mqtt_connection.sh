#!/bin/bash
# MQTT 브로커 연결 테스트 스크립트

MQTT_HOST="${1:-localhost}"
MQTT_PORT="${2:-1883}"

echo "=========================================="
echo "MQTT 브로커 연결 테스트"
echo "=========================================="
echo "브로커: $MQTT_HOST:$MQTT_PORT"
echo ""

# mosquitto_pub로 테스트
echo "1. 연결 테스트 중..."
if timeout 3 mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "test/connection" -m "test" 2>&1; then
    echo "✓ MQTT 브로커 연결 성공"
else
    echo "✗ MQTT 브로커 연결 실패"
    echo ""
    echo "확인 사항:"
    echo "  1. MQTT 브로커가 실행 중인지 확인:"
    echo "     sudo systemctl status mosquitto"
    echo ""
    echo "  2. 포트가 열려있는지 확인:"
    echo "     sudo ufw status"
    echo "     sudo ufw allow $MQTT_PORT/tcp"
    echo ""
    echo "  3. 브로커가 리스닝 중인지 확인:"
    echo "     sudo netstat -tlnp | grep $MQTT_PORT"
    echo ""
    echo "  4. 로컬에서 테스트:"
    echo "     mosquitto_sub -h localhost -t test/topic"
    exit 1
fi

echo ""
echo "2. 구독 테스트..."
timeout 2 mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "test/topic" -C 1 &
SUB_PID=$!
sleep 1
mosquitto_pub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "test/topic" -m "Hello MQTT"
wait $SUB_PID

if [ $? -eq 0 ]; then
    echo "✓ 구독/발행 테스트 성공"
else
    echo "✗ 구독/발행 테스트 실패"
fi

echo ""
echo "테스트 완료"
