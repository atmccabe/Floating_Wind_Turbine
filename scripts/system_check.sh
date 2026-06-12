#!/bin/bash

echo "===== Wind Turbine System Check ====="
echo ""

echo "1. Git version:"
git --version || echo "Git not working"
echo ""

echo "2. Python version:"
python3 --version || echo "Python not working"
echo ""

echo "3. Docker version:"
docker --version || echo "Docker not working"
echo ""

echo "4. Docker containers:"
docker ps
echo ""

echo "5. InfluxDB health:"
curl -s http://localhost:8086/health || echo "InfluxDB not responding"
echo ""
echo ""

echo "6. Grafana status:"
systemctl is-active grafana-server
echo ""

echo "7. Grafana HTTP test:"
curl -I http://localhost:3000 | head -5 || echo "Grafana not responding"
echo ""

echo "8. Node-RED HTTP test:"
curl -I http://localhost:1880 | head -5 || echo "Node-RED not responding. It may not be started."
echo ""

echo "===== Check complete ====="
