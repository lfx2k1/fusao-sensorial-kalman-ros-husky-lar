#!/usr/bin/env bash
# Uso: ./run_experiment.sh odom|odom_imu|odom_imu_gps
# Roda: husky+sensores -> EKF da config escolhida -> trajetoria automatica
# -> grava rosbag de /odometry/filtered e /gt/odom -> roda compute_metrics.py
set -e

CONFIG="$1"
if [[ -z "$CONFIG" ]]; then
  echo "Uso: $0 odom|odom_imu|odom_imu_gps"
  exit 1
fi

case "$CONFIG" in
  odom)          EKF_LAUNCH="ekf_odom.launch" ;;
  odom_imu)      EKF_LAUNCH="ekf_odom_imu.launch" ;;
  odom_imu_gps)  EKF_LAUNCH="ekf_odom_imu_gps.launch" ;;
  *) echo "Config invalida: $CONFIG"; exit 1 ;;
esac

PKG_DIR="$(rospack find kalman_localization)"
RESULTS_DIR="${PKG_DIR}/results"
mkdir -p "${RESULTS_DIR}"
BAG_PATH="${RESULTS_DIR}/${CONFIG}.bag"

echo ">>> Suba antes, em outro terminal: roslaunch kalman_localization husky_kalman.launch"
echo ">>> Iniciando EKF (${EKF_LAUNCH}) + gravacao do bag..."

roslaunch kalman_localization "${EKF_LAUNCH}" &
EKF_PID=$!
sleep 3

rosbag record -O "${BAG_PATH}" /odometry/filtered /gt/odom __name:=metrics_recorder &
BAG_PID=$!
sleep 2

rosrun kalman_localization drive_pattern.py

sleep 2
rosnode kill /metrics_recorder || true
sleep 2
kill "${EKF_PID}" 2>/dev/null || true

echo ">>> Bag salvo em ${BAG_PATH}"
echo ">>> Calculando metricas..."
rosrun kalman_localization compute_metrics.py "${BAG_PATH}" "${CONFIG}" "${RESULTS_DIR}"
