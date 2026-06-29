#!/usr/bin/env python3
"""
Uso:
    rosrun kalman_localization compute_metrics.py <bag> <label> [outdir]

Lê um rosbag gravado durante um teste (odom_only / odom_imu / odom_imu_gps),
extrai /odometry/filtered e /gt/odom, alinha por timestamp mais próximo e
calcula: erro de posição ao longo do tempo, RMSE, erro final e erro de
orientação (yaw). Salva CSV + gráficos (matplotlib) em <outdir>/<label>/.

NOTA (correção de alinhamento de frame):
/odometry/filtered é publicado no frame `odom`, que por convenção comeca em
(0, 0, 0) no instante em que o EKF sobe - independente de onde o robo esteja
de fato no mundo. /gt/odom e publicado no frame do mundo (pose real do
Gazebo). Antes de calcular qualquer erro, alinhamos a trajetoria filtrada ao
ground truth usando a primeira amostra de cada serie como referencia: achamos
a rotacao (diferenca de yaw) e a translacao que levam o ponto inicial do
filtrado para o ponto inicial do ground truth, e aplicamos essa transformacao
em toda a trajetoria filtrada antes de comparar. Sem isso, o erro de posicao
fica inflado por uma diferenca de referencia que nao tem nada a ver com a
qualidade da estimativa do EKF.
"""
import sys
import os
import math
import csv

import rosbag
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def yaw_from_quat(q):
    # yaw a partir de quaternion (sem depender de tf)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def read_odom_topic(bag, topic):
    t_list, x_list, y_list, yaw_list = [], [], [], []
    for _, msg, t in bag.read_messages(topics=[topic]):
        t_list.append(t.to_sec())
        x_list.append(msg.pose.pose.position.x)
        y_list.append(msg.pose.pose.position.y)
        yaw_list.append(yaw_from_quat(msg.pose.pose.orientation))
    return np.array(t_list), np.array(x_list), np.array(y_list), np.array(yaw_list)


def align_filtered_to_gt(x_f, y_f, yaw_f, x_g0, y_g0, yaw_g0):
    """
    Calcula a transformacao (rotacao + translacao) que leva a primeira pose
    do filtrado (x_f[0], y_f[0], yaw_f[0]) para a primeira pose do ground
    truth (x_g0, y_g0, yaw_g0), e aplica essa transformacao em toda a serie
    filtrada. Retorna as series alinhadas e o offset aplicado (para log).
    """
    x0_f, y0_f, yaw0_f = x_f[0], y_f[0], yaw_f[0]

    dyaw = angle_diff(yaw_g0, yaw0_f)
    c, s = math.cos(dyaw), math.sin(dyaw)

    dx = x_f - x0_f
    dy = y_f - y0_f

    x_aligned = x_g0 + (c * dx - s * dy)
    y_aligned = y_g0 + (s * dx + c * dy)
    yaw_aligned = yaw_f + dyaw

    offset_info = {
        'dx': x_g0 - x0_f,
        'dy': y_g0 - y0_f,
        'dyaw': dyaw,
    }
    return x_aligned, y_aligned, yaw_aligned, offset_info


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    bag_path = sys.argv[1]
    label = sys.argv[2]
    outdir = sys.argv[3] if len(sys.argv) > 3 else 'results'
    outdir = os.path.join(outdir, label)
    os.makedirs(outdir, exist_ok=True)

    bag = rosbag.Bag(bag_path)

    t_f, x_f, y_f, yaw_f = read_odom_topic(bag, '/odometry/filtered')
    t_g, x_g, y_g, yaw_g = read_odom_topic(bag, '/gt/odom')
    bag.close()

    if len(t_f) == 0 or len(t_g) == 0:
        print("ERRO: nao encontrei dados em /odometry/filtered e/ou /gt/odom no bag.")
        sys.exit(1)

    # Alinha cada amostra filtrada ao ground truth mais proximo no tempo
    idx = np.searchsorted(t_g, t_f)
    idx = np.clip(idx, 1, len(t_g) - 1)
    left = idx - 1
    right = idx
    use_left = np.abs(t_f - t_g[left]) <= np.abs(t_g[right] - t_f)
    nearest = np.where(use_left, left, right)

    gx = x_g[nearest]
    gy = y_g[nearest]
    gyaw = yaw_g[nearest]

    # ---- Correcao de frame: alinha o filtrado (frame odom) ao ground
    # truth (frame do mundo) usando a primeira pose de cada serie ----
    x_f, y_f, yaw_f, offset_info = align_filtered_to_gt(
        x_f, y_f, yaw_f, gx[0], gy[0], gyaw[0]
    )

    err_xy = np.sqrt((x_f - gx) ** 2 + (y_f - gy) ** 2)
    err_yaw = np.array([abs(angle_diff(a, b)) for a, b in zip(yaw_f, gyaw)])

    rmse_xy = math.sqrt(np.mean(err_xy ** 2))
    rmse_yaw = math.sqrt(np.mean(err_yaw ** 2))
    final_err = err_xy[-1]
    final_err_yaw = err_yaw[-1]
    t0 = t_f[0]

    # ---- CSV ----
    csv_path = os.path.join(outdir, 'metrics.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x_filtered', 'y_filtered', 'yaw_filtered',
                    'x_gt', 'y_gt', 'yaw_gt', 'pos_error', 'yaw_error'])
        for i in range(len(t_f)):
            w.writerow([t_f[i] - t0, x_f[i], y_f[i], yaw_f[i],
                        gx[i], gy[i], gyaw[i], err_xy[i], err_yaw[i]])

    summary_path = os.path.join(outdir, 'summary.txt')
    with open(summary_path, 'w') as f:
        f.write(f"Configuracao: {label}\n")
        f.write(f"Amostras: {len(t_f)}\n")
        f.write(f"RMSE posicao (m): {rmse_xy:.4f}\n")
        f.write(f"Erro final posicao (m): {final_err:.4f}\n")
        f.write(f"RMSE orientacao (rad): {rmse_yaw:.4f}\n")
        f.write(f"Erro final orientacao (rad): {final_err_yaw:.4f}\n")
        f.write("\n[NOTA] Metricas calculadas apos alinhar o frame 'odom' "
                "(filtrado) ao frame do ground truth pela pose inicial.\n")
        f.write(f"Offset de translacao aplicado: dx={offset_info['dx']:.3f} m, "
                f"dy={offset_info['dy']:.3f} m; offset de rotacao (dyaw): "
                f"{offset_info['dyaw']:.4f} rad "
                f"({math.degrees(offset_info['dyaw']):.2f} graus)\n")
    print(open(summary_path).read())

    # ---- Graficos ----
    plt.figure(figsize=(6, 6))
    plt.plot(x_g, y_g, label='Ground truth', linewidth=2)
    plt.plot(x_f, y_f, '--', label=f'Filtrado ({label}, alinhado)')
    plt.axis('equal')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title(f'Trajetoria - {label}')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(outdir, 'trajetoria.png'), dpi=150)
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(t_f - t0, err_xy)
    plt.xlabel('tempo (s)')
    plt.ylabel('erro de posicao (m)')
    plt.title(f'Erro de posicao ao longo do tempo - {label}')
    plt.grid(True)
    plt.savefig(os.path.join(outdir, 'erro_posicao.png'), dpi=150)
    plt.close()

    plt.figure(figsize=(8, 4))
    plt.plot(t_f - t0, err_yaw)
    plt.xlabel('tempo (s)')
    plt.ylabel('erro de orientacao (rad)')
    plt.title(f'Erro de orientacao ao longo do tempo - {label}')
    plt.grid(True)
    plt.savefig(os.path.join(outdir, 'erro_orientacao.png'), dpi=150)
    plt.close()

    print(f"Resultados salvos em: {outdir}")


if __name__ == '__main__':
    main()
