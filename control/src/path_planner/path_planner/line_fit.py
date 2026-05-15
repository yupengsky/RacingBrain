import numpy as np


def fit_line(points):
    x = points[:, 0]
    y = points[:, 1]
    A = np.vstack([x, np.ones(len(x))]).T
    k, b = np.linalg.lstsq(A, y, rcond=None)[0]  # 最小二乘法拟合直线
    return k, b


def distance_point_to_line(k, b, point):
    x, y = point
    return abs(k * x - y + b) / np.sqrt(k**2 + 1)


def ransac_line(points, cfg):
    best_inliers = []
    best_k, best_b = 0, 0

    for _ in range(cfg.fit_num_iterations):
        # 随机选择 n 个点
        if len(points) < cfg.fit_n_pts:
            n = len(points)
        else:
            n = cfg.fit_n_pts
        sample_indices = np.random.choice(len(points), n, replace=False)
        sample_points = points[sample_indices]

        fitted_k, fitted_b = fit_line(sample_points)

        # 计算所有点到直线的距离
        distances = [distance_point_to_line(fitted_k, fitted_b, p) for p in points]

        # 判断内点
        inliers = points[np.array(distances) < cfg.fit_error_threshold]

        # 如果当前内点数量更多，则更新最佳模型
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_k, best_b = fitted_k, fitted_b

    # 计算方向向量
    direction_vector = np.array([1, best_k]) / np.linalg.norm([1, best_k])

    return direction_vector
