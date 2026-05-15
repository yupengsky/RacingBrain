import numpy as np
from scipy.optimize import minimize


def fit_circle(points, initial_guess):
    def objective_function(params, p):
        x0, y0, r = params
        distances = np.sqrt((p[:, 0] - x0) ** 2 + (p[:, 1] - y0) ** 2)
        return np.sum((distances - r) ** 2)

    result = minimize(objective_function, initial_guess, args=(points,))
    return result.x  # 返回圆心 (x0, y0) 和半径 r


def ransac_circle(points, initial_guess, cfg):
    best_inliers = []
    best_circle = None

    for _ in range(cfg.fit_num_iterations):
        # 随机选择 n 个点
        if len(points) < cfg.fit_n_pts:
            n = len(points)
        else:
            n = cfg.fit_n_pts
        sample_indices = np.random.choice(len(points), n, replace=False)
        sample_points = points[sample_indices]

        # 拟合圆
        x0, y0, r = fit_circle(sample_points, initial_guess)

        # 计算所有点到圆的距离
        distances = np.abs(np.sqrt((points[:, 0] - x0) ** 2 + (points[:, 1] - y0) ** 2) - r)

        # 标记内点
        inliers = points[distances < cfg.fit_error_threshold]

        # 如果当前内点数量更多，则更新最佳模型
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_circle = (x0, y0, r)

    return best_circle
