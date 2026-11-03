"""Align recorded samples and draw figures without ROS dependencies."""

import numpy as np


def aligned_results(actual, target, circle_start=None):
    """Keep raw positions, but evaluate errors only after hover ends.

    Live recording supplies the first circle-state reception time. For older
    references without phase metadata, use the first nonzero horizontal speed.
    """
    actual, target = np.asarray(actual), np.asarray(target)
    t = actual[:, 0]
    reference = np.column_stack([
        np.interp(t, target[:, 0], target[:, i]) for i in range(1, 7)])
    valid = (t >= target[0, 0]) & (t <= target[-1, 0])
    if circle_start is None:
        starts = target[np.hypot(target[:, 4], target[:, 5]) > 1e-9, 0]
        circle_start = starts[0] if len(starts) else float('inf')
    moving = valid & (t >= circle_start)
    error = np.full(len(t), np.nan)
    error[moving] = np.hypot(actual[moving, 1] - reference[moving, 0],
                             actual[moving, 2] - reference[moving, 1])
    return np.column_stack((actual, reference[:, :3], error, moving.astype(int)))


def draw_results(axes, data, target):
    for ax in axes:
        ax.clear()
        ax.grid()
    data = data[data[:, 8] > 0]
    if not len(data):
        for ax in axes:
            ax.set_title('等待悬停结束，开始圆周轨迹')
        return
    start = data[0, 0]
    target = target[(target[:, 0] >= start) & (target[:, 0] <= data[-1, 0])]
    axes[0].plot(data[:, 1], data[:, 2], label='实际轨迹')
    axes[0].plot(target[:, 1], target[:, 2], '--', label='目标轨迹')
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].set(xlabel='北向位置 x (m)', ylabel='东向位置 y (m)', title='水平位置轨迹（NED）')
    axes[0].legend()
    axes[1].plot(data[:, 0] - start, -data[:, 3], label='实际轨迹')
    axes[1].plot(target[:, 0] - start, -target[:, 3], '--', label='目标轨迹')
    axes[1].set(xlabel='悬停结束后时间 (s)', ylabel='高度 (m)', title='相对高度（向上为正）')
    axes[1].legend()
    moving = data[:, 8] > 0
    rms = np.sqrt(np.mean(data[moving, 7] ** 2)) if np.any(moving) else float('nan')
    axes[2].plot(data[:, 0] - start, data[:, 7], color='tab:red')
    axes[2].set(xlabel='悬停结束后时间 (s)', ylabel='误差 (m)',
                title=f'水平位置跟踪误差\n圆周阶段 RMS = {rms:.3f} m')
