"""Step-response numbers for the tuning helper (pure Python, no Qt)."""


def analyze(t, sp, meas, t_step, t_end=None):
    """t, sp, meas: equally long lists. t_step: time of the set point step. t_end: end of the evaluated window.

    Returns a dict {step, rise_s, overshoot_pct, settle_s, final_err} or None if there is not enough data.
    rise_s = 10 % to 90 % time, settle_s = last time the error left a band of 5 % of the step (measured from the step),
    final_err = set point minus measurement at the end of the window (average of the last 10 % of the samples).
    """
    n = len(t)
    if n < 8 or len(sp) != n or len(meas) != n:
        return None
    if t_end is None:
        t_end = t[-1]
    idx = [i for i in range(n) if t_step <= t[i] <= t_end]
    pre = [i for i in range(n) if t[i] < t_step]
    if len(idx) < 6 or not pre:
        return None
    y0 = sum(meas[i] for i in pre[-3:]) / len(pre[-3:])
    target = sp[idx[-1]]
    step = target - y0
    if abs(step) < 1e-9:
        return None
    k = max(1, len(idx) // 10)
    y_end = sum(meas[i] for i in idx[-k:]) / k
    norm = [(meas[i] - y0) / step for i in idx]
    ts = [t[i] - t_step for i in idx]
    rise = None
    t10 = t90 = None
    for j, y in enumerate(norm):
        if t10 is None and y >= 0.1:
            t10 = ts[j]
        if t90 is None and y >= 0.9:
            t90 = ts[j]
            break
    if t10 is not None and t90 is not None:
        rise = t90 - t10
    peak = max(norm)
    overshoot = max(0.0, (peak - 1.0) * 100.0)
    settle = 0.0
    for j in range(len(norm) - 1, -1, -1):
        if abs(norm[j] - 1.0) > 0.05:
            settle = ts[min(j + 1, len(ts) - 1)]
            break
    if abs(norm[-1] - 1.0) > 0.05 and abs((y_end - y0) / step - 1.0) > 0.05:
        settle = None
    return {"step": step, "rise_s": rise, "overshoot_pct": overshoot, "settle_s": settle, "final_err": target - y_end}
