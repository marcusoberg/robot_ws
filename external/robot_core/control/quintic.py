def smooth_progress(fraction):
    return fraction**3 * (10 - 15 * fraction + 6 * fraction**2)


def smooth_rate(fraction):
    return 30 * fraction**2 * (1 - fraction)**2
