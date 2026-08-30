"""合成相机视角的赛道图，供回归测试用。

真实素材只有几张连续弯道，覆盖不到断口、十字、S 弯这些情况，
更要命的是没有"标准答案"——补出来的中心线到底偏多少，没法量。
这里直接按行画车道，中心线的真值就是画的时候那条，误差可以精确算。
"""
import numpy as np

W, H = 188, 120
BG, LANE = 208, 38          # 背景灰 / 车道灰


def centre(y, kind, t):
    """车道中心线在第 y 行的横坐标。y = H-1 是车头最近处。"""
    u = (H - 1 - y) / float(H - 1)          # 0 = 近，1 = 远
    if kind == 'straight':
        return W / 2 + 14 * np.sin(2 * np.pi * t) * u
    if kind == 'curve':
        return (W / 2 + 10 * np.sin(2 * np.pi * t) * u
                + 62 * u * u * np.sin(np.pi * (0.25 + 0.5 * t)))
    if kind == 'scurve':
        return (W / 2 + 8 * np.sin(2 * np.pi * t) * u
                + 46 * np.sin(3.1 * u + 2 * np.pi * t) * u)
    return W / 2


def halfwidth(y):
    """近宽远窄，粗略模拟透视。"""
    u = (H - 1 - y) / float(H - 1)
    return 15.0 * (1 - 0.72 * u) + 1.5


def frame(kind, t, feature=None, feat_rows=0, noise=True):
    """画一帧。feature 从画面上方随 t 压向车头，模拟车开过去。

    feature = 'cross' 画一条车道色的横杠（十字路口，车道与横杠连成一片）
    feature = 'gap'   把那一段抹成背景色（磨损，车道断开）
    """
    img = np.full((H, W), BG, np.uint8)
    truth = np.full(H, -1, int)

    for y in range(H):
        c = centre(y, kind, t)
        hw = halfwidth(y)
        x0, x1 = int(round(c - hw)), int(round(c + hw))
        if x1 >= 0 and x0 < W:
            img[y, max(0, x0):min(W, x1 + 1)] = LANE
            truth[y] = int(round(c))

    fy = None
    if feature:
        fy = int(-feat_rows + t * (H + 2 * feat_rows))
        a, b = max(0, fy), min(H, fy + feat_rows)
        if b > a:
            if feature == 'cross':
                img[a:b, :] = LANE          # 横杠也是车道色
            elif feature == 'gap':
                img[a:b, :] = BG            # 车道消失
                # 真值仍是原来那条线——补线就是要把它还原出来

    if noise:
        rng = np.random.default_rng(int(t * 1e6))
        img = np.clip(img.astype(np.int16) + rng.integers(-4, 5, (H, W)), 0, 255).astype(np.uint8)

    return img, truth, fy


def write_pgm(path, img):
    """image.c 的 read_pgm 固定跳过 15 字节文件头，这里的头正好 15 字节。"""
    header = b"P5\n%d %d\n255\n" % (W, H)
    assert len(header) == 15, len(header)
    with open(path, 'wb') as f:
        f.write(header + img.tobytes())
