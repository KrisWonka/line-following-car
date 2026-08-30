"""跑一遍回归测试。

    cd tests && make && python3 bench.py

两部分：

  1. 重建精度  —— 在连续弯道上人为挖掉一段，补出来的中心线跟真值差多少。
                   有断口的那几行才计分；算法明确标成 -1（没有测量值）的行跳过，
                   那是"没输出"不是"输出错了"，单独统计覆盖率。

  2. 序列稳定性 —— 六个场景各跑 48 帧，统计中心线出现大幅横跳的帧数。
                   静态图看不出这类问题：之前有两个 bug（断口压在画面底部时
                   整幅退化成一条直线、画面顶部把默认值当中心线画出来）都是
                   逐帧扫过去才发现的。
"""
import os
import subprocess
import sys

import numpy as np

import synth

HERE = os.path.dirname(os.path.abspath(__file__))
RUNNER = os.path.join(HERE, 'runner')
TMP = os.path.join(HERE, '_tmp')


def run(img):
    """跑一帧，返回 (二值图, 最终中心线)。"""
    os.makedirs(TMP, exist_ok=True)
    src = os.path.join(TMP, 'in.pgm')
    synth.write_pgm(src, img)
    subprocess.run([RUNNER, src, os.path.join(TMP, 'bin.pgm'),
                    os.path.join(TMP, 'cl.txt')], check=True, cwd=TMP)
    cl = np.full(synth.H, -1, int)
    with open(os.path.join(TMP, 'cl.txt')) as f:
        for line in f:
            y, x = line.split()
            cl[int(y)] = int(x)
    return cl


# ---------------------------------------------------------------- 重建精度
GAPS = [('c_curve', 'curve', 25, 45), ('c_curve', 'curve', 40, 62),
        ('c_curve', 'curve', 55, 78), ('c_scurve', 'scurve', 25, 45),
        ('c_scurve', 'scurve', 40, 62), ('c_scurve', 'scurve', 55, 78)]


def bench_gap():
    print('重建精度（人为挖断，单位像素，图宽 %d）' % synth.W)
    print('  %-22s %8s %6s %10s' % ('用例', '平均', '最大', '有输出'))
    means = []
    for tag, kind, y0, y1 in GAPS:
        img, truth, _ = synth.frame(kind, 0.5)
        img[y0:y1, :] = synth.BG                 # 挖掉一段，真值保持不变
        cl = run(img)
        errs = [abs(cl[y] - truth[y]) for y in range(y0, y1)
                if truth[y] >= 0 and cl[y] > 0]
        cover = len([y for y in range(y0, y1) if truth[y] >= 0])
        if not errs:
            print('  %-22s %8s' % ('%s %d-%d' % (tag, y0, y1), '无输出'))
            continue
        m = sum(errs) / float(len(errs))
        means.append(m)
        print('  %-22s %8.1f %6d %7d/%d'
              % ('%s %d-%d' % (tag, y0, y1), m, max(errs), len(errs), cover))
    if means:
        print('  %-22s %8.1f' % ('平均', sum(means) / len(means)))
    return sum(means) / len(means) if means else None


# ------------------------------------------------------------ 序列稳定性
CASES = [('直道', 'straight', None, 0), ('弯道', 'curve', None, 0),
         ('十字路口', 'curve', 'cross', 12), ('短断口', 'curve', 'gap', 10),
         ('长断口', 'curve', 'gap', 22), ('S 弯', 'scurve', None, 0)]

JUMP_PX = 20        # 相邻两行中心线横跳超过这个值就算异常
NFRAMES = 48


def bench_seq():
    print('\n序列稳定性（每个场景 %d 帧，中心线相邻行横跳 > %d px 的帧数）'
          % (NFRAMES, JUMP_PX))
    bad_total = 0
    for name, kind, feat, rows in CASES:
        bad = []
        for i in range(NFRAMES):
            img, _, _ = synth.frame(kind, i / float(NFRAMES - 1), feat, rows)
            cl = run(img)
            worst = 0
            for y in range(2, synth.H - 1):
                if cl[y] <= 0 or cl[y - 1] <= 0:
                    continue                     # -1 是"没有测量值"，不是跳变
                worst = max(worst, abs(cl[y] - cl[y - 1]))
            if worst > JUMP_PX:
                bad.append(i)
        bad_total += len(bad)
        print('  %-10s %2d/%d %s' % (name, len(bad), NFRAMES,
                                     bad if bad else ''))
    return bad_total


if __name__ == '__main__':
    if not os.path.exists(RUNNER):
        sys.exit('先编译： cd tests && make')
    mean = bench_gap()
    bad = bench_seq()
    print('\n参考基线（2026-08 修完补线路径后，本套合成用例）：'
          '重建精度 2.1 px，序列异常帧 4')
    print('异常帧集中在十字路口开头几帧——横杠占满整幅画面宽度时，'
          '那几行本就没有"车道中心"可言，属已知情况。')
    if mean is not None:
        print('本次：重建精度 %.1f px，序列异常帧 %d' % (mean, bad))
