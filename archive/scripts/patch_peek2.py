# -*- coding: utf-8 -*-
"""P82-b: 绕视连续化 —— 超时未见人时继续绕, 不回扫描空等。"""
import sys

F = sys.argv[1]
with open(F, 'rb') as fh:
    data = fh.read()


def C(s):
    return s.replace('\n', '\r\n').encode('utf-8')


def sub(old, new, tag, cnt=1):
    global data
    o, n = C(old), C(new)
    k = data.count(o)
    if k != cnt:
        print('FAIL[%s]: 命中 %d 次, 期望 %d' % (tag, k, cnt))
        sys.exit(1)
    data = data.replace(o, n, cnt)
    print('ok  %s' % tag)


# A. 配置
sub("""    'scan_seek_s': 3.0,             # P81: 原地扫描超过这么久仍无检出 → 横移绕视
""",
    """    'scan_seek_s': 3.0,             # P81: 原地扫描超过这么久仍无检出 → 横移绕视
    'peek_rounds': 4,               # P82: 一次"找不到人"最多连绕几段(每段 peek_move_s)
""",
    'A 配置 peek_rounds')

# B. 状态字段
sub("""        self._peek_n = 0              # 累计绕视次数(diag)
""",
    """        self._peek_n = 0              # 累计绕视次数(diag)
        self._peek_round = 0          # P82: 本轮"连续绕视"的第几段
""",
    'B 字段 _peek_round')

# C. 退出块改连续绕视
sub("""            if _over or _free:
                self.get_logger().info(
                    'FOLLOWER 绕视结束(%.1fs, %s) → 交回常规处理'
                    % (now - self._peek_t0,
                       '视线已绕通' if _free else '超时'))
                if self._peek_from == 'SCAN':
                    self._enter_scan(now, announce=False)
                else:
                    self._track = 'TRACK'   # 下一拍重新判定 → GOTO/SCAN
""",
    """            if _over or _free:
                # P82: 绕视**超时但人还没找回来**时继续绕, 不回扫描。
                # 回扫描要先空等 scan_dur_s(约 5.2s)才再给 scan_seek_s(3s)
                # 的横移窗口 —— 一个来回 8s 起步。实测"人躲柱后": SCAN 5.2s
                # → PEEK 5s → SCAN 5.2s → PEEK …, 要 16.3s 才重捕获。
                # 连续绕视把这段等待全换成位移, 绕柱一周人就露出来了。
                _chain = (not _free) and self._peek_from == 'SCAN' \\
                    and self._peek_round < int(self.cfg.get('peek_rounds', 4)) \\
                    and lost_dt > 0.5
                if _chain:
                    self._peek_round += 1
                    self._peek_n += 1          # 换边判据跟着段数走
                    self._peek_t0 = now
                    self._peek_free_t = 0.0
                    self._peek_obs = _los_blocked(rx, ry, px, py)
                    self.get_logger().info(
                        'FOLLOWER 绕视第 %d 段(上段未见人) → 继续绕行(视线%s)'
                        % (self._peek_round,
                           '被%s挡住' % self._peek_obs[0]
                           if self._peek_obs else '通畅'))
                else:
                    self._peek_round = 0
                    self.get_logger().info(
                        'FOLLOWER 绕视结束(%.1fs, %s) → 交回常规处理'
                        % (now - self._peek_t0,
                           '视线已绕通' if _free else '超时'))
                    if self._peek_from == 'SCAN':
                        self._enter_scan(now, announce=False)
                    else:
                        self._track = 'TRACK'   # 下一拍重新判定 → GOTO/SCAN
""",
    'C 连续绕视')

# D. 进入新一段绕视时清 _peek_round(从丢失判定进入的场景)
sub("""                    self._peek_from = self._track
                    self._peek_n += 1
""",
    """                    self._peek_from = self._track
                    self._peek_round = 0      # P82: 新一轮连续绕视
                    self._peek_n += 1
""",
    'D 丢失判定进入时清零')

with open(F, 'wb') as fh:
    fh.write(data)
print('written', len(data))
