# -*- coding: utf-8 -*-
"""回退 P82-b 的"连续绕视"改动(实测比原版慢)。"""
import sys

F = sys.argv[1]
raw = open(F, 'rb').read().decode('utf-8')


def CR(s):
    return s.replace('\n', '\r\n')


def sub(old, new, tag, cnt=1):
    global raw
    o, n = CR(old), CR(new)
    k = raw.count(o)
    if k != cnt:
        print('FAIL[%s]: 命中 %d 次, 期望 %d' % (tag, k, cnt))
        sys.exit(1)
    raw = raw.replace(o, n, cnt)
    print('ok  %s' % tag)


sub("""    'peek_rounds': 4,               # P82: 一次"找不到人"最多连绕几段(每段 peek_move_s)
""", "", 'A 去掉 peek_rounds 配置')

sub("""        self._peek_round = 0          # P82: 本轮"连续绕视"的第几段
""", "", 'B 去掉 _peek_round 字段')

sub("""            if _over or _free:
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
                    # 连绕期间**不**换边: 换边判据是 (_peek_n//2)%2, 每加一段
                    # 就 +1 会让相邻两段反向、位移互相抵消(实测 h83: 失检
                    # 28.0s, 比不连绕的 16.3s 还慢)。保持同向才绕得过去。
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
    """            if _over or _free:
                # P82 注: 试过"超时继续连绕"(peek_rounds), 两种换边策略都更慢
                #   (同向连绕 h84 失检 23.9s 且末态未检出; 逐段换边 h83 28.0s),
                #   都比"绕一段→完整扫描→再绕"的 16.3s 差 —— 扫描那 5s 虽然
                #   不动, 但它把 360° 重新扫了一遍, 覆盖效率高于继续平移。
                self.get_logger().info(
                    'FOLLOWER 绕视结束(%.1fs, %s) → 交回常规处理'
                    % (now - self._peek_t0,
                       '视线已绕通' if _free else '超时'))
                if self._peek_from == 'SCAN':
                    self._enter_scan(now, announce=False)
                else:
                    self._track = 'TRACK'   # 下一拍重新判定 → GOTO/SCAN
""", 'C 恢复原退出逻辑')

sub("""                    self._peek_from = self._track
                    self._peek_round = 0      # P82: 新一轮连续绕视
                    self._peek_n += 1
""",
    """                    self._peek_from = self._track
                    self._peek_n += 1
""", 'D 去掉清零行')

open(F, 'w', encoding='utf-8', newline='').write(raw)
print('reverted', len(raw))
