#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 P80 记录追加到当日 memory。"""

SEC = """

## P80 14:10–15:10（撞人归因：盲走）

- **先撤回 P79 的一句错话**：§20.8 说「min_all 0.41 一半来自开局预热」。把最小值
  的时刻逐一定位后，三次是 t=88.8 / 29.3 / 90.4 s，全都紧邻一次检测丢失。
  人 skirt 圆柱 r=0.26 + 车体 r=0.18 ⇒ **中心距 0.44 m 即物理接触**，0.41 是真撞上。
  教训：**报极值时必须同时报它发生的时刻和上下文**，否则很容易把它当成统计噪声。
- **评测缺的是"决策级"字段**。原来只记距离/角度/指令，遇到「0.65 m 还给 +0.23」
  只能猜。给 `tmp_random_eval.py` 加 `/follower/diag` 订阅（note/mode/td/sl/dh/dl/dr/dw），
  `_dbg` 加 `vt/pv/vr`，一次就能读出失效链条：
  视觉连续 4 帧 `det=0`，ALIGN 的「边转边走」仍按**陈旧** last-known 距离判「没到
  目标距离」，以 0.22 m/s 前推 1.0 s，把 0.62 m 顶到 0.41 m。**这是"撞人"的唯一真因。**
- **检测器在真值 0.70 m 就看不见人**（`vis` 原始 0.39 → detected=false，人体占满画幅），
  而深度相机 0.4 m 内报 `dh=99` 无回波。也就是说近距没有任何传感器兜底。
  P79 把后退闸门设在 0.70 m，恰好卡在「已经瞎了」的边界上 —— 「停在 0.77 m 等人
  自己走完」是自相矛盾的策略。
- **三处修**：① `blind_v_s=0.30` 盲走禁令（`_publish_smooth` 里只压正向，视觉连续
  3 帧无检出就不许前进）；② `back_hold_*` 主动让距（<0.85 m，速度 = gain×人的逼近
  速度 + k×(0.85-dist)，上限 0.45，需后方净空）；③ ALIGN 的「边转边走」<0.85 m 时
  改走同一套让距。②③是"跟人的逼近速度走"，人停就停 ⇒ 不会像 P79-C 那样距离一抖就倒车。
- **结果（tmp_eval_rep.sh 3 p80c 120）**：接触帧 101/44/49 → **0/0/0**；
  min_d 0.41 → **0.64**；in_band 98.6 → **100.0**；near 1.4 → 0；居中≤10° 96.4 → 97.5；
  max_deg 32~39 → **20.6**；丢人 0~3 → **0**。back 1.4% → 10.4%（让距换来的，
  12~14 次/100s、平均 0.78s，稳态仍是 0）。
- **判据**：接触 101→0 是「有/无」的区别（3 次全 0），远超噪声底（in_band ±6.3pt、
  min_d ±0.09）；in_band 100% 在这个宽带口径下不稀奇，真正有信息量的是 min_d 和接触。
- 参数：`blind_v_s=0.3`、`back_hold_m=0.85`、`back_hold_gain=1.0`、`back_hold_k=0.7`、
  `back_hold_vmax=0.45`、`back_hold_vmin=0.06`、`back_hold_on=1`。文档 `TaskPlan.md` §21。
- 工具坑（复发）：**`TaskPlan.md` 是 CRLF**，用 Python 替换/追加时 `open(..., newline='')`
  读、按 `\\r\\n` 写，否则 `old in s` 直接 AssertionError（`\\n` 匹配不到 `\\r\\n`）。
  另外 heredoc 里写含三引号的长文档容易断（`unterminated string literal`），
  改用 Write 工具落地一个 `tmp_*_doc.py` 再执行。
"""


def main():
    p = '.workbuddy/memory/2026-10-26.md'
    s = open(p, encoding='utf-8', newline='').read()
    nl = '\r\n' if '\r\n' in s else '\n'
    body = SEC.replace('\r\n', '\n').replace('\n', nl)
    open(p, 'a', encoding='utf-8', newline='').write(body)
    print('appended %d chars, nl=%r' % (len(body), nl))


if __name__ == '__main__':
    main()
