import ast
import io

p = '_p44_follower.py'
s = io.open(p, encoding='utf-8').read()
done = []


def rep(old, new, tag):
    global s
    if new in s:
        done.append(tag + '(已存在)')
        return
    assert s.count(old) == 1, '%s 匹配 %d 次' % (tag, s.count(old))
    s = s.replace(old, new, 1)
    done.append(tag)


# ---- P44-1: 斜前方只在"真的近"时才参与 d_head ----
rep(
    """        d_head = min(d_f, min(d_fl, d_fr) * 0.72)""",
    """        # P44: 斜前方不再无条件乘 0.72。原来 45° 方向 0.90m 的障碍会被
        # 折算成 0.65m(< stop_d 0.66) —— 机器人明明可以从旁边过去, 却直接
        # 停车。实测 150s 里 far_pct 73%、中位距离 2.41m, 大部分是被这条
        # "看什么都像正前方"的规则卡住的。
        _d_diag = min(d_fl, d_fr)
        d_head = min(d_f, _d_diag * 0.72) if _d_diag < 0.85 else d_f""",
    '1-斜前方折算',
)

# ---- P44-2: 限速不许退化成爬行 ----
rep(
    """            elif d_head < slow_d:
                v_cap = (d_head - stop_d) / tau
                if v_t > v_cap:
                    v_t = max(0.0, v_cap)""",
    """            elif d_head < slow_d:
                v_cap = (d_head - stop_d) / tau
                # P44: 原式在 d_head=0.70 时只给 (0.70-0.66)/0.6 = 0.067m/s,
                # 机器人贴着障碍以 7cm/s 磨, 人一走动就彻底掉队。只要还有
                # 余量就至少给 0.22m/s(10Hz 深度 + 0.04m 制动距离, 安全);
                # 余量不足 0.04m 就交给下面的急停分支去"停-转"。
                if d_head > stop_d + 0.04:
                    v_cap = max(v_cap, 0.22)
                if v_t > v_cap:
                    v_t = max(0.0, v_cap)""",
    '2-限速下限',
)

io.open(p, 'w', encoding='utf-8', newline='\n').write(s)
ast.parse(s)
print('P44 OK, 语法通过, %d 字符' % len(s))
for d in done:
    print('  - ' + d)
