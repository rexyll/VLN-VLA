"""修正 §14.4 第 2 条的表述（pgrep -x gz 是假门禁），TaskPlan + memory 同步"""
import paramiko

HOME = '/home/earl/workspace/VLA'

OLD_TP = ('| 2 | 清场门禁永远不过，残留数恒为 `4/2/9/1` | **`pgrep -f "gz sim"` 会匹配到自己的命令行**，'
          '最小值就是 1~2。改用 `pgrep -x gz` / `pgrep -x parameter_bridge` 后 1s 内即清场完成 |')
NEW_TP = ('| 2 | 清场门禁永远是「通过」或永远是「不过」，两个方向都错 | ①`pgrep -f "gz sim"` '
          '**会匹配到自己的命令行**，计数最小值 1~2，永远判不过；②更要命的是 '
          '**`gz sim` 的 comm 是 `ruby`**（gz sim 是个 ruby 包装脚本），'
          '`pgrep -x gz` 恒为 0 —— 换成它又变成「永远通过」的假门禁，'
          '实测这一版仍在旧 gz 存活时开了录制（`[RESET] 465.0 → 8.6` 就是铁证）。'
          '正确写法：`pgrep -f "gz si[m]"`（方括号避免自匹配）或 `pgrep -x ruby` |')

OLD_MEM = ('2. 清场门禁永远不过、残留数恒定 `4/2/9/1`：**`pgrep -f "gz sim"` 匹配到自己命令行**。'
           '改用 `pgrep -x gz` / `pgrep -x parameter_bridge`，1s 内清场完成。')
NEW_MEM = ('2. 清场门禁两边都是假的：`pgrep -f "gz sim"` **匹配到自己命令行**（计数最小值 1~2，永远判不过）；'
           '而 **`gz sim` 的 comm 是 `ruby`**（gz 是 ruby 包装脚本），换成 `pgrep -x gz` 就恒为 0，'
           '变成「永远通过」的假门禁 —— 实测这一版仍在旧 gz 存活时开录。'
           '正确写法：`pgrep -f "gz si[m]"` 或 `pgrep -x ruby`。')

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()

for path, old, new, tag in (
        (HOME + '/TaskPlan.md', OLD_TP, NEW_TP, 'TaskPlan'),
        (HOME + '/.workbuddy/memory/2026-10-25.md', OLD_MEM, NEW_MEM, 'memory')):
    with sftp.file(path, 'r') as f:
        s = f.read().decode('utf-8')
    n = s.count(old)
    if n != 1:
        print('%s: anchor count=%d, SKIP' % (tag, n))
        continue
    s = s.replace(old, new, 1)
    with sftp.file(path, 'w') as f:
        f.write(s.encode('utf-8'))
    print('%s: fixed' % tag)

sftp.close()
cli.close()
