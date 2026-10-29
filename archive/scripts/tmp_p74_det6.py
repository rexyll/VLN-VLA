"""P74: 只重跑 B —— 加固清场等待(等 gz 与 person_odom 彻底消失)"""
import time
import paramiko

HOME = '/home/earl/workspace/VLA'
SEED = '7'
DUR = 150

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()


def sh(cmd, timeout=600, quiet=False):
    _i, o, e = cli.exec_command(cmd, timeout=timeout)
    out = o.read().decode(errors='replace')
    err = e.read().decode(errors='replace')
    if not quiet:
        for ln in out.strip().splitlines()[-6:]:
            print('   ', ln)
        if err.strip():
            print('  STDERR:', err.strip()[:300])
    return out, err


print('==== B (加固清场) ====')
sh('cd %s && pkill -f "ros2 launch vla_follow" ; '
   'pkill -f "vla_follow/lib/vla_follow" ; pkill -f parameter_bridge ; '
   'pkill -f "/opt/ros/jazzy/lib/nav2_" ; pkill -f "gz sim" ; '
   'pkill -f tmp_p74 ; pkill -f tmp_set_follow ; echo killed' % HOME,
   timeout=120)
# 等到 gz / bridge / 节点全部消失
for i in range(20):
    time.sleep(2)
    o, _e = sh('pgrep -f "gz sim" | wc -l; pgrep -f parameter_bridge | wc -l; '
               'pgrep -f "vla_follow/lib/vla_follow" | wc -l', timeout=60,
               quiet=True)
    nums = [x for x in o.split() if x.strip().isdigit()]
    if nums and all(n == '0' for n in nums):
        print('   清场完成 (%ds)' % (2 * (i + 1)))
        break
else:
    print('   WARN 清场超时')
sh('sleep 2 ; echo settle', timeout=60, quiet=True)

sh('cd %s && rm -f det_b.json det_b.log ; '
   'setsid nohup bash p74_rec.sh %d det_b.json > det_b.log 2>&1 < /dev/null & '
   'sleep 3 ; echo rec-started' % (HOME, DUR), timeout=60)
sh('cd %s && DISPLAY=:0 PW_SEED=%s nohup bash vla_demo/start_demo.sh '
   '> demo_b.log 2>&1 & sleep 3 ; echo launched' % (HOME, SEED), timeout=90)
time.sleep(40)
sh('cd %s && setsid nohup python3 tmp_set_follow.py 1.0 %d '
   '> /dev/null 2>&1 < /dev/null & sleep 1 ; echo sf' % (HOME, DUR + 30),
   timeout=60, quiet=True)
for _ in range(70):
    time.sleep(5)
    o, _e = sh('tail -1 %s/det_b.log 2>/dev/null' % HOME, timeout=60, quiet=True)
    if 'wrote' in o:
        print('   ', o.strip())
        break
sh('pkill -f tmp_set_follow ; echo sf-stop', timeout=60, quiet=True)

print('==== compare A vs B ====')
sh('cd %s && python3 tmp_p74_cmp2.py det_a.json det_b.json' % HOME, timeout=180)
print('==== done ====')
sftp.close()
cli.close()
