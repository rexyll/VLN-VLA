"""P74: 只重跑 B（严格门禁: 必须确认无 gz/bridge/节点 才开录制）"""
import time
import paramiko

HOME = '/home/earl/workspace/VLA'
SEED = '7'
DUR = 110

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()
sftp.put('Z:/VLA/tmp_p74_det.py', HOME + '/tmp_p74_det.py')


def sh(cmd, timeout=600, quiet=False):
    _i, o, e = cli.exec_command(cmd, timeout=timeout)
    out = o.read().decode(errors='replace')
    err = e.read().decode(errors='replace')
    if not quiet:
        for ln in out.strip().splitlines()[-6:]:
            print('   ', ln)
        if err.strip():
            print('  STDERR:', err.strip()[:400])
    return out, err


print('==== 硬清场 ====')
for _ in range(3):
    sh('pkill -9 -f "ros2 launch vla_follow" ; pkill -9 -f "vla_follow/lib" ; '
       'pkill -9 -f parameter_bridge ; pkill -9 -f "/opt/ros/jazzy/lib/nav2_" ; '
       'pkill -9 -f "gz sim" ; pkill -9 -f "gz-sim" ; '
       'pkill -9 -f tmp_p74_det ; pkill -9 -f tmp_set_follow ; '
       'pkill -9 -f start_demo ; echo cycle', timeout=90, quiet=True)
    time.sleep(3)
    o, _e = sh('pgrep -fc "gz sim"; pgrep -fc parameter_bridge; '
               'pgrep -fc "vla_follow/lib"; pgrep -fc tmp_p74_det',
               timeout=60, quiet=True)
    nums = [x for x in o.split() if x.strip().isdigit()]
    print('   残留 gz/bridge/vla/rec =', nums)
    if nums and all(n == '0' for n in nums):
        break
else:
    print('   仍有残留, 继续尝试')
time.sleep(3)

print('==== 起录制器 ====')
sh('cd %s && rm -f det_b.json det_b.log ; '
   'setsid nohup bash p74_rec.sh %d det_b.json > det_b.log 2>&1 < /dev/null & '
   'sleep 3 ; echo rec-started' % (HOME, DUR), timeout=60)
print('==== 起 demo ====')
sh('cd %s && DISPLAY=:0 PW_SEED=%s nohup bash vla_demo/start_demo.sh '
   '> demo_b.log 2>&1 & sleep 5 ; echo launched' % (HOME, SEED), timeout=120)
time.sleep(35)
sh('cd %s && setsid nohup python3 tmp_set_follow.py 1.0 %d '
   '> /dev/null 2>&1 < /dev/null & sleep 1 ; echo sf' % (HOME, DUR + 40),
   timeout=60, quiet=True)

for i in range(50):
    time.sleep(6)
    o, _e = sh('cat %s/det_b.log 2>/dev/null | tr "\\n" "|"' % HOME,
               timeout=60, quiet=True)
    if 'wrote' in o:
        print('   ', o.strip()[:300])
        break
    if i % 5 == 0:
        print('   [%ds] %s' % (6 * (i + 1), o.strip()[:160]))
sh('pkill -f tmp_set_follow ; echo sf-stop', timeout=60, quiet=True)

print('==== compare A vs B ====')
sh('cd %s && python3 tmp_p74_cmp2.py det_a.json det_b.json' % HOME, timeout=120)
print('==== done ====')
sftp.close()
cli.close()
