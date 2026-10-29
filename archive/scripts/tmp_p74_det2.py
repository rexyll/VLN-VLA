"""P74: 同种子两轮确定性验证
run B: 直接录当前实例(已是 use_sim_time 版本)
run A: 重启后录
两轮都全程跑 tmp_set_follow.py 固定距离 1.0, 消除语音"两米"的一次性扰动。
"""
import time
import paramiko

HOME = '/home/earl/workspace/VLA'
SEED = '7'

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()
sftp.put(r'Z:\VLA\tmp_p74_det.py', HOME + '/tmp_p74_det.py')


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


SRC = ('source /opt/ros/jazzy/setup.bash && '
       'source %s/vla_ws/install/setup.bash && ' % HOME)


def record(tag):
    print('-- recording %s (120s sim)' % tag)
    # 固定跟随距离, 24Hz, 覆盖整个录制窗口
    sh('cd %s && (nohup python3 tmp_set_follow.py 1.0 150 >/dev/null 2>&1 &) ; '
       'sleep 1 ; echo setfollow-started' % HOME, timeout=60)
    sh('cd %s && %spython3 tmp_p74_det.py --dur 120 --out det_%s.json'
       % (HOME, SRC, tag), timeout=300)
    sh('pkill -f tmp_set_follow.py ; echo sf-stopped', timeout=60, quiet=True)


# B: 当前实例
print('==== run B (当前实例) ====')
record('b')

# A: 重启同种子
print('==== run A (重启) ====')
sh('cd %s && pkill -f "ros2 launch vla_follow" ; '
   'pkill -f "vla_follow/lib/vla_follow" ; pkill -f parameter_bridge ; '
   'pkill -f "/opt/ros/jazzy/lib/nav2_" ; pkill -f "gz sim" ; '
   'pkill -f tmp_p74 ; sleep 4 ; echo cleaned' % HOME, timeout=120)
sh('cd %s && DISPLAY=:0 PW_SEED=%s nohup bash vla_demo/start_demo.sh '
   '> demo_a.log 2>&1 & echo launched' % (HOME, SEED), timeout=60)
for _ in range(30):
    time.sleep(3)
    o, _e = sh('grep -c "PERSON 行走" %s/vla_ws/logs/demo.log 2>/dev/null' % HOME,
               timeout=60, quiet=True)
    if o.strip() and o.strip() != '0':
        break
time.sleep(8)
record('a')

print('==== done ====')
sftp.close()
cli.close()
