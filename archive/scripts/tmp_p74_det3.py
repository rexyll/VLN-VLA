"""P74: 两次干净重启 + 同种子录轨迹 + 比对"""
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
sftp.put(r'Z:\VLA\tmp_p74_cmp.py', HOME + '/tmp_p74_cmp.py')

SRC = ('source /opt/ros/jazzy/setup.bash && '
       'source %s/vla_ws/install/setup.bash && ' % HOME)


def sh(cmd, timeout=600, quiet=False):
    _i, o, e = cli.exec_command(cmd, timeout=timeout)
    out = o.read().decode(errors='replace')
    err = e.read().decode(errors='replace')
    if not quiet:
        for ln in out.strip().splitlines()[-5:]:
            print('   ', ln)
        if err.strip():
            print('  STDERR:', err.strip()[:300])
    return out, err


def full_run(tag):
    print('==== %s ====' % tag)
    sh('cd %s && pkill -f "ros2 launch vla_follow" ; '
       'pkill -f "vla_follow/lib/vla_follow" ; pkill -f parameter_bridge ; '
       'pkill -f "/opt/ros/jazzy/lib/nav2_" ; pkill -f "gz sim" ; '
       'pkill -f tmp_p74 ; pkill -f tmp_set_follow ; sleep 5 ; echo cleaned'
       % HOME, timeout=120)
    sh('cd %s && DISPLAY=:0 PW_SEED=%s nohup bash vla_demo/start_demo.sh '
       '> demo_%s.log 2>&1 & echo launched' % (HOME, SEED, tag), timeout=60)
    for _ in range(30):
        time.sleep(3)
        o, _e = sh('grep -c "PERSON 行走" %s/vla_ws/logs/demo.log 2>/dev/null'
                   % HOME, timeout=60, quiet=True)
        if o.strip() and o.strip() != '0':
            break
    time.sleep(8)
    sh('cd %s && (nohup python3 tmp_set_follow.py 1.0 160 >/dev/null 2>&1 &) ; '
       'sleep 1 ; echo sf' % HOME, timeout=60, quiet=True)
    sh('cd %s && %spython3 tmp_p74_det.py --dur 120 --out det_%s.json'
       % (HOME, SRC, tag), timeout=300)
    sh('pkill -f tmp_set_follow ; echo sf-stop', timeout=60, quiet=True)


full_run('a')
full_run('b')
print('==== compare ====')
sh('cd %s && %spython3 tmp_p74_cmp.py det_a.json det_b.json' % (HOME, SRC),
   timeout=180)
print('==== done ====')
sftp.close()
cli.close()
