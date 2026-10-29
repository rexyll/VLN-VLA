"""P74: 重启 demo(PW_SEED=7) 并跑两轮确定性轨迹记录"""
import sys
import time
import paramiko

HOME = '/home/earl/workspace/VLA'

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
        tail = '\n'.join(out.strip().splitlines()[-6:])
        if tail:
            print(tail)
        if err.strip():
            print('STDERR:', err.strip()[:300])
    return out, err


sftp.put(r'Z:\VLA\tmp_p74_det.py', HOME + '/tmp_p74_det.py')
print('uploaded det script')

for tag in ('a', 'b'):
    print('==== run %s ====' % tag)
    sh('cd %s && pkill -f "ros2 launch vla_follow" ; '
       'pkill -f "vla_follow/lib/vla_follow" ; pkill -f parameter_bridge ; '
       'pkill -f "/opt/ros/jazzy/lib/nav2_" ; pkill -f "gz sim" ; '
       'pkill -f p74_det ; sleep 4 ; echo cleaned' % HOME, timeout=120)
    sh('cd %s && DISPLAY=:0 PW_SEED=7 nohup bash vla_demo/start_demo.sh '
       '> demo_%s.log 2>&1 & echo launched' % (HOME, tag), timeout=60)
    # 等节点就绪(最多 90s), 看到 PERSON "人"节点就绪 即认为起来
    for _ in range(30):
        time.sleep(3)
        out, _e = sh('grep -c "PERSON" %s/demo_%s.log 2>/dev/null' % (HOME, tag),
                     timeout=60, quiet=True)
        if out.strip() and out.strip() != '0':
            break
    sh('grep -E "PERSON|随机种子|use_sim_time|Traceback|Error" %s/demo_%s.log | head -6'
       % (HOME, tag), timeout=60)
    time.sleep(6)
    sh('cd %s && DISPLAY=:0 python3 tmp_p74_det.py --dur 120 --out det_%s.json'
       % (HOME, tag), timeout=300)

sftp.close()
cli.close()
print('done')
