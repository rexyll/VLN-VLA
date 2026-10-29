"""P74: 录制器先起、再启 demo，两轮同种子对比（以人起步为原点）"""
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
for n in ('tmp_p74_det.py', 'tmp_p74_cmp2.py', 'p74_rec.sh'):
    sftp.put('Z:/VLA/' + n, HOME + '/' + n)
cli.exec_command('chmod +x %s/p74_rec.sh' % HOME, timeout=30)[1].read()


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
    # 1) 先起录制器（独立会话，全部 fd 重定向，不占 SSH 通道）
    sh('cd %s && rm -f det_%s.json det_%s.log ; '
       'setsid nohup bash p74_rec.sh %d det_%s.json > det_%s.log 2>&1 < /dev/null & '
       'sleep 3 ; echo rec-started' % (HOME, tag, tag, DUR, tag, tag),
       timeout=60)
    # 2) 再启 demo
    sh('cd %s && DISPLAY=:0 PW_SEED=%s nohup bash vla_demo/start_demo.sh '
       '> demo_%s.log 2>&1 & sleep 3 ; echo launched' % (HOME, SEED, tag),
       timeout=90)
    # 3) 固定跟随距离
    time.sleep(40)
    sh('cd %s && setsid nohup python3 tmp_set_follow.py 1.0 %d '
       '> /dev/null 2>&1 < /dev/null & sleep 1 ; echo sf' % (HOME, DUR + 30),
       timeout=60, quiet=True)
    # 4) 等录制完成
    for _ in range(70):
        time.sleep(5)
        o, _e = sh('tail -1 %s/det_%s.log 2>/dev/null' % (HOME, tag),
                   timeout=60, quiet=True)
        if 'wrote' in o:
            print('   ', o.strip())
            break
    sh('pkill -f tmp_set_follow ; echo sf-stop', timeout=60, quiet=True)


full_run('a')
full_run('b')
print('==== compare ====')
sh('cd %s && python3 tmp_p74_cmp2.py det_a.json det_b.json' % HOME, timeout=180)
print('==== done ====')
sftp.close()
cli.close()
