import time

import paramiko

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)

RUNS = [('r7', 'rpl'), ('r8', 'rpl')]
for tag, mode in RUNS:
    t = time.time()
    i, o, e = cli.exec_command(
        'cd /home/earl/workspace/VLA && bash p75_one.sh %s 100 %s 2>&1 | tail -18'
        % (tag, mode), timeout=900)
    out = o.read().decode(errors='replace')
    print('#### %s / %s  (%.0fs)' % (tag, mode, time.time() - t))
    print(out)
    print(flush=True)
cli.close()
