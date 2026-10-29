"""追加 P74 记忆（VM 侧写）"""
import os
import paramiko

HOME = '/home/earl/workspace/VLA'
TARGET = HOME + '/.workbuddy/memory/2026-10-25.md'
LOCAL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     'tmp_p74_memory.md')

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()

with sftp.file(TARGET, 'r') as f:
    old = f.read().decode('utf-8')
add = open(LOCAL, encoding='utf-8').read()

if '## P74 评测可重复性改造' in old:
    print('already appended, skip')
else:
    with sftp.file(TARGET, 'w') as f:
        f.write((old.rstrip('\n') + '\n' + add).encode('utf-8'))
    print('appended')

with sftp.file(TARGET, 'r') as f:
    new = f.read().decode('utf-8')
print('chars %d -> %d' % (len(old), len(new)))
print('verify:', new.count('## P74 评测可重复性改造'))
sftp.close()
cli.close()
