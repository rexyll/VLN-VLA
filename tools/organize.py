#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""organize.py —— VLA 项目目录归置

把散落在项目根的 900+ 个文件按「代码 / 文档 / 测试 / 工具 / 归档」五类归位。

用法:
    python3 organize.py --root <项目根> [--apply]

不带 --apply 只打印分类统计(dry-run)。
"""
import os
import sys
import shutil

# ---------------------------------------------------------------- 精确映射
# 相对项目根的路径 -> 目标相对路径
EXACT = {
    # ---- 文档 ----
    'TaskPlan.md':                     'docs/TaskPlan.md',
    'VLA实施方案.md':                   'docs/VLA实施方案.md',
    '目录归置与两端同步说明.md':          'docs/目录归置与两端同步说明.md',
    'p82_section23.md':                'docs/reports/P82-贴身死锁与帧率定案.md',
    'tmp_p73_memory.md':               'docs/reports/P73-记录.md',
    'tmp_p74_memory.md':               'docs/reports/P74-记录.md',
    'tmp_p73_sec13.md':                'docs/reports/P73-章节.md',
    'tmp_p74_sec14.md':                'docs/reports/P74-章节.md',
    'tmp_p75_sec15.md':                'docs/reports/P75-章节.md',
    'tmp_p76_sec16.md':                'docs/reports/P76-章节.md',
    # deploy 里的报告
    'deploy/代码实际进度与TaskPlan一致性核对报告.md':
        'docs/reports/代码实际进度与TaskPlan一致性核对报告.md',
    'deploy/蓝色胶囊障碍物-根因与修复.md':
        'docs/reports/蓝色胶囊障碍物-根因与修复.md',
    'deploy/跟随打转问题-诊断与修复报告.md':
        'docs/reports/跟随打转问题-诊断与修复报告.md',
    'deploy/近距离障碍-丢目标与打转-根因与修复.md':
        'docs/reports/近距离障碍-丢目标与打转-根因与修复.md',
    'deploy/避障稳定性-修复报告.md':
        'docs/reports/避障稳定性-修复报告.md',
    'deploy/Gazebo轨迹可视化-说明.md':
        'docs/reports/Gazebo轨迹可视化-说明.md',

    # ---- 测试: 评测链 (eval_repeat -> eval_once -> eval_run) ----
    'tmp_eval_rep.sh':                 'tests/eval_repeat.sh',
    'p75_eval.sh':                     'tests/eval_once.sh',
    'tmp_run_eval.sh':                 'tests/eval_run.sh',
    'tmp_set_follow.py':               'tests/set_follow.py',
    'tmp_random_eval.py':              'tests/random_eval.py',
    'tmp_follow.sh':                   'tests/follow_cmd.sh',
    'tmp_boot.sh':                     'tests/boot_demo.sh',
    'walk_rec.jsonl':                  'tests/data/walk_rec.jsonl',

    # ---- 测试: 受控场景 ----
    'tmp_orbit_test.py':               'tests/test_orbit.py',
    'tmp_hide_test.py':                'tests/test_hide.py',
    'tmp_stick_test.py':               'tests/test_stick.py',
    'tmp_official_test.py':            'tests/test_official.py',

    # ---- 测试: 诊断探针 ----
    'tmp_snap4.py':                    'tests/diag/snap_cameras.py',
    'tmp_v_sweep.py':                  'tests/diag/sweep_brightness.py',
    'tmp_det_trace.py':                'tests/diag/trace_detector.py',
    'tmp_los_probe.py':                'tests/diag/probe_los.py',
    'tmp_cam_check.py':                'tests/diag/check_camera.py',
    'tmp_depth_diag.py':               'tests/diag/diag_depth.py',
    'tmp_draw_fov.py':                 'tests/diag/draw_fov.py',
    'tmp_dump_pose.py':                'tests/diag/dump_pose.py',
    'tmp_gz_snap.py':                  'tests/diag/gz_snapshot.py',
    'tmp_rate.py':                     'tests/diag/topic_rate.py',
    'tmp_rate2.py':                    'tests/diag/topic_rate2.py',
    'tmp_logstat.py':                  'tests/diag/log_stat.py',

    # ---- 工具: 双端同步 ----
    'vmsync.py':                       'tools/vmsync.py',
    'vmsync_run.sh':                   'tools/vmsync_run.sh',
    'vmput.py':                        'tools/vmput.py',
    'vm.py':                           'tools/vm.py',
    'vm_ssh.py':                       'tools/vm_ssh.py',
    'sync_deploy.py':                  'tools/sync_deploy.py',
    'migrate.sh':                      'tools/migrate.sh',

    # ---- 归档: 旧打包 / 旧模型 ----
    'VLA代码包.zip':                    'archive/packages/VLA代码包.zip',
}

# ---------------------------------------------------------------- 目录映射
DIR_MAP = {
    'VLA知识库':        'docs/knowledge',
    '_legacy':          'archive/legacy/_legacy',
    'demo_follow':      'archive/legacy/demo_follow',
    '.stage_p0':        'archive/legacy/.stage_p0',
    '.bak_p0':          'archive/legacy/.bak_p0',
    '_tmp_artifacts':   'archive/artifacts/_tmp_artifacts',
    'tmp_cmp':          'archive/artifacts/tmp_cmp',
    'cap_png':          'archive/images/cap_png',
    'cap_pillar':       'archive/images/cap_pillar',
    'shots':            'archive/images/shots',
    'logs':             'archive/old-logs',
    '__pycache__':      None,          # 编译缓存, 直接删
}

# ROS 包内的 .bak_* 备份 -> archive/scripts/backup_ros/
ROS_BAK_DIRS = [
    'vla_ws/src/vla_follow/vla_follow',
    'vla_ws/src/vla_follow/worlds',
]

# ---------------------------------------------------------------- 规则映射
EXT_DIR = {
    '.log': 'archive/logs', '.out': 'archive/logs', '.done': 'archive/logs',
    '.json': 'archive/data', '.jsonl': 'archive/data', '.csv': 'archive/data',
    '.png': 'archive/images', '.xwd': 'archive/images',
    '.b64': 'archive/images', '.raw': 'archive/images',
    '.txt': 'archive/data',
    '.sdf': 'archive/sdf',
    '.zip': 'archive/packages',
    '.wav': 'archive/audio',
}

SKIP_TOP = {'vla_ws', 'vla_demo', 'deploy', '.workbuddy', '.git',
            'docs', 'tests', 'tools', 'archive'}

# 评测产物(json/jsonl)单独存一份, 便于跨版本回归对比
import re
EVAL_RE = re.compile(r'^(eval|e\d|t\d+_|p\d+[a-z]*_|p\d+[a-z]*\d*_|demo).*\.(json|jsonl)$')
# 旧版本 follower 快照
VER_RE = re.compile(r'.*follower.*\.py$', re.I)


def classify_top(name, isdir):
    """对顶层条目做规则分类, 返回目标相对路径(文件)或 None(跳过)"""
    if isdir:
        return DIR_MAP.get(name, 'archive/misc/' + name)
    if name in EXACT:
        return EXACT[name]
    # 旧版本快照优先于通用规则
    if name.endswith('.py') and VER_RE.match(name):
        return 'archive/versions/' + name
    if EVAL_RE.match(name):
        return 'archive/data/eval/' + name
    ext = os.path.splitext(name)[1].lower()
    if ext in EXT_DIR:
        return EXT_DIR[ext] + '/' + name
    if ext in ('.py', '.sh'):
        return 'archive/scripts/' + name
    return 'archive/misc/' + name


def main():
    args = [a for a in sys.argv[1:]]
    apply_ = '--apply' in args
    root = os.getcwd()
    if '--root' in args:
        root = args[args.index('--root') + 1]
    root = os.path.abspath(root)

    plan = []          # (src_rel, dst_rel)
    removes = []

    # 1) 顶层条目
    for name in sorted(os.listdir(root)):
        src = os.path.join(root, name)
        if name in SKIP_TOP:
            continue
        isdir = os.path.isdir(src)
        dst_rel = classify_top(name, isdir)
        if dst_rel is None:
            removes.append(name)
        else:
            plan.append((name, dst_rel))

    # 2) ROS 包内 .bak_*
    for d in ROS_BAK_DIRS:
        full = os.path.join(root, d)
        if not os.path.isdir(full):
            continue
        for name in sorted(os.listdir(full)):
            if '.bak' in name or name.endswith('.src_old'):
                plan.append((d + '/' + name,
                             'archive/scripts/backup_ros/' + name))

    # 3) deploy 里的 .bak
    depbak = os.path.join(root, 'deploy')
    if os.path.isdir(depbak):
        for name in sorted(os.listdir(depbak)):
            if os.path.isfile(os.path.join(depbak, name)) and \
                    ('.bak' in name or name.startswith('_')):
                plan.append(('deploy/' + name,
                             'archive/scripts/backup_ros/' + name))

    # ---- 报告 ----
    from collections import Counter
    cnt = Counter(dst.split('/')[0] + '/' +
                  (dst.split('/')[1] if '/' in dst else '')
                  for _, dst in plan)
    print('=== 归置计划 (root=%s, %s) ===' %
          (root, 'APPLY' if apply_ else 'DRY-RUN'))
    for k, v in sorted(cnt.items()):
        print('  %-28s %5d' % (k, v))
    print('  删除(编译缓存): %s' % (len(removes)))
    print('  合计移动: %d 项' % len(plan))
    if not apply_:
        print('\n(dry-run, 未改动任何文件; 加 --apply 执行)')
        return

    # ---- 执行 ----
    n_ok = n_skip = 0
    for src_rel, dst_rel in plan:
        s = os.path.join(root, src_rel)
        d = os.path.join(root, dst_rel)
        if not os.path.exists(s):
            n_skip += 1
            continue
        os.makedirs(os.path.dirname(d), exist_ok=True)
        if os.path.exists(d):
            d = d + '.dup'
        try:
            shutil.move(s, d)
            n_ok += 1
        except Exception as e:
            print('  !! 失败 %s -> %s : %s' % (src_rel, dst_rel, e))
    for name in removes:
        try:
            shutil.rmtree(os.path.join(root, name))
        except Exception as e:
            print('  !! 删除失败 %s: %s' % (name, e))
    print('移动 %d 项, 跳过(不存在) %d 项, 删除 %d 个缓存目录' %
          (n_ok, n_skip, len(removes)))


if __name__ == '__main__':
    main()
