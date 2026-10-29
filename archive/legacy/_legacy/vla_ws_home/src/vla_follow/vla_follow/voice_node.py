#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""voice_node.py —— 真人语音指令识别与解析 (Vosk 离线中文引擎)

输入: /voice/request (std_msgs/String)
     - 内容为 16kHz 单声道 WAV 文件路径(真人语音), 或 "mic" 打开麦克风
     - 另外自动监听 ~/vla_demo/audio/inbox/ 目录, 丢入的 WAV 会被识别
输出: /robot/command (std_msgs/String, JSON)
     - {"action": "FOLLOW", "distance": 2.0, ...}   跟随指令(米)
     - {"action": "STOP"}                            停止指令
     - {"action": "UNKNOWN", ...}                    未识别
"""
import json
import os
import re
import shutil
import subprocess
import threading
import time
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CN_NUM = {'一': 1, '二': 2, '两': 2, '三': 3, '四': 4,
          '五': 5, '六': 6, '七': 7, '八': 8, '九': 9}
STOP_WORDS = ('停止', '停下', '停一下', '站住', '别跟', '不要跟', '取消跟随')
FOLLOW_WORDS = ('跟随', '跟着', '跟上', '跟住', '随我', '跟在我')
VLA_HOME = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')
INBOX_DIR = os.path.join(VLA_HOME, 'vla_demo', 'audio', 'inbox')
PROCESSED_DIR = os.path.join(VLA_HOME, 'vla_demo', 'audio', 'processed')


def parse_cn_number(s):
    """'2'/'2.5'/'两'/'十'/'二十五' -> float"""
    s = s.strip()
    if not s:
        return None
    if re.fullmatch(r'[0-9]+(\.[0-9]+)?', s):
        return float(s)
    total, num = 0, 0
    for ch in s:
        if ch == '十':
            total += (num or 1) * 10
            num = 0
        elif ch in CN_NUM:
            num = CN_NUM[ch]
        else:
            return None
    return float(total + num)


class VoiceNode(Node):

    def __init__(self):
        super().__init__('voice_node')
        self.declare_parameter(
            'model_path',
            os.path.join(VLA_HOME, 'vla_demo', 'vosk-model-small-cn-0.22'))
        self.model_path = self.get_parameter('model_path').value
        self.pub_cmd = self.create_publisher(String, '/robot/command', 10)
        self.create_subscription(String, '/voice/request', self._on_request, 10)
        self.model = None
        self._busy = threading.Lock()
        threading.Thread(target=self._load_model, daemon=True).start()
        os.makedirs(INBOX_DIR, exist_ok=True)
        os.makedirs(PROCESSED_DIR, exist_ok=True)
        threading.Thread(target=self._inbox_loop, daemon=True).start()
        self.get_logger().info(
            'VOICE 真人语音识别节点已启动 (Vosk 离线中文, 等待语音指令...)')

    def _load_model(self):
        from vosk import Model
        t0 = time.time()
        self.model = Model(self.model_path)
        self.get_logger().info(
            f'VOICE 中文声学模型加载完成: {self.model_path} '
            f'({time.time() - t0:.1f}s), 可以开始识别')

    # ---------- 输入入口 ----------
    def _on_request(self, msg):
        threading.Thread(target=self._recognize_any, args=(msg.data,),
                         daemon=True).start()

    def _inbox_loop(self):
        while True:
            try:
                for name in os.listdir(INBOX_DIR):
                    if name.lower().endswith('.wav'):
                        src = os.path.join(INBOX_DIR, name)
                        self._recognize_any(src)
                        shutil.move(src, os.path.join(PROCESSED_DIR, name))
            except Exception:
                pass
            time.sleep(2.0)

    # ---------- 识别 ----------
    def _recognize_any(self, src):
        if src == 'mic':
            return self._recognize_mic()
        if self.model is None:
            self.get_logger().warn('VOICE 声学模型尚未加载完成, 请稍后重试')
            return
        if not os.path.isfile(src):
            self.get_logger().warn(f'VOICE 音频文件不存在: {src}')
            return
        try:
            wf = wave.open(src, 'rb')
        except Exception as e:
            self.get_logger().warn(f'VOICE 无法读取音频 {src}: {e}')
            return
        if wf.getframerate() != 16000 or wf.getnchannels() != 1:
            self.get_logger().warn(
                f'VOICE 音频需为 16kHz 单声道 WAV (实际 {wf.getframerate()}Hz/'
                f'{wf.getnchannels()}ch): {src}')
            wf.close()
            return
        with self._busy:
            self._recognize_file(src, wf)

    def _recognize_file(self, src, wf):
        from vosk import KaldiRecognizer
        rec = KaldiRecognizer(self.model, 16000)
        rec.SetWords(False)
        name = os.path.basename(src)
        self.get_logger().info(f'VOICE 收到真人语音指令音频: {name}, 正在识别...')
        t0 = time.time()
        text = ''
        while True:
            data = wf.readframes(4000)
            if len(data) == 0:
                break
            if rec.AcceptWaveform(data):
                r = json.loads(rec.Result())
                if r.get('text', '').strip():
                    text = r['text'].strip()
            else:
                part = json.loads(rec.PartialResult()).get('partial', '').strip()
                if part:
                    self.get_logger().info(f'VOICE 识别中... "{part}"')
        final = json.loads(rec.FinalResult()).get('text', '').strip() or text
        wf.close()
        self.get_logger().info(
            f'VOICE ★ 识别结果("{name}"): "{final}" (耗时 {time.time() - t0:.1f}s)')
        self._dispatch(final, name)

    def _recognize_mic(self):
        if self.model is None:
            self.get_logger().warn('VOICE 声学模型尚未加载完成')
            return
        try:
            p = subprocess.Popen(
                ['arecord', '-q', '-t', 'raw', '-f', 'S16_LE', '-r', '16000',
                 '-c', '1'], stdout=subprocess.PIPE)
        except Exception as e:
            self.get_logger().warn(f'VOICE 无法打开麦克风: {e}')
            return
        from vosk import KaldiRecognizer
        rec = KaldiRecognizer(self.model, 16000)
        self.get_logger().info('VOICE 麦克风监听中(最长 15 秒), 请说话...')
        deadline = time.time() + 15.0
        try:
            while time.time() < deadline:
                data = p.stdout.read(4000)
                if not data:
                    break
                if rec.AcceptWaveform(data):
                    t = json.loads(rec.Result()).get('text', '').strip()
                    if t:
                        self.get_logger().info(f'VOICE ★ 识别结果(麦克风): "{t}"')
                        self._dispatch(t, 'microphone')
                        break
        finally:
            p.terminate()

    # ---------- 解析 ----------
    def _dispatch(self, text, source):
        norm = re.sub(r'[\s,，。.、!?!?？:：]', '', text)
        if any(w in norm for w in STOP_WORDS):
            cmd = {'action': 'STOP'}
        else:
            m = re.search(r'([0-9]+(?:\.[0-9]+)?|[一二两三四五六七八九十]+)\s*米',
                          norm)
            dist = parse_cn_number(m.group(1)) if m else None
            if any(w in norm for w in FOLLOW_WORDS):
                # 带距离(如"保持两米")则携带; 无距离由 follower 用配置默认(P15)
                cmd = {'action': 'FOLLOW'}
                if dist is not None:
                    cmd['distance'] = dist
            else:
                cmd = {'action': 'UNKNOWN'}
        cmd['text'] = text
        cmd['source'] = source
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False)
        self.pub_cmd.publish(msg)
        self.get_logger().info(
            f'VOICE ★ 解析为机器指令: {json.dumps(cmd, ensure_ascii=False)}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
