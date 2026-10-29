#!/usr/bin/env python3
"""把 xwd 截图转 png(无 ImageMagick 时的兜底)。"""
import struct
import sys

import numpy as np
from PIL import Image


def conv(src, dst):
    b = open(src, 'rb').read()
    (hsz, ver, fmt, depth, w, h, xo, yo, order, bunit, bbit, bpad,
     bpp, bpl) = struct.unpack('>14I', b[:56])
    ncol = struct.unpack('>I', b[80:84])[0]   # 偏移 76 是 colormap_entries
    off = hsz + ncol * 12
    print('hdr %dx%d depth=%d bpp=%d bpl=%d order=%d ncol=%d hsz=%d off=%d total=%d' % (
        w, h, depth, bpp, bpl, order, ncol, hsz, off, len(b)))
    if order == 0:          # LSB
        arr = np.frombuffer(b[off:off + bpl * h], dtype=np.uint8)
        arr = arr.reshape(h, bpl)
        if bpp == 32:
            px = arr[:, :w * 4].reshape(h, w, 4)
            rgb = px[:, :, [2, 1, 0]]
        elif bpp == 24:
            px = arr[:, :w * 3].reshape(h, w, 3)
            rgb = px[:, :, [2, 1, 0]]
        else:
            raise SystemExit('unsupported bpp %d' % bpp)
    else:
        arr = np.frombuffer(b[off:off + bpl * h], dtype='>u4')
        arr = arr.reshape(h, bpl // 4)[:, :w]
        rgb = np.stack([(arr >> 16) & 255, (arr >> 8) & 255, arr & 255],
                       axis=-1).astype(np.uint8)
    Image.fromarray(rgb).save(dst)
    print('saved %s %dx%d bpp=%d' % (dst, w, h, bpp))


if __name__ == '__main__':
    conv(sys.argv[1], sys.argv[2])
