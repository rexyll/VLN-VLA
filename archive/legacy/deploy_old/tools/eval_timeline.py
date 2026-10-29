import json, sys

fname = sys.argv[1] if len(sys.argv) > 1 else 'eval14.json'
d = json.load(open(fname))
recs = d['recs']
t0 = recs[0]['t']
print('n=%d span=%.1fs' % (len(recs), recs[-1]['t'] - t0))

# 每 3s 一行
bucket = {}
for r in recs:
    k = int((r['t'] - t0) // 3)
    bucket.setdefault(k, []).append(r)

hdr = '%5s %7s %7s %6s %6s %6s %6s %6s %5s %-6s %-8s %s' % (
    't', 'rx', 'ry', 'px', 'py', 'd', 'vis_d', 'gap', 'lost', 'cam', 'track', 'note')
print(hdr)
for k in sorted(bucket):
    rs = bucket[k]
    ds = [r['d'] for r in rs]
    vd = [r['vis_d'] for r in rs if r.get('vis_d', -1) > 0]
    gaps = [r['gap'] for r in rs if r.get('gap') is not None]
    lost_n = sum(1 for r in rs if r.get('lost'))
    last = rs[-1]
    print('%5.0f %7.2f %7.2f %6.2f %6.2f %6.2f %6.2f %6.2f %5d %-6s %-8s v=%+.2f w=%+.2f gt=%s' % (
        k * 3,
        sum(r['rx'] for r in rs) / len(rs), sum(r['ry'] for r in rs) / len(rs),
        sum(r['px'] for r in rs) / len(rs), sum(r['py'] for r in rs) / len(rs),
        sum(ds) / len(ds),
        (sum(vd) / len(vd)) if vd else -1.0,
        (min(gaps) if gaps else -9),
        lost_n, last.get('cam', ''), last.get('track', ''),
        last.get('cmd_v', 0.0), last.get('cmd_w', 0.0),
        str(last.get('goto')),
    ))
