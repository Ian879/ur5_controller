#!/usr/bin/env python3
"""直接探 SAM server：用 look-pose 图，不同 prompt 点，看 mask 是否随点变化。
隔离 SAM server 行为（不经 ROS）。
"""
import base64, json, urllib.request, sys
import cv2, numpy as np

URL = "http://10.70.76.200:7890/api/sam/segment"
IMG = "/tmp/look_color.png"   # look-pose 图，cola 在中心

img = cv2.imread(IMG)
if img is None:
    print("读不到", IMG); sys.exit(1)
h, w = img.shape[:2]
print(f"图 {w}x{h}")

def call_sam(points, label=None):
    _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    boundary = "----SAMBoundary"
    body = (
        f"--{boundary}\r\n".encode()
        + b'Content-Disposition: form-data; name="image"; filename="frame.jpg"\r\n'
        + b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes()
        + f"\r\n--{boundary}\r\n".encode()
        + b'Content-Disposition: form-data; name="points"\r\n\r\n'
        + json.dumps(points).encode()
        + f"\r\n--{boundary}--\r\n".encode()
    )
    req = urllib.request.Request(URL, data=body, method="POST")
    req.add_header("Content-Type", f"multipart/form-data; boundary={boundary}")
    resp = urllib.request.urlopen(req, timeout=15)
    return json.loads(resp.read())

# 测试点：cola 中心(640,360)，桌面下方(640,650)，左腕(100,400)，无点(空)
tests = [
    ("cola中心", [[640, 360]]),
    ("桌面下方", [[640, 650]]),
    ("左腕", [[100, 400]]),
]
for name, pts in tests:
    try:
        r = call_sam(pts, name)
    except Exception as e:
        print(f"[{name}] 调用失败: {e}"); continue
    print(f"\n=== [{name}] prompt={pts} ===")
    print(f"  返回 keys: {list(r.keys())}")
    print(f"  score={r.get('score')}  其它: " + ", ".join(f"{k}={str(v)[:60]}" for k,v in r.items() if k not in ('mask_png_b64','mask')))
    mb = r.get("mask_png_b64") or r.get("mask")
    if not mb:
        print("  ❌ 无 mask 字段"); continue
    mask = cv2.imdecode(np.frombuffer(base64.b64decode(mb), np.uint8), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        print("  ❌ mask 解码失败"); continue
    fg = mask > 127
    cov = 100*fg.sum()/(mask.shape[0]*mask.shape[1])
    ys, xs = np.where(fg)
    bbox = (int(xs.min()),int(xs.max()),int(ys.min()),int(ys.max())) if len(xs) else None
    print(f"  mask {mask.shape} dtype={mask.dtype} 唯一值数={len(np.unique(mask))} 覆盖={cov:.1f}% bbox={bbox}")
    print(f"  mask 唯一值(前10): {sorted(np.unique(mask).tolist())[:10]}")
    cv2.imwrite(f"/tmp/samtest_{name}.png", mask)
