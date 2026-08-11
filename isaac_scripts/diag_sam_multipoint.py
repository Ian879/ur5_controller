#!/usr/bin/env python3
"""测 SAM 多点 prompt 能否把 mask 收紧到 cola。"""
import base64, json, urllib.request, cv2, numpy as np
URL = "http://10.70.76.200:7890/api/sam/segment"
img = cv2.imread("/tmp/look_color.png")
_, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])


def call(points):
    b = "----b"
    body = (f"--{b}\r\n".encode()
            + b'Content-Disposition: form-data; name="image"; filename="f.jpg"\r\n'
            + b"Content-Type: image/jpeg\r\n\r\n" + buf.tobytes()
            + f"\r\n--{b}\r\n".encode())
    body += b'Content-Disposition: form-data; name="points"\r\n\r\n'
    body += json.dumps(points).encode() + f"\r\n--{b}--\r\n".encode()
    req = urllib.request.Request(URL, data=body, method="POST")
    req.add_header("Content-Type", f"multipart/form-data; boundary={b}")
    r = json.loads(urllib.request.urlopen(req, timeout=15).read())
    m = cv2.imdecode(np.frombuffer(base64.b64decode(r["mask_png_b64"]), np.uint8), cv2.IMREAD_GRAYSCALE)
    fg = m > 127
    cov = 100 * fg.sum() / (m.shape[0] * m.shape[1])
    ys, xs = np.where(fg)
    bbox = (int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())) if len(xs) else None
    return cov, bbox, r.get("score"), m


pts9 = [[632, 356], [640, 356], [648, 356], [632, 364], [640, 364], [648, 364], [632, 372], [640, 372], [648, 372]]
tests = {
    "1点中心": [[640, 360]],
    "4点cola": [[632, 358], [648, 358], [640, 350], [640, 366]],
    "9点cola": pts9,
}
for name, P in tests.items():
    try:
        cov, bbox, sc, m = call(P)
        cv2.imwrite(f"/tmp/sammulti_{name}.png", m)
        print(f"{name}: cov={cov:.1f}% bbox={bbox} score={float(sc):.3f}")
    except Exception as e:
        print(f"{name}: {e}")
