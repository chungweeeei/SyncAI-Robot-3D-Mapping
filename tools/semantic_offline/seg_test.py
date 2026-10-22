"""Time and eyeball the 2D backends on four bag images: seg_test.py <bag> <out> <model>..."""

import sys
import time
from pathlib import Path
import cv2
import numpy as np
from bagio import ImageIndex
from segment import make_segmenter, model_tag

bag = Path(sys.argv[1])
out = Path(sys.argv[2])
out.mkdir(exist_ok=True)
idx = ImageIndex(bag)
want = [900, 2700, 4500, 6000]
if max(want) >= len(idx.stamps):  # a shorter bag (or one file of a split one): four spread-out images
    want = [int(len(idx.stamps) * f) for f in (0.12, 0.37, 0.62, 0.87)]
imgs = {j: img for j, _, img in idx.read_many(want)}
for model in sys.argv[3:]:
    seg = make_segmenter(model)
    tag = model_tag(model)
    seg(imgs[want[0]])  # warm-up
    t = time.time()
    for j, img in imgs.items():
        lab, conf, bnd = seg(img)
        vis = cv2.addWeighted(img, 0.45, seg.colorize(lab), 0.55, 0)
        vis[bnd] = (255, 255, 255)
        ids, counts = np.unique(lab[lab != 255], return_counts=True)
        top = sorted(zip(counts, ids), reverse=True)[:6]
        txt = ", ".join(f"{seg.id2label[int(i)].split(',')[0]} {c / lab.size:.0%}" for c, i in top)
        cv2.putText(vis, f"{tag}: {txt}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(
            vis, f"{tag}: {txt}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1
        )
        cv2.imwrite(str(out / f"seg_{j}_{tag}.jpg"), vis, [cv2.IMWRITE_JPEG_QUALITY, 85])
        print(f"{tag} image {j}: {txt} | mean conf {conf.mean():.2f} boundary {bnd.mean():.1%}")
    print(f"{tag}: {(time.time() - t) / len(imgs):.2f} s / image on {seg.device}")
