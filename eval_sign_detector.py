#!/usr/bin/env python3
# =============================================================================
# eval_sign_detector.py
# -----------------------------------------------------------------------------
# NXP Cup India 2026 - offline evaluation for the sign board detector.
# NOT part of the submission.
#
# WHY THIS EXISTS
#   Detection loss is not interpretable in absolute terms - a val loss of 0.98
#   says nothing about whether glyphs are actually found. Before spending more
#   epochs we need the answers that matter:
#
#     1. Per-class precision / recall at IoU 0.5. Are the letters and arrows
#        being detected at all, and is any single class failing?
#     2. ROUTING ACCURACY - the metric the mission actually depends on. After
#        pairing letters to arrows, does the predicted table
#        ("A->Left, B->Right, ...") match the ground truth table for that
#        image? A model can have mediocre box quality and still route
#        perfectly, or excellent boxes and pair badly.
#
#   Optionally writes annotated images so the failures can be inspected by eye,
#   which is usually faster than staring at numbers.
#
# USAGE
#   python3 eval_sign_detector.py --data <coco_root> --model sign_detector.pt
#   python3 eval_sign_detector.py --data ... --save-vis out_dir --limit 40
# =============================================================================

import argparse
import json
import os
from collections import defaultdict

import numpy as np
import torch
import torchvision
from PIL import Image, ImageDraw
from torchvision.models.detection.anchor_utils import AnchorGenerator
from torchvision.transforms import functional as TF

LETTERS = {'A', 'B', 'C', 'X', 'Y', 'Z'}
ARROWS = {'Left', 'Right', 'Straight'}

PAIR_MAX_DX_MULT = 2.0
PAIR_MAX_DY_MULT = 5.0


# =============================================================================
# PAIRING  (kept identical to the runtime node so results transfer)
# =============================================================================

def pair_letters_with_arrows(dets):
    letters = [d for d in dets if d['name'] in LETTERS]
    arrows = [d for d in dets if d['name'] in ARROWS]
    if not letters or not arrows:
        return {}

    table, used = {}, set()
    for letter in sorted(letters, key=lambda d: -d.get('score', 1.0)):
        max_dx = PAIR_MAX_DX_MULT * letter['w']
        max_dy = PAIR_MAX_DY_MULT * letter['h']
        best, best_cost = None, None
        for i, arrow in enumerate(arrows):
            if i in used:
                continue
            dx = abs(arrow['cx'] - letter['cx'])
            dy = arrow['cy'] - letter['cy']
            if dy <= 0 or dy > max_dy or dx > max_dx:
                continue
            cost = dx * 2.0 + dy
            if best_cost is None or cost < best_cost:
                best, best_cost = i, cost
        if best is not None:
            used.add(best)
            table[letter['name']] = arrows[best]['name']
    return table


def to_det(name, x1, y1, x2, y2, score=1.0):
    return {'name': name, 'score': score,
            'cx': (x1 + x2) / 2.0, 'cy': (y1 + y2) / 2.0,
            'w': x2 - x1, 'h': y2 - y1,
            'box': (x1, y1, x2, y2)}


def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


# =============================================================================

def load_model(path, device):
    ckpt = torch.load(path, map_location='cpu')
    names = ckpt['class_names']
    model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn(
        weights=None,
        min_size=ckpt.get('min_size', 512),
        max_size=ckpt.get('max_size', 512),
        num_classes=ckpt['num_classes'])
    anchor_sizes = tuple(tuple(s) for s in
                         ckpt.get('anchor_sizes', ((8, 16, 32, 64, 128),) * 3))
    model.rpn.anchor_generator = AnchorGenerator(
        anchor_sizes, ((0.5, 1.0, 2.0),) * len(anchor_sizes))
    model.load_state_dict(ckpt['state_dict'])
    model.eval().to(device)
    return model, names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True)
    ap.add_argument('--split', default='test')
    ap.add_argument('--model', default='sign_detector.pt')
    ap.add_argument('--score', type=float, default=0.55)
    ap.add_argument('--limit', type=int, default=0, help='0 = all images')
    ap.add_argument('--save-vis', default=None)
    args = ap.parse_args()

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model, class_names = load_model(args.model, device)
    print(f"device {device} | classes {class_names} | score>={args.score}\n")

    split_dir = os.path.join(args.data, args.split)
    data = json.load(open(os.path.join(split_dir, '_annotations.coco.json')))

    cats = [c for c in data['categories']
            if c['name'].lower() not in ('aim-2026', 'none')]
    cats.sort(key=lambda c: c['id'])
    catid_to_name = {c['id']: c['name'] for c in cats}

    gt_by_img = defaultdict(list)
    for a in data['annotations']:
        if a['category_id'] in catid_to_name:
            gt_by_img[a['image_id']].append(a)

    images = data['images']
    if args.limit:
        images = images[:args.limit]

    if args.save_vis:
        os.makedirs(args.save_vis, exist_ok=True)

    tp = defaultdict(int)
    fp = defaultdict(int)
    fn = defaultdict(int)

    route_exact = 0        # predicted table identical to GT table
    route_partial = 0      # every predicted pair correct, but some missing
    route_wrong = 0        # at least one pair contradicts GT
    route_total = 0
    pair_tp = pair_fp = pair_fn = 0

    for n, info in enumerate(images, 1):
        img = Image.open(os.path.join(split_dir, info['file_name'])).convert('RGB')
        tensor = TF.to_tensor(img).to(device)
        with torch.no_grad():
            out = model([tensor])[0]

        preds = []
        for box, label, score in zip(out['boxes'].cpu().numpy(),
                                     out['labels'].cpu().numpy(),
                                     out['scores'].cpu().numpy()):
            if score < args.score:
                continue
            idx = int(label) - 1
            if 0 <= idx < len(class_names):
                x1, y1, x2, y2 = box
                preds.append(to_det(class_names[idx], x1, y1, x2, y2, float(score)))

        gts = []
        for a in gt_by_img.get(info['id'], []):
            x, y, w, h = a['bbox']
            gts.append(to_det(catid_to_name[a['category_id']],
                              x, y, x + w, y + h))

        # --- greedy IoU matching, per class ---
        matched = set()
        for p in sorted(preds, key=lambda d: -d['score']):
            best, best_iou = None, 0.0
            for j, g in enumerate(gts):
                if j in matched or g['name'] != p['name']:
                    continue
                v = iou(p['box'], g['box'])
                if v > best_iou:
                    best, best_iou = j, v
            if best is not None and best_iou >= 0.5:
                matched.add(best)
                tp[p['name']] += 1
            else:
                fp[p['name']] += 1
        for j, g in enumerate(gts):
            if j not in matched:
                fn[g['name']] += 1

        # --- routing accuracy (the metric that matters) ---
        gt_table = pair_letters_with_arrows(gts)
        pr_table = pair_letters_with_arrows(preds)
        if gt_table:
            route_total += 1
            wrong = any(pr_table.get(k) not in (None, v)
                        for k, v in gt_table.items())
            wrong = wrong or any(k not in gt_table for k in pr_table)
            if wrong:
                route_wrong += 1
            elif pr_table == gt_table:
                route_exact += 1
            else:
                route_partial += 1

            for k, v in gt_table.items():
                if pr_table.get(k) == v:
                    pair_tp += 1
                elif k in pr_table:
                    pair_fp += 1
                    pair_fn += 1
                else:
                    pair_fn += 1
            for k in pr_table:
                if k not in gt_table:
                    pair_fp += 1

        if args.save_vis and n <= 40:
            vis = img.copy()
            d = ImageDraw.Draw(vis)
            for g in gts:
                d.rectangle(g['box'], outline=(0, 200, 0), width=1)
            for p in preds:
                d.rectangle(p['box'], outline=(255, 60, 60), width=1)
                d.text((p['box'][0], max(0, p['box'][1] - 10)),
                       f"{p['name']} {p['score']:.2f}", fill=(255, 60, 60))
            vis.save(os.path.join(args.save_vis, f"{n:03d}_{info['file_name']}"))

        if n % 100 == 0:
            print(f"  ...{n}/{len(images)}", flush=True)

    # =========================================================================
    print("\n=== per-class detection @ IoU 0.5 ===")
    print(f"{'class':<10} {'TP':>5} {'FP':>5} {'FN':>5} {'prec':>7} {'rec':>7}")
    for name in class_names:
        t, f_, m = tp[name], fp[name], fn[name]
        prec = t / (t + f_) if (t + f_) else 0.0
        rec = t / (t + m) if (t + m) else 0.0
        print(f"{name:<10} {t:>5} {f_:>5} {m:>5} {prec:>7.3f} {rec:>7.3f}")

    tt, tf, tm = sum(tp.values()), sum(fp.values()), sum(fn.values())
    print(f"{'ALL':<10} {tt:>5} {tf:>5} {tm:>5} "
          f"{tt / (tt + tf) if tt + tf else 0:>7.3f} "
          f"{tt / (tt + tm) if tt + tm else 0:>7.3f}")

    print("\n=== routing accuracy (what the mission depends on) ===")
    if route_total:
        print(f"images with a GT routing table : {route_total}")
        print(f"  exact match                  : {route_exact} "
              f"({100 * route_exact / route_total:.1f}%)")
        print(f"  partial (correct but missing): {route_partial} "
              f"({100 * route_partial / route_total:.1f}%)")
        print(f"  WRONG (contradicts GT)       : {route_wrong} "
              f"({100 * route_wrong / route_total:.1f}%)")
        p = pair_tp / (pair_tp + pair_fp) if (pair_tp + pair_fp) else 0.0
        r = pair_tp / (pair_tp + pair_fn) if (pair_tp + pair_fn) else 0.0
        print(f"\npair-level precision {p:.3f}   recall {r:.3f}")
        print("\nNOTE: a WRONG pair sends the buggy down the wrong road, which "
              "is far\ncostlier than a MISSING pair (which just means we keep "
              "exploring).\nOptimise the score threshold for precision, not "
              "recall.")
    else:
        print("no ground-truth routing tables found in this split")

    if args.save_vis:
        print(f"\nannotated samples written to {args.save_vis}/ "
              f"(green = GT, red = prediction)")


if __name__ == '__main__':
    main()