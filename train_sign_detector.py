#!/usr/bin/env python3
# =============================================================================
# train_sign_detector.py
# -----------------------------------------------------------------------------
# NXP Cup India 2026 - sign board detector training. Run OFFLINE; only the
# resulting .pt weights ship inside b3rb_ros_line_follower/.
#
# WHY torchvision AND NOT YOLO
#   The approved module list allows torch 2.3.0 / torchvision 0.18.0 but NOT
#   ultralytics, so every YOLO export in the dataset zip is unusable without
#   written consent. The COCO export is read natively by torchvision, so this
#   route stays inside the rules with zero extra dependencies.
#
# DATASET SHAPE (from the provided COCO export)
#   classes : AIM-2026 (Roboflow placeholder, id 0 -> background), then
#             A B C          = patient buildings 1/2/3
#             X Y Z          = hospitals 1/2/3
#             Left Right Straight = direction arrows
#   1896 train images, 18367 annotations (~9.7 boxes/image)
#   images are 512x512, letterboxed ("Fit (black edges)")
#
#   Letters and arrows are SEPARATE detections. Pairing them into a routing
#   instruction ("A is to the Left") is done at inference time by geometry,
#   not by the model.
#
# MODEL CHOICE
#   fasterrcnn_mobilenet_v3_large_320_fpn by default: markedly better than
#   SSDLite on small objects (sign glyphs are small in frame) while still
#   being a 320-resolution mobile backbone, so CPU inference stays viable.
#   The evaluation machine may have no GPU, so weights are saved CPU-loadable.
#
# USAGE
#   python3 train_sign_detector.py --data /path/to/NXPCUP_2026.v2-v1_a.coco
#   python3 train_sign_detector.py --data ... --epochs 25 --batch 4
#
# OUTPUT
#   sign_detector.pt   - state_dict + class names + input size, CPU-loadable
# =============================================================================

import argparse
import json
import os
import time

import torch
import torchvision
from PIL import Image
from torch.utils.data import Dataset, DataLoader
from torchvision.models.detection.anchor_utils import AnchorGenerator
from torchvision.transforms import functional as TF


# =============================================================================
# DATASET
# =============================================================================

class CocoSignDataset(Dataset):
    """
    Minimal COCO reader for torchvision detection models.

    torchvision expects, per image:
        boxes  FloatTensor[N, 4] in absolute xyxy
        labels Int64Tensor[N]     with 0 reserved for background

    Roboflow writes bboxes as [x, y, w, h], and its category ids start at 0
    with a dataset-name placeholder. We remap category ids onto a contiguous
    1..K range so 0 stays free for background.
    """

    def __init__(self, root, split, train=True):
        self.dir = os.path.join(root, split)
        ann_path = os.path.join(self.dir, '_annotations.coco.json')
        with open(ann_path) as f:
            data = json.load(f)

        # --- class remap -------------------------------------------------
        # Drop the Roboflow placeholder (supercategory == 'none' or the
        # dataset name); keep everything else in a stable, sorted order.
        cats = [c for c in data['categories']
                if c['name'].lower() not in ('aim-2026', 'none')]
        cats.sort(key=lambda c: c['id'])
        self.class_names = [c['name'] for c in cats]
        self.cat_to_label = {c['id']: i + 1 for i, c in enumerate(cats)}

        self.images = {im['id']: im for im in data['images']}
        self.by_image = {im_id: [] for im_id in self.images}
        for ann in data['annotations']:
            if ann['category_id'] not in self.cat_to_label:
                continue
            self.by_image.setdefault(ann['image_id'], []).append(ann)

        self.ids = sorted(self.images.keys())
        self.train = train

    def __len__(self):
        return len(self.ids)

    def __getitem__(self, idx):
        img_id = self.ids[idx]
        info = self.images[img_id]
        img = Image.open(os.path.join(self.dir, info['file_name'])).convert('RGB')

        boxes, labels = [], []
        for ann in self.by_image.get(img_id, []):
            x, y, w, h = ann['bbox']
            if w <= 1 or h <= 1:
                continue                       # degenerate box, skip
            boxes.append([x, y, x + w, y + h])
            labels.append(self.cat_to_label[ann['category_id']])

        if boxes:
            boxes_t = torch.as_tensor(boxes, dtype=torch.float32)
            labels_t = torch.as_tensor(labels, dtype=torch.int64)
        else:
            # Negative sample: valid, and useful for suppressing false positives.
            boxes_t = torch.zeros((0, 4), dtype=torch.float32)
            labels_t = torch.zeros((0,), dtype=torch.int64)

        img_t = TF.to_tensor(img)

        # Horizontal flip would swap Left/Right semantics, so the only safe
        # train-time augmentation here is photometric. Roboflow already applied
        # geometric augmentation when exporting, so we keep this light.
        if self.train and torch.rand(1).item() < 0.3:
            img_t = TF.adjust_brightness(img_t, 0.7 + 0.6 * torch.rand(1).item())

        target = {'boxes': boxes_t, 'labels': labels_t,
                  'image_id': torch.tensor([img_id])}
        return img_t, target


def collate(batch):
    return tuple(zip(*batch))


# =============================================================================
# TRAIN
# =============================================================================

def build_model(num_classes):
    """
    Faster R-CNN MobileNetV3-320-FPN, adapted for very small objects.

    Two defaults would otherwise destroy this dataset:

    1. min_size/max_size default to 320/640, so a 512x512 input gets scaled
       DOWN and the median 18x15 px glyph shrinks to ~11 px. We pin both to
       512 to keep native resolution.

    2. The default anchor sizes are (32, 64, 128, 256, 512) - the smallest
       anchor is already larger than most objects here, so the RPN proposes
       almost nothing useful. We shift the ladder down to
       (8, 16, 32, 64, 128). The anchor COUNT per location is unchanged
       (5 sizes x 3 ratios), so the pretrained RPN head still fits and does
       not need rebuilding.
    """
    model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn(
        weights='DEFAULT',
        min_size=512,
        max_size=512,
    )

    # --- retarget the anchor ladder at small objects ---
    anchor_sizes = ((8, 16, 32, 64, 128),) * 3
    aspect_ratios = ((0.5, 1.0, 2.0),) * len(anchor_sizes)
    model.rpn.anchor_generator = AnchorGenerator(anchor_sizes, aspect_ratios)

    # --- swap the classifier head for our class count ---
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = \
        torchvision.models.detection.faster_rcnn.FastRCNNPredictor(
            in_features, num_classes)
    return model


def warmup_scheduler(optimizer, warmup_iters, start_factor=0.01):
    """
    Linear LR warmup over the first `warmup_iters` optimiser steps.

    This matters here specifically. We replaced the anchor ladder (necessary:
    an 18 px glyph against the stock 32 px minimum anchor gives IoU ~0.26, so
    the RPN would see almost no positive samples) AND swapped in a fresh box
    predictor. The pretrained RPN regression head is therefore mismatched to
    the new anchor semantics. Hitting it with the full learning rate from step
    one kicks the model out of its pretrained basin faster than it can adapt,
    which shows up as loss climbing steadily instead of falling.
    """
    def f(step):
        if step >= warmup_iters:
            return 1.0
        alpha = step / float(max(warmup_iters, 1))
        return start_factor * (1.0 - alpha) + alpha
    return torch.optim.lr_scheduler.LambdaLR(optimizer, f)


def build_optimizer(params, args):
    """AdamW by default: far more forgiving when detection heads are replaced."""
    if args.opt == 'sgd':
        return torch.optim.SGD(params, lr=args.lr, momentum=0.9,
                               weight_decay=5e-4)
    return torch.optim.AdamW(params, lr=args.lr, weight_decay=0.01)


@torch.no_grad()
def evaluate_loss(model, loader, device):
    """
    Mean training-style loss over the validation split.

    torchvision detection models only return losses in train() mode, so we
    temporarily switch modes while keeping gradients off. This is a proxy for
    quality, not a real mAP - good enough to pick the best epoch.
    """
    model.train()
    total, batches = 0.0, 0
    for images, targets in loader:
        images = [i.to(device) for i in images]
        targets = [{k: v.to(device) for k, v in t.items()} for t in targets]
        losses = model(images, targets)
        total += sum(l.item() for l in losses.values())
        batches += 1
    return total / max(batches, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True,
                    help='path to NXPCUP_2026.v2-v1_a.coco')
    ap.add_argument('--epochs', type=int, default=20)
    ap.add_argument('--batch', type=int, default=4)
    ap.add_argument('--opt', choices=['adamw', 'sgd'], default='adamw')
    ap.add_argument('--lr', type=float, default=None,
                    help='default: 1e-4 for adamw, 0.002 for sgd')
    ap.add_argument('--warmup', type=int, default=500,
                    help='optimiser steps of linear LR warmup')
    ap.add_argument('--out', default='sign_detector.pt')
    ap.add_argument('--workers', type=int, default=4)
    args = ap.parse_args()

    if args.lr is None:
        args.lr = 1e-4 if args.opt == 'adamw' else 0.002

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"device: {device}   optimiser: {args.opt}   lr: {args.lr}")

    train_ds = CocoSignDataset(args.data, 'train', train=True)
    val_ds = CocoSignDataset(args.data, 'valid', train=False)
    print(f"classes ({len(train_ds.class_names)}): {train_ds.class_names}")
    print(f"train images: {len(train_ds)}   val images: {len(val_ds)}")

    train_ld = DataLoader(train_ds, batch_size=args.batch, shuffle=True,
                          num_workers=args.workers, collate_fn=collate)
    val_ld = DataLoader(val_ds, batch_size=args.batch, shuffle=False,
                        num_workers=args.workers, collate_fn=collate)

    num_classes = len(train_ds.class_names) + 1        # + background
    model = build_model(num_classes).to(device)

    params = [p for p in model.parameters() if p.requires_grad]
    opt = build_optimizer(params, args)
    warm = warmup_scheduler(opt, args.warmup)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    global_step = 0

    best = float('inf')
    for epoch in range(1, args.epochs + 1):
        model.train()
        t0, running = time.time(), 0.0

        for step, (images, targets) in enumerate(train_ld, 1):
            images = [i.to(device) for i in images]
            targets = [{k: v.to(device) for k, v in t.items()} for t in targets]

            losses = model(images, targets)
            loss = sum(losses.values())

            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(params, 5.0)
            opt.step()

            # Warmup advances per optimiser step; the cosine schedule advances
            # per epoch, and only takes over once warmup has finished.
            global_step += 1
            if global_step <= args.warmup:
                warm.step()

            running += loss.item()
            if step % 50 == 0:
                lr_now = opt.param_groups[0]['lr']
                print(f"  epoch {epoch} step {step}/{len(train_ld)} "
                      f"loss {running / step:.4f} lr {lr_now:.2e}", flush=True)

        if global_step > args.warmup:
            sched.step()
        val_loss = evaluate_loss(model, val_ld, device)
        print(f"epoch {epoch}: train {running / len(train_ld):.4f}  "
              f"val {val_loss:.4f}  ({time.time() - t0:.0f}s)")

        if val_loss < best:
            best = val_loss
            # Save on CPU so the checkpoint loads on a GPU-less evaluation box.
            torch.save({
                'state_dict': {k: v.cpu() for k, v in model.state_dict().items()},
                'class_names': train_ds.class_names,
                'num_classes': num_classes,
                'arch': 'fasterrcnn_mobilenet_v3_large_320_fpn',
                'input_letterbox': 512,
                'min_size': 512,
                'max_size': 512,
                'anchor_sizes': ((8, 16, 32, 64, 128),) * 3,
            }, args.out)
            print(f"  -> saved {args.out} (val {best:.4f})")

    print(f"\ndone. best val loss {best:.4f}, weights in {args.out}")
    print("Copy the .pt into b3rb_ros_line_follower/ so it ships with the "
          "submission.")


if __name__ == '__main__':
    main()