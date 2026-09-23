#!/usr/bin/env python3
"""
sender.py — AVTP Video Sender
==============================
Reads PNG image frames from a directory, encapsulates each frame into one or
more AVTP CVF packets (with proper fragmentation for frames larger than the
Ethernet MTU), and transmits them over the specified network interface.

This script must be run inside the 'sender' network namespace with root
privileges (raw socket access is required by Scapy):

    sudo ip netns exec sender python3 sender.py [options]

The virtual network must be set up first:

    sudo bash setup.sh

Options
-------
    --interface     Network interface to use           (default: veth-s)
    --fps           Target transmission frame rate      (default: 30)
    --images-dir    Directory containing PNG frames     (default: _out/camera)
    --stream-id     64-bit AVTP stream ID (hex)        (default: 0xAABBCCDDEEFF0001)
    --loop          Loop through frames indefinitely    (default: off)
"""

import argparse
import os
import sys
import time
from pathlib import Path

# Scapy may print a warning about IPv6 — suppress it for cleaner output
import logging
logging.getLogger("scapy.runtime").setLevel(logging.ERROR)

from scapy.all import conf, get_if_hwaddr, sendp

import avtp as avtp_lib


# ── Argument parsing ──────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="AVTP CVF video sender — transmits PNG frames over a "
                    "virtual L2 network using the AVTP protocol."
    )
    parser.add_argument(
        "--interface", "-i",
        default="veth-s",
        help="Network interface to send on (default: veth-s)",
    )
    parser.add_argument(
        "--fps", "-f",
        type=float,
        default=30.0,
        help="Target frames per second (default: 30)",
    )
    parser.add_argument(
        "--images-dir", "-d",
        default="_out/camera",
        help="Directory containing PNG frame files (default: _out/camera)",
    )
    parser.add_argument(
        "--stream-id", "-s",
        default="0xAABBCCDDEEFF0001",
        help="64-bit AVTP stream ID in hex (default: 0xAABBCCDDEEFF0001)",
    )
    parser.add_argument(
        "--loop", "-l",
        action="store_true",
        help="Loop through all frames indefinitely until Ctrl-C",
    )
    return parser.parse_args()


# ── Frame discovery ───────────────────────────────────────────────────────────

def discover_frames(images_dir: str) -> list[Path]:
    """Return a sorted list of JPG files found in *images_dir*."""
    p = Path(images_dir)
    if not p.is_dir():
        print(f"[!] Images directory not found: {images_dir}", file=sys.stderr)
        sys.exit(1)

    frames = sorted(p.glob("*.jpg"))
    if not frames:
        print(f"[!] No JPG files found in: {images_dir}", file=sys.stderr)
        sys.exit(1)

    return frames


# ── Main sender loop ──────────────────────────────────────────────────────────

def run(args):
    stream_id = int(args.stream_id, 16)
    frame_period = 1.0 / args.fps  # seconds between frames

    frames = discover_frames(args.images_dir)
    total_frames = len(frames)

    # Retrieve the MAC address of the sending interface
    try:
        src_mac = get_if_hwaddr(args.interface)
    except Exception as exc:
        print(f"[!] Cannot read MAC from interface '{args.interface}': {exc}",
              file=sys.stderr)
        sys.exit(1)

    conf.iface = args.interface

    print("=" * 65)
    print("  AVTP Video Sender")
    print("=" * 65)
    print(f"  Interface : {args.interface}  (MAC {src_mac})")
    print(f"  Stream ID : {args.stream_id}")
    print(f"  FPS       : {args.fps}")
    print(f"  Frames    : {total_frames}  ({args.images_dir})")
    print(f"  Loop      : {'yes' if args.loop else 'no'}")
    print("=" * 65)
    print()

    seq_counter = 0       # AVTP sequence_num (wraps at 255, per stream)
    frames_sent = 0
    bytes_sent = 0
    pkts_sent = 0
    start_time = time.time()

    try:
        iteration = 0
        while True:
            iteration += 1
            if args.loop:
                print(f"[~] Loop iteration {iteration}")

            for frame_idx, frame_path in enumerate(frames, start=1):
                frame_start = time.time()

                image_bytes = frame_path.read_bytes()
                frame_num = (frames_sent) & 0xFFFF  # wraps at 65535

                packets = avtp_lib.fragment_image(
                    image_bytes=image_bytes,
                    stream_id=stream_id,
                    frame_num=frame_num,
                    seq_counter=seq_counter,
                    src_mac=src_mac,
                )

                sendp(packets, iface=args.interface, verbose=0)

                seq_counter = (seq_counter + len(packets)) & 0xFF
                frames_sent += 1
                bytes_sent += len(image_bytes)
                pkts_sent += len(packets)

                elapsed = time.time() - start_time
                print(
                    f"  [TX] Frame {frame_idx:>4}/{total_frames}"
                    f"  {len(image_bytes):>7} B"
                    f"  {len(packets):>3} pkts"
                    f"  seq {frame_num:>5}"
                    f"  elapsed {elapsed:>7.1f}s"
                )

                # Rate limiting: sleep for the remaining time in this frame period
                elapsed_frame = time.time() - frame_start
                sleep_for = frame_period - elapsed_frame
                if sleep_for > 0:
                    time.sleep(sleep_for)

            if not args.loop:
                break

    except KeyboardInterrupt:
        print("\n[!] Interrupted by user.")

    # ── Summary ───────────────────────────────────────────────────────────────
    total_elapsed = time.time() - start_time
    actual_fps = frames_sent / total_elapsed if total_elapsed > 0 else 0
    print()
    print("=" * 65)
    print("  Transmission complete")
    print("=" * 65)
    print(f"  Frames sent  : {frames_sent}")
    print(f"  Packets sent : {pkts_sent}")
    print(f"  Bytes sent   : {bytes_sent:,}")
    print(f"  Elapsed time : {total_elapsed:.2f}s")
    print(f"  Actual FPS   : {actual_fps:.1f}")
    print("=" * 65)


if __name__ == "__main__":
    args = parse_args()
    run(args)
