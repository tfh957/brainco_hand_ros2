#!/usr/bin/env python3
"""
Run on Windows PC (or any PC) to send line-delimited JSON frames via TCP.
"""
import argparse
import json
import socket
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", required=True)
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--fps", type=float, default=10.0)
    args = p.parse_args()

    period = 1.0 / max(0.1, args.fps)

    with socket.create_connection((args.host, args.port), timeout=5.0) as s:
        print(f"connected -> {args.host}:{args.port}")
        while True:
            frame = {
                "hand_positions": [1.0, 1.2, 1.2, 1.2, 1.2, 1.2],
                "hand_sec": 0.15,
            }
            payload = (json.dumps(frame, ensure_ascii=False) + "\n").encode("utf-8")
            s.sendall(payload)
            time.sleep(period)


if __name__ == "__main__":
    main()
