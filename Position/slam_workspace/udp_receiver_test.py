#!/usr/bin/env python3
"""
UDP 定位数据接收测试工具
用法: python3 udp_receiver_test.py [port]
默认监听 9005 端口，统计实际接收频率、抖动、位置数据。
"""

import socket
import json
import time
import sys
import statistics

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 9005
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)
    
    print(f"=== UDP 定位数据接收测试 ===")
    print(f"监听端口: {port}")
    print(f"等待数据中...\n")
    
    # 统计变量
    count = 0
    start_time = None
    last_time = None
    intervals = []  # 帧间隔 (ms)
    positions = []  # 最近100帧位置
    WINDOW = 100    # 滑动窗口大小
    REPORT_INTERVAL = 500  # 每500帧打印一次详细报告
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                now = time.perf_counter()
                
                if start_time is None:
                    start_time = now
                    last_time = now
                    print(f"收到来自 {addr[0]}:{addr[1]} 的数据")
                    print(f"数据格式: {data.decode('utf-8', errors='replace')[:120]}")
                    print(f"\n{'='*60}")
                    print(f"{'帧数':>8} | {'频率Hz':>8} | {'平均间隔ms':>10} | {'抖动ms':>8} | 位置 (X, Y, Yaw)")
                    print(f"{'-'*60}")
                    count = 1
                    continue
                
                interval_ms = (now - last_time) * 1000.0
                intervals.append(interval_ms)
                if len(intervals) > WINDOW:
                    intervals.pop(0)
                last_time = now
                count += 1
                
                # 解析位置
                try:
                    pkt = json.loads(data.decode('utf-8'))
                    pos = pkt.get('pos', [0, 0, 0])
                    euler = pkt.get('euler', [0, 0, 0])
                    # 发送端: pos=[-y, 0, x], euler=[0, -yaw, 0]
                    x = pos[2]
                    y = -pos[0]
                    yaw = -euler[1]
                    positions.append((x, y, yaw))
                    if len(positions) > WINDOW:
                        positions.pop(0)
                except:
                    x, y, yaw = 0, 0, 0
                
                # 每 REPORT_INTERVAL 帧打印统计
                if count % REPORT_INTERVAL == 0:
                    elapsed = now - start_time
                    avg_hz = (count - 1) / elapsed if elapsed > 0 else 0
                    avg_interval = statistics.mean(intervals)
                    
                    if len(intervals) >= 2:
                        jitter = statistics.stdev(intervals)
                    else:
                        jitter = 0
                    
                    # 位置变化率 (mm/frame)
                    if len(positions) >= 2:
                        dx = (positions[-1][0] - positions[-2][0]) * 1000
                        dy = (positions[-1][1] - positions[-2][1]) * 1000
                    else:
                        dx, dy = 0, 0
                    
                    print(f"{count:>8} | {avg_hz:>8.1f} | {avg_interval:>10.2f} | {jitter:>8.2f} | "
                          f"X={x:.3f} Y={y:.3f} Yaw={yaw:.1f}° Δ={dx:.1f},{dy:.1f}mm")
                
            except socket.timeout:
                if count == 0:
                    print(".", end="", flush=True)
                continue
                
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        if count > 1 and start_time:
            elapsed = time.perf_counter() - start_time
            avg_hz = (count - 1) / elapsed
            print(f"\n{'='*60}")
            print(f"=== 最终统计 ===")
            print(f"总帧数: {count}")
            print(f"总时间: {elapsed:.2f} 秒")
            print(f"平均频率: {avg_hz:.1f} Hz")
            if len(intervals) >= 2:
                print(f"平均间隔: {statistics.mean(intervals):.3f} ms")
                print(f"间隔标准差(抖动): {statistics.stdev(intervals):.3f} ms")
                print(f"最小间隔: {min(intervals):.3f} ms")
                print(f"最大间隔: {max(intervals):.3f} ms")
            
            # 位置变化统计
            if len(positions) >= 2:
                total_dist = 0
                for i in range(1, len(positions)):
                    dx = positions[i][0] - positions[i-1][0]
                    dy = positions[i][1] - positions[i-1][1]
                    total_dist += (dx**2 + dy**2)**0.5
                speed = total_dist / elapsed if elapsed > 0 else 0
                print(f"平均速度: {speed:.3f} m/s")
            
            # 生成 Hz 分布直方图
            if intervals:
                bins = [0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 10.0, 100.0]
                print(f"\n帧间隔分布 (ms):")
                for i in range(len(bins)-1):
                    count_in_bin = sum(1 for iv in intervals if bins[i] <= iv < bins[i+1])
                    pct = count_in_bin / len(intervals) * 100
                    bar = '#' * int(pct / 2)
                    print(f"  [{bins[i]:>5.1f}, {bins[i+1]:>5.1f}): {count_in_bin:>5} ({pct:>5.1f}%) {bar}")

if __name__ == "__main__":
    main()
