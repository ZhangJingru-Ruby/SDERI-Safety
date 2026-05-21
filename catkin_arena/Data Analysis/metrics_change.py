#!/usr/bin/env python3
import argparse
import os
import pandas as pd

def process_metrics_file(file_path, keep_rows=100, require_rows=False):
    try:
        # 读取CSV
        df = pd.read_csv(file_path)
        original_rows = len(df)

        if require_rows and original_rows < keep_rows:
            print(f"跳过: {file_path} (只有{original_rows}行，少于{keep_rows}行)")
            return

        # 只保留前100行
        df = df.head(keep_rows)

        # 覆盖保存
        df.to_csv(file_path, index=False)
        print(f"处理完成: {file_path} ({original_rows}行 -> 保留前{len(df)}行)")
    except Exception as e:
        print(f"处理失败: {file_path}, 错误: {e}")

def find_and_process_metrics(root_dir=".", keep_rows=100, require_rows=False):
    for dirpath, _, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename == "metrics.csv":
                file_path = os.path.join(dirpath, filename)
                process_metrics_file(file_path, keep_rows=keep_rows, require_rows=require_rows)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="批量裁剪 metrics.csv 行数，默认保留前100行。")
    parser.add_argument("--root", "-r", default=".", help="递归查找目录，默认当前目录")
    parser.add_argument("--keep-rows", "-n", type=int, default=100, help="保留行数，默认100")
    parser.add_argument(
        "--require-rows",
        action="store_true",
        help="如果 metrics.csv 少于保留行数则跳过，避免把不足100个episode的数据当作100个统计。",
    )
    args = parser.parse_args()

    find_and_process_metrics(args.root, keep_rows=args.keep_rows, require_rows=args.require_rows)
