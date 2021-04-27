#!/usr/bin/env python3


import argparse
import pandas as pd
import pathlib
import os


def dir_path(string):
    if os.path.isdir(string):
        return string
    else:
        raise NotADirectoryError(string)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("src_path", type=dir_path)
    parser.add_argument("out_file")
    parser.add_argument("--raw", dest="raw", action="store_true")
    args = parser.parse_args()

    # collect data from all runs
    data_frames = []
    for path in pathlib.Path(args.src_path).rglob("results.csv"):
        data_frames.append(pd.read_csv(path.as_posix()))

    # determine min, max and median
    if not args.raw:
        reduced_data_frames = []
        for df in data_frames:
            reduced_df = df.drop(columns=["time (ms)", "iteration"]).drop_duplicates()
            reduced_df["min time (ms)"] = df["time (ms)"].min()
            reduced_df["max time (ms)"] = df["time (ms)"].max()
            reduced_df["median time (ms)"] = df["time (ms)"].median()
            reduced_df["mean time (ms)"] = df["time (ms)"].mean()
            reduced_data_frames.append(reduced_df)
        data_frames = reduced_data_frames

    # concatenate the data
    concat = pd.concat(data_frames, ignore_index=True)

    # write the concatenated results
    concat.to_csv(args.out_file)


if __name__ == "__main__":
    main()
