import json
import argparse
import math
import os
import itertools
import pandas as pd


def analyize_averages(filename, output_format="csv"):
    analysis = {}

    with open(filename, "r") as file:
        logs = json.load(file)

    for log in logs:
        algo = log["search_info"]["name"]
        eps = log["search_info"]["eps"][0]

        if eps not in analysis:
            analysis[eps] = {}

        if algo not in analysis[eps]:
            analysis[eps][algo] = {
                "queries_count": 0,
                "total_solutions": 0,
                "total_runtime": 0,
                "min_runtime": math.inf,
                "max_runtime": 0,
            }

        amount_of_solutions = log["finish_info"]["amount_of_solutions"]
        if algo == "PPA": # PPA logs both solutions in each path pair
            amount_of_solutions /= 2

        analysis[eps][algo]["total_solutions"] += amount_of_solutions
        analysis[eps][algo]["total_runtime"] += log["total_runtime(ms)"]
        analysis[eps][algo]["min_runtime"] = min(analysis[eps][algo]["min_runtime"], log["total_runtime(ms)"])
        analysis[eps][algo]["max_runtime"] = max(analysis[eps][algo]["max_runtime"], log["total_runtime(ms)"])
        analysis[eps][algo]["queries_count"] += 1


    for eps in analysis:
        for algo in analysis[eps]:
            analysis[eps][algo]["avg_solutions"] = \
                analysis[eps][algo]["total_solutions"]/analysis[eps][algo]["queries_count"]
            analysis[eps][algo]["avg_runtime"] = \
                analysis[eps][algo]["total_runtime"]/analysis[eps][algo]["queries_count"]

        analysis[eps]["avg_speedup"] = calc_avg_speedup(analysis[eps])


    if output_format == "json":
        analysis_json_filename = f"{os.path.splitext(filename)[0]}_analysis.json"

        with open(analysis_json_filename, "w") as file:
            file.write(json.dumps(analysis, indent=4))

    elif output_format == "csv":
        analysis_csv_filename = f"{os.path.splitext(filename)[0]}_analysis.csv"

        analysis_df = dict_to_df(analysis)
        analysis_df.to_csv(analysis_csv_filename, index=True)


def calc_avg_speedup(algo_dicts):
    avg_speedup = {}
    pairs = itertools.combinations(algo_dicts.keys(), 2)
    for pair in pairs:
        avg_speedup[f"{pair[0]}_{pair[1]}"] = algo_dicts[pair[0]]["avg_runtime"]/algo_dicts[pair[1]]["avg_runtime"]

    return avg_speedup


def dict_to_df(analysis_dict):
    column_set = set()
    for row in analysis_dict:
        for column_group in analysis_dict[row]:
            column_keys = [f"{column_group}_{column}" for column in analysis_dict[row][column_group].keys()]
            column_set.update(column_keys)

    columns = ["eps"] + sorted(column_set)

    analysis_df = pd.DataFrame(columns=columns)
    analysis_df["eps"] = analysis_dict.keys()
    analysis_df = analysis_df.set_index(["eps"])

    for row in analysis_dict:
        for column_group in analysis_dict[row]:
            for column in analysis_dict[row][column_group]:
                df_column = f"{column_group}_{column}"
                analysis_df[df_column][row] = analysis_dict[row][column_group][column]

    return analysis_df



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze log files of the search algorithms")
    parser.add_argument("log_filename")
    parser.add_argument("output_format", default="csv")

    args = parser.parse_args()
    analyize_averages(args.log_filename, args.output_format)
