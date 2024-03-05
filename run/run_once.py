import subprocess
import sys
import argparse
import os
from write import write_average_result
from datetime import datetime

turn_cost = 5
unique_file = "unique2.cost"


def run_once(map, metric, algorithm, eps, agent_num, eps_ratio=0, CB=True, eager=True, solution_num=sys.maxsize):
    if(algorithm == "BBMOCBS-K"):
        return
    if eps_ratio == 0.8:
        eps_ratio = 1
    if eps_ratio == 0:
        return
    if CB == True and eager == False:
        return
    root_directory = "/home/hacken/SummerResearch/BB/"
    data_directory = os.path.join(root_directory, "dataset", map)
    scenario_directory = os.path.join(data_directory, "scenario")
    cost_directory = os.path.join(data_directory, "cost")
    map_path = os.path.join(data_directory, map + ".map")
    exe_path = os.path.relpath(os.path.join(root_directory, "build/bin/multiobj"), os.getcwd())

    dim = len(metric)

    if not os.path.exists(map_path):
        print(map + " does not exist!")
        exit(1)

    # output file
    time = "" # datetime.now().strftime("-%m%d-%H:%M:%S")
    output_directory = os.path.join(root_directory, str(dim)+"d.out", map, "n="+str(agent_num))
    
    if eager:
        if_eager = "-E"; if_eager_ = "true"
    else:
        if_eager = ""; if_eager_ = "false"
    
    if CB:
        if_CB = "-RCB"; if_CB_ = "true"
    else:
        if_CB = "-REV"; if_CB_ = "false"
    
    if algorithm == "BBMOCBS-eps":
        output_file = os.path.join(output_directory, "EPS-"+"{:.2f}-{}{}".format(eps, if_eager, metric, time))
        solution_num_command = ""
    elif algorithm == "BBMOCBS-pex":
        output_file = os.path.join(output_directory, "PEX-{:.2f}-{:.1f}{}{}-{}{}".format(eps, eps_ratio, if_CB, if_eager, metric, time))
        solution_num_command = ""
    elif algorithm == "BBMOCBS-k":
        output_file = os.path.join(output_directory, "K-{}{}{}-{}{}".format(solution_num, if_CB, if_eager, metric, time))
        solution_num_command = " -k " + str(solution_num)
        

    # if the last file is unfinished, terminate
    if algorithm != "BBMOCBS-k" and eps != 0.1:
        if eps == 0:
            eps_ = 0.03
        if eps == 0.03:
            eps_ = 0.05
        if eps == 0.05:
            eps_ = 0.1
        output_directory_ = os.path.join(root_directory, str(dim)+"d.out", map, "n="+str(agent_num))
        if algorithm == "BBMOCBS-eps":
            output_file_ = os.path.join(output_directory_, "EPS-"+"{:.2f}-{}{}".format(eps_, if_eager, metric, time))
        elif algorithm == "BBMOCBS-pex":
            output_file_ = os.path.join(output_directory_, "PEX-{:.2f}-{:.1f}{}{}-{}{}".format(eps_, eps_ratio, if_CB, if_eager, metric, time))

        if not os.path.exists(output_file_):
            return
        else:
            with open(output_file_, 'r') as output:
                if_last_over = False
                for line in output:
                    if "AVERAGE" in line:
                        if_last_over = True
                if not if_last_over:
                    return

    if agent_num > 4:
        output_directory_ = os.path.join(root_directory, str(dim)+"d.out", map, "n="+str(agent_num-4))
        if algorithm == "BBMOCBS-eps":
            output_file_ = os.path.join(output_directory_, "EPS-"+"{:.2f}-{}{}".format(eps, if_eager, metric, time))
        elif algorithm == "BBMOCBS-pex":
            output_file_ = os.path.join(output_directory_, "PEX-{:.2f}-{:.1f}{}{}-{}{}".format(eps, eps_ratio, if_CB, if_eager, metric, time))
        elif algorithm == "BBMOCBS-k":
            output_file_ = os.path.join(output_directory_, "K-"+str(solution_num)+if_CB+if_eager+"-"+metric+time)

        if not os.path.exists(output_file_):
            return
        else:
            with open(output_file_, 'r') as output:
                if_last_over = False
                for line in output:
                    if "AVERAGE" in line:
                        if_last_over = True
                if not if_last_over:
                    return


    if not os.path.exists(output_file):
        with open(output_file, "w") as output:
            time_ = datetime.now().strftime("%D %Y  --%H:%M:%S\n\n")
            output.write(
                time_ + "\n"
                + "Map: " + map + "\n" 
                + "Algorithm: " + algorithm + "\n"
                + "eps = {:.2f},  {:.1f}\n".format(eps, eps_ratio)
                + "Agent Number = " + str(agent_num) + "\n"
                + "Cost Metric = " + str(metric) + "\n"
                + "Turn Cost = " + str(turn_cost) + "\n"
            )
    
    scenarios = os.listdir(scenario_directory)

    for scene_num in range(1, 26):
        for filename in scenarios:
            if "-{}.scen".format(scene_num) in filename:
                scenario_path = os.path.join(scenario_directory, filename)
                break
        
        have_scenario = False
        have_searched = False
        with open(output_file, "r") as output:
            for line in output:
                if scenario_path in line:
                    have_scenario = True
                elif have_scenario == True and "HLMergingTime" in line:
                    have_searched = True
                    break

        if have_searched:
            continue

        cost_map = []
        turn_dim = -1
        for i in range(dim):
            if metric[i] == 'r':
                cost_file = os.path.join(cost_directory, "random-" + str(100*i+scene_num) + ".cost")
            elif metric[i] == 'd':
                cost_file = os.path.join(cost_directory, "distance.cost")
            elif metric[i] == 't':
                cost_file = os.path.join(cost_directory, unique_file)
                turn_dim = i

            if not os.path.exists(cost_file):
                print(cost_file + "does not exist!")
            else:
                cost_map.append(cost_file)
        
        if len(cost_map) != 0:
            cost_str = "Cost Map:"
            for i in range(len(cost_map)):
                cost_str += "    " + cost_map[i]
            cost_str += "\n"
        else:
            cost_str = ""

        if not have_scenario:
            with open(output_file, "a") as output:
                output.write(
                    "\n************************************************************************\n"
                    + "Scenario File:  " + scenario_path + "\n"
                    + cost_str
                    + "\n\n"
                )

        if_error = False
        if agent_num > 4:
            with open(output_file_, 'r') as output:
                if_find = False
                for line in output:
                    if scenario_path in line:
                        if_find = True
                        continue
                    if if_find:
                        if "SUCCESS" in line:
                            if_last_success = True
                            break
                        if "FAIL" in line:
                            if_last_success = False
                            break
            if not if_last_success:
                if_error = True

        if not if_error:
            command = "./{} -m {} -n {} -s {} -e {:.2f} -r {:.1f} -d {} -a {} --CB {} --eager {} --turn_dim {} --turn_cost {} -o {}".format(
                exe_path, map_path, agent_num, scenario_path, eps, eps_ratio, dim, algorithm, if_CB_, if_eager_, turn_dim, turn_cost, output_file) + solution_num_command 
            
            if dim == 2: 
                command += " --c1 " + cost_map[0] + " --c2 " + cost_map[1]
            elif dim == 3:
                command += " --c1 " + cost_map[0] + " --c2 " + cost_map[1] + " --c3 " + cost_map[2]
            
            os.system(command)

            with open(output_file, "r") as output:
                lines = output.readlines()
                if "SolutionNum" not in lines[-1] and "SolutionNum" not in lines[-2]:
                    if_error = True
        
        if if_error:
            with open(output_file, "a") as output:
                output.write("FAIL\napex cost: \n\nreal cost:\n\n")
                if algorithm == "BBMOCBS-k":
                    output.write("epsilon = 0\n")
                output.write("\n\nHLMergingTime = 0\nLowLevelTime = 0\nTotal Time = 0\nConflictSolvingNum = 0\nSolutionNum = 0\n")

        with open(output_file, "a") as output:
            output.write("\n\n")

    with open(output_file, "r") as output:
        for line in output:
            if "AVERAGE" in line:
                return
            
    total_HLMergingTime, total_LowLevelTime, total_time, total_ConflictSovlingNum, total_SolutionNum, total_SuccessNum, total_num, total_epsilon = write_average_result(output_file)
    with open(output_file, "a") as output:
        if total_SuccessNum == 0:
            output.write("AVERAGE :\n")
            if algorithm == "BBMOCBS-k":
                output.write("EPSILON = 0\n")
            output.write("HLMergingTime = 0\n")
            output.write("LowLevelTime = 0\n")
            output.write("Total Time = 0\n")
            output.write("ConflictSolvingNum = 0\n")
            output.write("SolutionNum = 0\n")
            output.write("Success Rate = 0/0  = 0")
        else:
            output.write("AVERAGE :\n")
            if algorithm == "BBMOCBS-k":
                output.write("EPSILON = {:.4f}\n".format(total_epsilon/total_SuccessNum))
            output.write("HLMergingTime = {:.4f}\n".format(total_HLMergingTime/total_SuccessNum))
            output.write("LowLevelTime = {:.4f}\n".format(total_LowLevelTime/total_SuccessNum))
            output.write("Total Time = {:.4f}\n".format(total_time/total_SuccessNum))
            output.write("ConflictSolvingNum = {:.4f}\n".format(total_ConflictSovlingNum/total_SuccessNum))
            output.write("SolutionNum = {:.4f}\n".format(total_SolutionNum/total_SuccessNum))
            output.write("Success Rate = {}/{} = {:.4f}".format(total_SuccessNum, total_num, total_SuccessNum/total_num))
    # with open(output_file, "a"):
    #     output.write(
    #         "AVERAGE :"
    #         # unfinished
    #     )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", type=str, required=True, help="map file")
    parser.add_argument("-c", "--metric", type=str, required=True, help="cost metric: r, t, d")
    parser.add_argument("-a", "--algorithm", type=str, required=True, help="algorithm: BBMOCBS-eps, BBMOCBS-pex, BBMOCBS-k")
    parser.add_argument("-e", "--eps", type=float, default=0, help="approximate factor")
    parser.add_argument("-r", "--eps_ratio", type=float, default=0, help="high-level-merging eps / eps")
    parser.add_argument("-n", "--agent_num", type=int, required=True, help="number of agents")
    parser.add_argument("--CB", action="store_true", help="if conflict-based merging")
    parser.add_argument("--eager", action="store_true", help="if eager-solution update")
    parser.add_argument("-k", "--solution_num", type=int, default=sys.maxsize, help="desirable number of solutions: only for BB-MO-CBS-k")
    
    args = parser.parse_args()
    run_once(map=args.map, metric=args.metric, algorithm=args.algorithm, eps=args.eps, agent_num=args.agent_num, eps_ratio=args.eps_ratio, CB=args.CB, eager=args.eager, solution_num=args.solution_num)