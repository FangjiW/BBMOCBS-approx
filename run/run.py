import os
import multiprocessing
import json
from run_once import run_once



def run():
    with open('config.json', 'r') as file:
        programs = json.load(file)


    with multiprocessing.Pool(processes=4) as pool:
        pool.starmap(run_once, [(program["map"], program["metric"], program["algorithm"], program["eps"], program["agent_num"],  
                                 program["eps_ratio"], program["CB"], program["eager"], program["solution_num"]) for program in programs])
    


if __name__ == "__main__":
    while True:
        run()