import matplotlib.pyplot as plt
import numpy as np

import os
x = [4, 8, 12, 16, 20, 24, 28]
mm = 3
if mm == 1:
    mymod = "Success Rate"
if mm == 2:
    mymod = "Total Time"
if mm == 3:
    mymod = "DomPruneNum"

myk = 1
mycost = "unique-distance"
date = ""
map = "random-64-64-20"
if_turn = 5
eps = 0.030

directory_path = "new_2.out/" + map

def process_directory(directory_path, num_values, mod, k):
    def get_success_rates(file_prefix):
        success_rates = []
        for i in num_values:
            sub_directory = os.path.join(directory_path, f"n={i}")
            matching_files = [filename for filename in os.listdir(sub_directory) if filename.startswith(file_prefix)]
            matching_files.sort()
            l = 0
            for file_path in matching_files:
                l += 1
                if l == k:
                    with open(os.path.join(sub_directory, file_path), 'r') as file:
                        lines = file.readlines()
                        for line in reversed(lines):
                            if line.startswith(mod):
                                sign = "="
                                if mod == "DomPruneNum":
                                    sign = "/"
                                number_str = line.split(sign)[-1].strip()
                                if number_str == "-nan":
                                    success_rates.append(float(number_str))
                                else :
                                    success_rates.append(float(number_str))
                                break
        return success_rates
    suc1 = 1; suc2 = 1; suc3 = 1; suc4 = 1; suc5 = 1; suc6 = 1; suc7 = 1
    if eps == 0.020:
        suc1 = get_success_rates("0.000, 0.020, 0.000, 0.020" + date)
        suc2 = get_success_rates("0.000, 0.020, 0.016, 0.020" + date)
        suc3 = get_success_rates("0.016, 0.020, 0.000, 0.000" + date)
        suc4 = get_success_rates("0.016, 0.020, 0.000, 0.020" + date)
        suc5 = get_success_rates("0.016, 0.020, 0.016, 0.020" + date)
        suc6 = get_success_rates("BOA-0.020" + date)
        suc7 = get_success_rates("BOA-0.000" + date)
    if eps == 0.050:
        suc1 = get_success_rates("0.000, 0.050, 0.000, 0.050" + date)
        suc2 = get_success_rates("0.000, 0.050, 0.040, 0.050" + date)
        suc3 = get_success_rates("0.040, 0.050, 0.000, 0.000" + date)
        suc4 = get_success_rates("0.040, 0.050, 0.000, 0.050" + date)
        suc5 = get_success_rates("0.040, 0.050, 0.040, 0.050" + date)
        suc6 = get_success_rates("BOA-0.050" + date)
        suc7 = get_success_rates("BOA-0.000" + date)
    if eps == 0.030:
        suc1 = get_success_rates("0.000, 0.030, 0.000, 0.030" + date)
        suc2 = get_success_rates("0.000, 0.030, 0.024, 0.030" + date)
        suc3 = get_success_rates("0.024, 0.030, 0.000, 0.000" + date)
        suc4 = get_success_rates("0.024, 0.030, 0.000, 0.030" + date)
        suc5 = get_success_rates("0.024, 0.030, 0.024, 0.030" + date)
        suc6 = get_success_rates("BOA-0.030" + date)
        suc7 = get_success_rates("BOA-0.000" + date)
    if eps == 0.010:
        suc1 = get_success_rates("0.000, 0.010, 0.000, 0.010" + date)
        suc2 = get_success_rates("0.000, 0.010, 0.008, 0.010" + date)
        suc3 = get_success_rates("0.008, 0.010, 0.000, 0.000" + date)
        suc4 = get_success_rates("0.008, 0.010, 0.000, 0.010" + date)
        suc5 = get_success_rates("0.008, 0.010, 0.008, 0.010" + date)
        suc6 = get_success_rates("BOA-0.010" + date)
        suc7 = get_success_rates("BOA-0.000" + date)
    
    return suc1, suc2, suc3, suc4, suc5, suc6, suc7

result = process_directory(directory_path, x, mymod, myk)

suc1, suc2, suc3, suc4, suc5, suc6, suc7 = result
# if mymod == "Success Rate":
#     suc7.append(0); suc7.append(0)
# else:
#     suc7.append(None); suc7.append(None)

# plt.plot(x, suc1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=2, marker="o", linestyle="--", markersize=4)
# plt.plot(x, suc2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=2, marker="*", linestyle="--", markersize=4)
# # plt.plot(x, suc3, label="0.016, 0.020, 0.000, 0.000", color="purple", linewidth=2, marker="v", linestyle="--", markersize=4)
# plt.plot(x, suc4, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=2, marker="^", linestyle="--", markersize=4)
# plt.plot(x, suc5, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=2, marker="v", linestyle="--", markersize=4)
# plt.plot(x, suc6, label="NAMOA*dr-eps", color="c", linewidth=2, marker="s", linestyle="--", markersize=4)
# plt.plot(x, suc7, label="NAMOA*dr", color="black", linewidth=2, marker="D", linestyle="--", markersize=4)
# plt.title(map + "    eps = 0.020    " + mycost)

# print(suc1)
# print(suc2)
# print(suc3)
# print(suc4)
# print(suc5)
# print(suc6)
# print(suc7)
# input()
plt.plot(x, suc1, label="BB-MO-CBS-eps-LP", color="blue", linewidth=2, marker="o", linestyle="--", markersize=6)
plt.plot(x, suc2, label="BB-MO-CBS-eps-LM-LP", color="red", linewidth=2, marker="*", linestyle="--", markersize=6)
plt.plot(x, suc3, label="BB-MO-CBS-eps-HM", color="purple", linewidth=2, marker="v", linestyle="--", markersize=6)
plt.plot(x, suc4, label="BB-MO-CBS-eps-LP-HM", color="green", linewidth=2, marker="^", linestyle="--", markersize=6)
plt.plot(x, suc5, label="BB-MO-CBS-eps-LM-LP-HM", color="m", linewidth=2, marker="v", linestyle="--", markersize=6)
plt.plot(x, suc6, label="BB-MO-CBS-eps", color="c", linewidth=2, marker="s", linestyle="--", markersize=6)
plt.plot(x, suc7, label="BB-MO-CBS", color="black", linewidth=2, marker="D", linestyle="--", markersize=6)
if if_turn:
    title = map + "    eps = " + str(eps) + "\n" + mycost + "    turn_cost = " + str(if_turn)
else:
    title = map + "    eps = " + str(eps) + "\n" + mycost
plt.title(title)

plt.legend()
plt.grid(True)
plt.xlabel("agent num")
# tempmod2 = "success rate"
# tempmod1 = "suc_rate"
if mymod == "Success Rate":
    tempmod1 = "suc_rate"
    tempmod2 = "success rate"
if mymod == "Total Time":
    tempmod1 = "time"
    tempmod2 = "average time"
if mymod == "DomPruneNum":
    tempmod1 = "conflict"
    tempmod2 = "#conflict"

if mycost == "unique-distance":
    tempcost = "ud"
if mycost == "random-random":
    tempcost = "rr"
if mycost == "random-distance":
    tempcost = "rd"
if mycost == "random-distance-unique":
    tempcost = "rdu"
if mycost == "unique-random":
    tempcost = "ur"

plt.ylabel(tempmod2)
plt.savefig("image_/" + map + "-eps=" + str(eps) + "-" + tempmod1 + "-" + tempcost + ".jpg")
plt.show()
