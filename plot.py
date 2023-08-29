import matplotlib.pyplot as plt
import numpy as np

x = [4, 8, 12, 16, 20]
#empty-48-48  eps = 0.02
# y1 = [1, 1, 0.90625, 0.28125, 0.03125]
# plt.plot(x, y1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=3, marker="o", linestyle="--", markersize=4)
# y2 = [1, 1, 0.90625, 0.21875, 0.03125]
# plt.plot(x, y2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=3, marker="*", linestyle="--", markersize=4)
# y3 = [1, 1, 0.90625, 0.4375, 0.15625]
# plt.plot(x, y3, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=3, marker="^", linestyle="--", markersize=4)
# y4 = [1, 1, 0.875, 0.5, 0.125]
# plt.plot(x, y4, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=3, marker="v", linestyle="--", markersize=4)
# y5 = [1, 1, 0.90625, 0.59375, 0]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=4)
# y6 = [1, 0.625, 0.25, 0.03125, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=4)
# plt.title("empty-48-48    eps = 0.020")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("success rate")
# plt.savefig("image/empty-48-48-eps=0.020-suc_rate-rr.jpg")
# plt.show()

# #random-32-32-20  eps=0.02
# y1 = [1, 1, 0.6875, 0.28125, 0.03125]
# plt.plot(x, y1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=3, marker="o", linestyle="--", markersize=4)
# y2 = [1, 1, 0.75, 0.15625, 0]
# plt.plot(x, y2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [1, 1, 0.71875, 0.28125, 0]
# plt.plot(x, y3, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [1, 1, 0.6875, 0.21875, 0]
# plt.plot(x, y4, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [1, 1, 0.75, 0.25, 0]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y6 = [1, 0.84375, 0.375, 0, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("random-32-32-20    eps = 0.020    random cost——random cost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("success rate")
# plt.savefig("image/random-32-32-20-eps=0.020-suc_rate-rr.jpg")
# plt.show()

# #room-32-32-4  eps=0.02
# y1 = [1, 0.78125, 0.4375, 0.1875, 0]
# plt.plot(x, y1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [1, 0.8125, 0.4375, 0.21875, 0]
# plt.plot(x, y2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [1, 0.78125, 0.46875, 0.28125, 0]
# plt.plot(x, y3, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [1, 0.8125, 0.4375, 0.28125, 0]
# plt.plot(x, y4, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [1, 0.8125, 0.4375, 0.125, 0]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y6 = [1, 0.625, 0.25, 0.03125, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("room-32-32-4    eps = 0.020    random cost——random cost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("success rate")
# plt.savefig("image/room-32-32-4-eps=0.020-suc_rate-rr.jpg")
# plt.show()

# #TIME
# #empty-48-48  eps = 0.02
# y1 = [1.39687, 6.86571, 30.3407, 76.0468, 102.041]
# plt.plot(x, y1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [1.23081, 6.48872, 31.0598, 70.2234, 103.263]
# plt.plot(x, y2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [1.5494, 8.24078, 27.9413, 45.1109, 55.39]
# plt.plot(x, y3, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [1.38197, 6.73122, 24.812, 55.9605, 73.3252]
# plt.plot(x, y4, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [1.01788, 6.09152, 18.7816, 44.2729, 49.6664]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y6 = [2.17991, 13.9467, 24.4721, 27.4166, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("empty-48-48    eps = 0.020    random cost——random cost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("average time")
# plt.savefig("empty-48-48-eps=0.020-time-rr.jpg")
# plt.show()

# #Conflict Num
# y1 = [0.625, 3, 7.93103, 14.7778, 13]
# plt.plot(x, y1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [0.6875, 2.6875, 6.89655, 15.1429, 13]
# plt.plot(x, y2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [0.78125, 6.15625, 14.7241, 26.5714, 28.2]
# plt.plot(x, y3, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [0.90625, 5.21875, 12.5, 30.75, 33]
# plt.plot(x, y4, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [0.5625, 6.46875, 16.2759, 51.1579, 75.6]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y6 = [2.625, 22.1034, 42.8824, 72, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("empty-48-48    eps = 0.020    random cost —— random cost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("conflict number")
# plt.savefig("empty-48-48-eps=0.020-conflict-rr.jpg")
# plt.show()


# #TIME
# #random-32-32-20  eps = 0.02
# y1 = [0.216014, 2.62915, 15.8556, 67.5208, 115.008]
# plt.plot(x, y1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [0.222755, 3.62561, 23.2989, 72.1482, 0]
# plt.plot(x, y2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [0.246888, 5.7013, 18.5205, 51.9336, 0]
# plt.plot(x, y3, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [0.223079, 4.06397, 12.8707, 52.3652, 0]
# plt.plot(x, y4, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [0.149635, 1.47092, 7.00241, 44.2319, 0]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y6 = [0.296042, 6.02511, 19.3523, 0, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("random-32-32-20    eps = 0.020")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("average time")
# plt.savefig("image/random-32-32-20-eps=0.020-time-rr.jpg")
# plt.show()

# #Conflict Num
# y1 = [1.25, 18.2812, 56.2727, 73.5556, 100]
# plt.plot(x, y1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [1.21875, 21.9375, 73.0417, 69.6, 0]
# plt.plot(x, y2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [1.375, 59.1875, 124.043, 226.889, 0]
# plt.plot(x, y3, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [1.28125, 40.5938, 102.5, 248.571, 0]
# plt.plot(x, y4, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [1.40625, 22.8438, 83.4167, 387.375, 0]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y6 = [3.0625, 83.7407, 276.75, 0, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("random-32-32-20    eps = 0.020")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("conflict number")
# plt.savefig("image/random-32-32-20-eps=0.020-conflict-rr.jpg")
# plt.show()

# #TIME
# #room-32-32-4  eps = 0.02
# y1 = [0.766102, 14.2994, 23.5392, 26.3534, 0]
# plt.plot(x, y1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [0.585615, 11.2232, 22.3537, 72.3457, 0]
# plt.plot(x, y2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [0.966023, 17.511, 27.9206, 30.2698, 0]
# plt.plot(x, y3, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [0.778257, 12.6903, 12.5875, 33.7528, 0]
# plt.plot(x, y4, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [0.736538, 7.60392, 24.5706, 44.2954, 0]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y6 = [3.59969, 32.1444, 32.3841, 45.0345, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("room-32-32-4    eps = 0.020    random cost——random cost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("average time")
# plt.savefig("room-32-32-4-eps=0.020-time-rr.jpg")
# plt.show()

# #Conflict Num
# y1 = [15.4375, 183.96, 161.571, 133.333, 0]
# plt.plot(x, y1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [13.9375, 157.769, 157.5, 381.143, 0]
# plt.plot(x, y2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [19.3125, 290.52, 377.933, 460.333, 0]
# plt.plot(x, y3, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [19.3125, 240.577, 175.214, 380.778, 0]
# plt.plot(x, y4, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [22.5625, 188.538, 518.571, 584.25, 0]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y6 = [104.156, 934.95, 888.5, 695, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("room-32-32-4    eps = 0.020    random cost——random cost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("conflict number")
# plt.savefig("room-32-32-4-eps=0.020-conflict-rr.jpg")
# plt.show()


# # empty-48-48 eps = 0.050  randomcost+randomcost
# y1 = [1, 1, 0.96875, 0.75, 0.28125]
# plt.plot(x, y1, label="0.000, 0.050, 0.000, 0.050", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [1, 1, 0.96875, 0.6875, 0.1875]
# plt.plot(x, y2, label="0.000, 0.050, 0.040, 0.050", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [1, 1, 1, 0.84375, 0.5625]
# plt.plot(x, y3, label="0.040, 0.050, 0.000, 0.050", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [1, 1, 0.9375, 0.71875, 0.28125]
# plt.plot(x, y4, label="0.040, 0.050, 0.040, 0.050", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [1, 1, 1, 0.75, 0.4375]
# plt.plot(x, y4, label="0.040, 0.050, 0.000, 0.000", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y6 = [1, 1, 0.90625, 0.6875, 0.25]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y7 = [1, 0.625, 0.25, 0.03125, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("empty-48-48    eps = 0.050   randomcost+randomcost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("success rate")
# plt.show()

# #TIME
# y1 = [1.0697, 2.60231, 12.4251, 41.789, 73.8132]
# plt.plot(x, y1, label="0.000, 0.050, 0.000, 0.050", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [0.86395, 2.42501, 13.2883, 41.7095, 45.649]
# plt.plot(x, y2, label="0.000, 0.050, 0.040, 0.050", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [1.18025, 3.06992, 13.5632, 33.4421, 62.4872]
# plt.plot(x, y3, label="0.040, 0.050, 0.000, 0.050", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [0.785247, 2.36425, 9.74668, 36.7973, 45.3821]
# plt.plot(x, y4, label="0.040, 0.050, 0.040, 0.050", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [1.49501, 4.6129, 17.5497, 36.1083, 53.6399]
# plt.plot(x, y4, label="0.040, 0.050, 0.000, 0.000", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y6 = [0.844322, 3.96501, 18.5535, 36.5971, 57.9016]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y7 = [2.17991, 13.9467, 24.4721, 27.4166, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("empty-48-48    eps = 0.050   randomcost+randomcost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("average time")
# plt.show()

# #Conflict Num
# y1 = [0.375, 1.3125, 5.45161, 15.4583, 17.4444]
# plt.plot(x, y1, label="0.000, 0.050, 0.000, 0.050", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [0.375, 1.8125, 7.19355, 16.7273, 15.1667]
# plt.plot(x, y2, label="0.000, 0.050, 0.040, 0.050", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y3 = [0.4375, 2.03125, 8.40625, 17.037, 31.0556]
# plt.plot(x, y3, label="0.040, 0.050, 0.000, 0.050", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [0.375, 2, 7.16667, 18, 25.2222]
# plt.plot(x, y4, label="0.040, 0.050, 0.040, 0.050", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y5 = [0.40625, 2.8125, 9.84375, 24.9167, 39.0714]
# plt.plot(x, y4, label="0.040, 0.050, 0.000, 0.000", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y6 = [0.25, 3.09375, 12.069, 37.7727, 62.375]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y7 = [2.625, 22.1034, 42.8824, 72, 0]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("empty-48-48    eps = 0.050   randomcost+randomcost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("conflict number")
# plt.show()

# # empty-48-48 eps = 0.050  uniquecost+uniquecost
# y1 = [1, 1, 1, 1, 1]
# plt.plot(x, y1, label="0.000, 0.050, 0.000, 0.050", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [1, 1, 1, 1, 1]
# plt.plot(x, y2, label="0.000, 0.050, 0.040, 0.050", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y5 = [1, 1, 1, 1, 0.84375]
# plt.plot(x, y4, label="0.040, 0.050, 0.000, 0.000", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y3 = [1, 1, 1, 1, 1]
# plt.plot(x, y3, label="0.040, 0.050, 0.000, 0.050", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [1, 1, 1, 1, 1]
# plt.plot(x, y4, label="0.040, 0.050, 0.040, 0.050", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y6 = [1, 1, 1, 1, 1]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y7 = [1, 1, 1, 0.75, 0.40625]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("empty-48-48    eps = 0.050   uniquecost+uniquecost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("success rate")
# plt.show()

# #TIME
# y1 = [0.172711, 0.376126, 0.630253, 1.40699, 2.54716]
# plt.plot(x, y1, label="0.000, 0.050, 0.000, 0.050", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [0.174772, 0.378632, 0.691419, 1.4416, 2.66515]
# plt.plot(x, y2, label="0.000, 0.050, 0.040, 0.050", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y5 = [0.179658, 0.447814, 0.793105, 3.37189, 13.1189]
# plt.plot(x, y4, label="0.040, 0.050, 0.000, 0.000", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y3 = [0.173967, 0.397322, 0.657048, 8.34375, 4.20936]
# plt.plot(x, y3, label="0.040, 0.050, 0.000, 0.050", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [0.176122, 0.379555, 0.708081, 1.48922, 3.95234]
# plt.plot(x, y4, label="0.040, 0.050, 0.040, 0.050", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y6 = [0.167466, 0.377058, 0.627345, 1.88439, 5.3395]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y7 = [0.172977, 0.694088, 3.6005, 8.89161, 9.12812]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("empty-48-48    eps = 0.050   uniquecost+uniquecost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("average time")
# plt.show()

# #Conflict Num
# y1 = [0.0625, 0.75, 2, 7.78125, 19]
# plt.plot(x, y1, label="0.000, 0.050, 0.000, 0.050", color="blue", linewidth=3, marker="o", linestyle="--", markersize=8)
# y2 = [0.0625, 0.8125, 2.1875, 8.46875, 19.5]
# plt.plot(x, y2, label="0.000, 0.050, 0.040, 0.050", color="red", linewidth=3, marker="*", linestyle="--", markersize=8)
# y5 = [0.09375, 1.25, 3.28125, 28.8438, 139.704]
# plt.plot(x, y4, label="0.040, 0.050, 0.000, 0.000", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y3 = [0.0625, 0.78125, 2.25, 8.34375, 35.1875]
# plt.plot(x, y3, label="0.040, 0.050, 0.000, 0.050", color="green", linewidth=3, marker="^", linestyle="--", markersize=8)
# y4 = [0.0625, 0.84375, 2.40625, 9.1875, 25.2222]
# plt.plot(x, y4, label="0.040, 0.050, 0.040, 0.050", color="m", linewidth=3, marker="v", linestyle="--", markersize=8)
# y6 = [0.09375, 0.78125, 1.90625, 14.125, 51.6875]
# plt.plot(x, y5, label="BOA*-eps", color="c", linewidth=3, marker="s", linestyle="--", markersize=8)
# y7 = [0.15625, 4.40625, 38.8438, 99.4583, 127.923]
# plt.plot(x, y6, label="BOA*", color="black", linewidth=3, marker="D", linestyle="--", markersize=8)
# plt.title("empty-48-48    eps = 0.050   uniquecost+uniquecost")

# plt.legend()
# plt.grid(True)
# plt.xlabel("agent num")
# plt.ylabel("conflict number")
# plt.show()

#random-32-32-20
#success rate

import os
num = [4, 8, 12, 16, 20]
mm = 3
if mm == 1:
    mymod = "Success Rate"
if mm == 2:
    mymod = "Total Time"
if mm == 3:
    mymod = "DomPruneNum"

myk = 1
mycost = "random cost——distance cost——unique cost"
date = ""
map = "random-32-32-20"

directory_path = "new_3.out/" + map

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
    # suc1 = get_success_rates("0.000, 0.020, 0.000, 0.020" + date)
    # suc2 = get_success_rates("0.000, 0.020, 0.016, 0.020" + date)
    # suc3 = get_success_rates("0.016, 0.020, 0.000, 0.000" + date)
    # suc4 = get_success_rates("0.016, 0.020, 0.000, 0.020" + date)
    # suc5 = get_success_rates("0.016, 0.020, 0.016, 0.020" + date)
    # suc6 = get_success_rates("BOA-0.020" + date)
    # suc7 = get_success_rates("BOA-0.000" + date)

    suc1 = get_success_rates("0.000, 0.050, 0.000, 0.050" + date)
    suc2 = get_success_rates("0.000, 0.050, 0.040, 0.050" + date)
    suc3 = get_success_rates("0.040, 0.050, 0.000, 0.000" + date)
    suc4 = get_success_rates("0.040, 0.050, 0.000, 0.050" + date)
    suc5 = get_success_rates("0.040, 0.050, 0.040, 0.050" + date)
    suc6 = get_success_rates("NAMOAdr-0.050" + date)
    suc7 = get_success_rates("NAMOAdr-0.000" + date)
    
    return suc1, suc2, suc3, suc4, suc5, suc6, suc7


num_values = [4, 8, 12, 16, 20]
result = process_directory(directory_path, num_values, mymod, myk)

suc1, suc2, suc3, suc4, suc5, suc6, suc7 = result
if mymod == "Success Rate":
    suc7.append(0); suc7.append(0)
else:
    suc7.append(None); suc7.append(None)

# plt.plot(x, suc1, label="0.000, 0.020, 0.000, 0.020", color="blue", linewidth=2, marker="o", linestyle="--", markersize=4)
# plt.plot(x, suc2, label="0.000, 0.020, 0.016, 0.020", color="red", linewidth=2, marker="*", linestyle="--", markersize=4)
# # plt.plot(x, suc3, label="0.016, 0.020, 0.000, 0.000", color="purple", linewidth=2, marker="v", linestyle="--", markersize=4)
# plt.plot(x, suc4, label="0.016, 0.020, 0.000, 0.020", color="green", linewidth=2, marker="^", linestyle="--", markersize=4)
# plt.plot(x, suc5, label="0.016, 0.020, 0.016, 0.020", color="m", linewidth=2, marker="v", linestyle="--", markersize=4)
# plt.plot(x, suc6, label="NAMOA*dr-eps", color="c", linewidth=2, marker="s", linestyle="--", markersize=4)
# plt.plot(x, suc7, label="NAMOA*dr", color="black", linewidth=2, marker="D", linestyle="--", markersize=4)
# plt.title(map + "    eps = 0.020    " + mycost)

plt.plot(x, suc1, label="0.000, 0.050, 0.000, 0.050", color="blue", linewidth=2, marker="o", linestyle="--", markersize=4)
plt.plot(x, suc2, label="0.000, 0.050, 0.040, 0.050", color="red", linewidth=2, marker="*", linestyle="--", markersize=4)
plt.plot(x, suc3, label="0.040, 0.050, 0.000, 0.000", color="purple", linewidth=2, marker="v", linestyle="--", markersize=4)
plt.plot(x, suc4, label="0.040, 0.050, 0.000, 0.050", color="green", linewidth=2, marker="^", linestyle="--", markersize=4)
plt.plot(x, suc5, label="0.040, 0.050, 0.040, 0.050", color="m", linewidth=2, marker="v", linestyle="--", markersize=4)
plt.plot(x, suc6, label="NAMOA*dr-eps", color="c", linewidth=2, marker="s", linestyle="--", markersize=4)
plt.plot(x, suc7, label="NAMOA*dr", color="black", linewidth=2, marker="D", linestyle="--", markersize=4)
plt.title(map + "    eps = 0.050    " + mycost)

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
    tempmod2 = "conflict number"

if mycost == "unique cost——distance cost":
    tempcost = "ud"
if mycost == "random cost——random cost":
    tempcost = "rr"
if mycost == "random cost——distance cost":
    tempcost = "rd"
if mycost == "random cost——distance cost——unique cost":
    tempcost = "rdu"
if mycost == "unique cost——random cost":
    tempcost = "ur"

plt.ylabel(tempmod2)
plt.savefig("image/" + map + "-eps=0.050-" + tempmod1 + "-" + tempcost + ".jpg")
plt.show()
