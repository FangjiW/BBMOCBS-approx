import os

# 获取当前目录下所有以"0.000, 0.020, 0.016, 0.020-rr-Sat"开头的文件
files_to_process = [filename for filename in os.listdir() if filename.startswith("0.000, 0.010, 0.008, 0.010-ud-Sun Sep 10 13:04:36 2023")]

for filename in files_to_process:
    # Initialize variables for each file
    total_non_dom_time = 0
    total_low_level_time = 0
    total_time = 0
    total_cat_time = 0
    total_dom_prune_num = 0
    total_node_expand_num = 0
    total_solution_num = 0
    total_valid_solution_num = 0
    total_success_count = 0
    total_num = 0

    # Read the data from the current file
    with open(filename, 'r') as file:
        lines = file.readlines()

    # Process each line of the data
        succes_num = 0
        for line in lines:
            line = line.strip()
            if line.startswith("FAIL"):
                total_num += 1
            if line.startswith("SUCCESS"):
                total_success_count += 1
                succes_num += 1
                total_num += 1
                
            if line.startswith("NonDomTime"):
                parts = line.split("=")
                if len(parts) == 2:
                    # 如果成功分割成两部分，第二部分即为数字
                    total_non_dom_time += float(parts[1]) * succes_num
                            
            if line.startswith("LowLevelTime"):
                parts = line.split("=")
                if len(parts) == 2:
                    # 如果成功分割成两部分，第二部分即为数字
                    total_low_level_time += float(parts[1]) * succes_num
                            
            if line.startswith("Total Time"):
                parts = line.split("=")
                if len(parts) == 2:
                    total_time += float(parts[1]) * succes_num
                            
            if line.startswith("CAT Time"):
                parts = line.split("=")
                if len(parts) == 2:
                    total_cat_time += float(parts[1]) * succes_num
                            
            if line.startswith("DomPruneNum"):
                parts = line.split("=")
                if len(parts) == 2:
                    # 获取NodeExpandNum部分（位于等号后面的部分），并去除首尾空格
                    num = parts[1].strip()

                    # 进一步分割DomPruneNum和NodeExpandNum部分，以提取数值
                    value = num.split('/')
                    total_dom_prune_num += float(value[0])*succes_num
                    total_node_expand_num += float(value[1])*succes_num
            
            if line.startswith("SolutionNum = "):
                parts = line.split("=")
                if len(parts) == 2:
                    # 获取等号后面的部分，并去除首尾空格
                    values_part = parts[1].strip()

                    # 进一步分割两个数值部分，以提取数值
                    values = values_part.split('/')

                    if len(values) == 2:
                        # 提取两个数值部分
                        total_solution_num += float(values[0])*succes_num
                        total_valid_solution_num += float(values[1])*succes_num
                        succes_num = 0

    # Calculate averages for the current file
    print("AVERAGE:")
    print("NonDomTime = ", round(total_non_dom_time/total_success_count, 4))
    print("LowLevelTime = ", round(total_low_level_time/total_success_count, 4))
    print("Total Time = ", round(total_time/total_success_count, 4))
    print("CAT Time = ", round(total_cat_time/total_success_count, 4))
    print("DomPruneNum/NodeExpandNum = ", round(total_dom_prune_num/total_success_count), "/", round(total_node_expand_num/total_success_count, 4))
    print("SolutionNum = ", round(total_solution_num/total_success_count), "/", round(total_valid_solution_num/total_success_count, 4))
    print("Success Rate = ", total_success_count, "/", total_num, " = ", round(total_success_count/total_num, 4))
    # print(total_non_dom_time/total_success_count)
    # print(total_low_level_time/total_success_count)
    # print(total_time/total_success_count)
    # print(total_cat_time/total_success_count)
    # print(total_dom_prune_num/total_success_count)
    # print(total_node_expand_num/total_success_count)
    # print(total_solution_num/total_success_count)
    # print(total_valid_solution_num/total_success_count)
    # print(total_success_count/total_num)
