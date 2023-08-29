import os

def fix_filenames_in_directory(directory):
    for root, dirs, files in os.walk(directory):
        for filename in files:
            new_filename = filename.rstrip('\n')
            if new_filename != filename:
                old_path = os.path.join(root, filename)
                new_path = os.path.join(root, new_filename)
                os.rename(old_path, new_path)
                print(f"Renamed: {old_path} -> {new_path}")

if __name__ == "__main__":
    target_directory = "你的目标目录的路径"
    fix_filenames_in_directory("new_2.out")
    fix_filenames_in_directory("new_3.out")