import os

def execute_script(script_path):
    # Execute the provided script
    os.system(script_path)

def traverse_directories(root_dir, script_to_execute):
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith('.launch'):
                launch_file_path = os.path.join(root, file)
                script_path = script_to_execute + " " + launch_file_path
                execute_script(script_path)
                # Delete the old .launch file after execution
                os.remove(launch_file_path)

if __name__ == "__main__":
    root_directory = "/home/ahmo030/arena_ws"
    script_to_execute = "migrate_launch_file"
    traverse_directories(root_directory, script_to_execute)
