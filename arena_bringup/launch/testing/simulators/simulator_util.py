import os

def find_file(root, file_name, exclude_dirs=None):
    """
    Recursively searches for a file in a directory, excluding specified directories.
    
    :param root: Root directory to start the search.
    :param file_name: Name of the file to search for.
    :param exclude_dirs: List of directory names to exclude from the search.
    :return: Full path to the file if found, else None.
    """
    exclude_dirs = set(exclude_dirs or [])
    
    for dirpath, dirnames, filenames in os.walk(root):
        # Filter out excluded directories
        dirnames[:] = [d for d in dirnames if d not in exclude_dirs]
        
        # Check if the file is in the current directory
        if file_name in filenames:
            return os.path.join(dirpath, file_name)
    
    return None
