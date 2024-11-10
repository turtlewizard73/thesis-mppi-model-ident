def write_path_to_file(file_path: str, path: str):
    """Create or overwrite a text file with the given path."""
    with open(file_path, 'w') as file:
        file.write(path)
    print(f"Path '{path}' written to '{file_path}'")


def load_path_from_file(file_path: str) -> str:
    """Load the path from the specified text file."""
    try:
        with open(file_path, 'r') as file:
            path = file.read().strip()
        print(f"Path loaded from '{file_path}': {path}")
        return path
    except FileNotFoundError:
        print(f"File '{file_path}' not found.")
        return None


# Example usage:
file_name = "path.txt"
path_to_save = "/example/path/to/directory222"

# Write path to file
write_path_to_file(file_name, path_to_save)

# Load path from file
loaded_path = load_path_from_file(file_name)
