import os

def should_include_file(file_path, include_patterns, exclude_patterns):
    """
    Determine if a file should be included based on patterns.
    """
    file_name = os.path.basename(file_path)

    for pattern in exclude_patterns:
        if pattern in file_path or pattern in file_name:
            return False

    if not include_patterns:
        return True

    for pattern in include_patterns:
        if pattern in file_path or pattern in file_name:
            return True

    return False

def aggregate_code_content(directories, output_file, include_patterns=None, exclude_patterns=None):
    """
    Aggregate code content from multiple directories into a single file.
    
    Args:
        directories (list): List of directory paths to process
        output_file (str): Path to the output file
        include_patterns (list): File patterns to include (e.g., ['.py', '.md'])
        exclude_patterns (list): File patterns to exclude (e.g., ['.pyc', '__pycache__'])
    """
    with open(output_file, 'w', encoding='utf-8') as out_file:
        for directory in directories:
            for root, _, files in os.walk(directory):
                for file in files:
                    file_path = os.path.join(root, file)
                    if should_include_file(file_path, include_patterns, exclude_patterns):
                        try:
                            with open(file_path, 'r', encoding='utf-8') as f:
                                code_snippet = f.read()
                                relative_path = os.path.relpath(file_path, directory)
                                out_file.write(f"\n\nFile: {relative_path}\n\n```\n{code_snippet}\n```\n")
                        except Exception as e:
                            print(f"Error reading file {file_path}: {str(e)}")

if __name__ == "__main__":
    # Example configuration
    directories = [
        './flight-controller',
    ]
    output_file = 'aggregated_sources.txt'
    
    include_patterns = ['.c', '.h', '.txt', '.md']
    exclude_patterns = [
        '.whl', '.egg', '.pyc', '.git', '.tox', 
        '.venv', '.vscode', '.pytest_cache', '__pycache__'
    ]

    aggregate_code_content(
        directories=directories,
        output_file=output_file,
        include_patterns=include_patterns,
        exclude_patterns=exclude_patterns
    )

    print(f"Aggregated content saved to: {output_file}")
