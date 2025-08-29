#!/usr/bin/env python3
import re
import sys

def replace_code_tags(file_path):
    """Replace all {@code X} patterns with `X` (backticks around the content) in the given file."""
    
    # Read the file
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Replace {@code X} with `X` (backticks around the content)
    # This regex matches {@code followed by any characters until }
    pattern = r'\{@code\s*([^}]+)\}'
    replacement = r'`\1`'
    
    # Perform the replacement
    new_content = re.sub(pattern, replacement, content)
    
    # Write back to the file
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    
    print(f"Replaced code tags with backticks in {file_path}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 replace_code_tags.py <file_path>")
        sys.exit(1)
    
    file_path = sys.argv[1]
    replace_code_tags(file_path)
