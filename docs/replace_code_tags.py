#!/usr/bin/env python3
import re
import sys
import os
import glob

def replace_code_tags(file_path):
    """Replace all {@code X} patterns with `X` (backticks around the content) in the given file."""
    
    try:
        # Read the file
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Replace {@code X} with `X` (backticks around the content)
        # This regex matches {@code followed by any characters until }
        pattern = r'\{@code\s*([^}]+)\}'
        replacement = r'`\1`'
        
        # Perform the replacement
        new_content = re.sub(pattern, replacement, content)
        
        # Only write if content actually changed
        if new_content != content:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(new_content)
            print(f"Updated: {file_path}")
        else:
            print(f"No changes needed: {file_path}")
            
    except Exception as e:
        print(f"Error processing {file_path}: {e}")

def process_directory(directory_path):
    """Process all Java files recursively in the given directory."""
    
    # Find all Java files recursively
    java_files = glob.glob(os.path.join(directory_path, "**/*.java"), recursive=True)
    
    if not java_files:
        print(f"No Java files found in {directory_path}")
        return
    
    print(f"Found {len(java_files)} Java files in {directory_path}")
    
    # Process each Java file
    for java_file in java_files:
        replace_code_tags(java_file)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 replace_code_tags.py <file_or_directory_path>")
        print("  If path is a file: processes that file only")
        print("  If path is a directory: processes all .java files recursively")
        sys.exit(1)
    
    path = sys.argv[1]
    
    if os.path.isfile(path):
        # Process single file
        if path.endswith('.java'):
            replace_code_tags(path)
        else:
            print(f"Error: {path} is not a Java file")
            sys.exit(1)
    elif os.path.isdir(path):
        # Process directory recursively
        process_directory(path)
    else:
        print(f"Error: {path} is not a valid file or directory")
        sys.exit(1)
