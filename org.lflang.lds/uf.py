"""This script updates individual class files in a language
and diagram server fat jar that has already been created.
"""

import os
import argparse
import subprocess
import time
import threading
import tempfile
import shutil

# This is the path to the fat jar that is to be updated.
FAT_JAR = os.path.join('vscode-extension', 'ls', 'lflang-lds.jar')

def main(args):
    t0 = time.time()
    n = 0
    count_thread = threading.Thread(target=count)
    count_thread.start()
    try:
        for src_dir_name in ('src', 'xtend-gen'):
            directory = get_directory(args.name, src_dir_name)
            condition = lambda f: (
                os.path.isfile(os.path.join(directory, f))
                and compiler(f) is not None
            )
            if not directory:
                class_name_start = args.name.rindex('.')
                directory = get_directory(
                    args.name[:class_name_start],
                    src_dir_name
                )
                previous_condition = condition
                condition = lambda f: (
                    previous_condition(f) and (
                        os.path.splitext(os.path.basename(f))[0]
                        == args.name[(class_name_start+1):]
                    )
                )
            if not directory: continue
            files = list(filter(condition, os.listdir(directory)))
            update_fat_jar(directory, files)
            n += len(files)
        success('{} SOURCE FILE(S) UPDATED in {:.0f} seconds.'.format(
            n, time.time() - t0
        ))
    finally:
        count_thread.terminate = True

# ~~~~~~~~~~~~~~~ Compilation-related logic ~~~~~~~~~~~~~~~~

def update_fat_jar(directory, files):
    """Updates the language server fat jar with the specified Java and/or
    Kotlin files.
    :param directory: the directory in which the given files live
    :param file: the names of the files to be updated
    """
    if not files: return
    fat_jar_abs = os.path.join(get_repo_root(), FAT_JAR)
    output_dir = os.path.join(tempfile.gettempdir(), 'org.lflang.lds', 'src')
    for file in files:
        compiler(file)(fat_jar_abs, directory, file, output_dir)
    class_files = [
        os.path.relpath(artifact, output_dir)
        for artifact in files_with_extension(output_dir, '.class')
    ]
    clean_print('Updating fat jar with {}...'.format(', '.join([
        os.path.basename(class_file) for class_file in class_files
    ])))
    subprocess.check_call(
        ['jar', 'uf', fat_jar_abs, *class_files],
        cwd=output_dir
    )
    # This is not safe from symlink attacks!
    shutil.rmtree(output_dir)

def compiler(file):
    if file.endswith('.kt'):
        return compile_kotlin
    if file.endswith('.java'):
        return compile_java

def _javac_like_compiler(name):
    def compiler(classpath, directory, file, output_dir):
        clean_print('Compiling {}...'.format(file))
        subprocess.check_call(
            [name, '-cp', classpath, '-d', output_dir, file],
            cwd=directory,
            shell=(os.name == 'nt')  # True iff the OS is Windows.
        )
    return compiler

compile_java = _javac_like_compiler('javac')
"""Compiles the Java file `file`.
:param classpath: an absolute path to a jar containing all files
    needed by this file
:param directory: the directory in which the Java file lives
:param file: the name of the Java file
"""

compile_kotlin = _javac_like_compiler('kotlinc')
"""Compiles the Kotlin file `file`.
:param classpath: an absolute path to a jar containing all files
    needed by this file
:param directory: the directory in which the Kotlin file lives
:param file: the name of the Kotlin file
"""

# ~~~~~~~~~~~~~~~ File system-related logic ~~~~~~~~~~~~~~~~

def files_with_extension(directory, extension):
    for dirpath, dirnames, filenames in os.walk(directory):
        for filename in filenames:
            if os.path.splitext(filename)[1] == extension:
                yield os.path.join(dirpath, filename)

def get_directory(package_name, src_dir_name):
    """Returns the directory associated with the module that has the
    given canonical name. Returns None if there is no such directory.
    :param package_name: the canonical name of the desired package
    """
    parts = package_name.split('.')
    def get_directory(subproject_len, src_dir_name):
        return os.path.join(
            get_repo_root(),
            '.'.join(parts[:subproject_len]),
            src_dir_name, *parts
        )
    subproject_len = 0
    while subproject_len <= len(parts):
        subproject_len += 1
        path = get_directory(subproject_len, src_dir_name)
        if os.path.isdir(path):
            return path # The path leads to a package.

def get_repo_root():
    """Returns the absolute path to the root of the repository in which
    this script was invoked.
    """
    return get_suitable_parent(
        lambda path: os.path.isdir(os.path.join(path, '.git'))
    )

def get_src_directory(path):
    """Returns a path to the parent `src` directory of the specified
    path.
    """
    return get_suitable_parent(
        lambda path: os.path.basename(path) == 'src',
        path
    )

def get_suitable_parent(condition, path='.'):
    assert path != os.pardir, 'Could not find the requested parent directory.'
    if condition(path):
        return os.path.abspath(path)
    return get_suitable_parent(condition, get_parent_dir(path))

def get_parent_dir(path):
    """Returns the parent directory of the file denoted by `path`."""
    return os.path.abspath(os.path.join(path, os.pardir))

# ~~~~~~~~~~~~~~~  Printing-related logic   ~~~~~~~~~~~~~~~~

def get_clean_print(n_chars_to_overwrite):
    def clean_print(message, r=255, g=255, b=255, end='\n'):
        difference = n_chars_to_overwrite - len(message + end)
        print(colored(r, g, b, message), end='')
        if end in ('\r', '\n'):
            print(' ' * difference, end=end)
        if end == '\r':
            return get_clean_print(len(message))
        elif end == '\n':
            return get_clean_print(0)
        else:
            return get_clean_print(difference)
    return clean_print

class Printer:
    def __init__(self):
        self._clean_print = get_clean_print(0)
    def clean_print(self, message):
        self._clean_print = self._clean_print(message)
    def progress(self, message):
        self._clean_print = self._clean_print(message, 255, 255, 0, end='\r')
    def error(self, message):
        self._clean_print = self._clean_print('[ERROR]', 255, 0, 0, end=' ')
        self._clean_print = self._clean_print(message)
    def success(self, message):
        self._clean_print = self._clean_print(
            'SUCCESS: {}'.format(message), 100, 255, 0
        )
    def count(self):
        t0 = time.time()
        while not getattr(threading.current_thread(), 'terminate', False):
            self.progress('Elapsed time: {:.0f} seconds'.format(time.time() - t0))
            time.sleep(0.1)

_printer = Printer()
clean_print = _printer.clean_print
progress = _printer.progress
error = _printer.error
success = _printer.success
count = _printer.count

def colored(r, g, b, text):
    return '\033[38;2;{};{};{}m{} \033[38;2;255;255;255m'.format(r, g, b, text)

# ~~~~~~~~~~~~~~~       Entry point         ~~~~~~~~~~~~~~~~

if __name__ == '__main__':
    argParser = argparse.ArgumentParser(
        description='This script updates individual class files in a '
                    'language and diagram server fat jar that has '
                    'already been created.'
    )
    argParser.add_argument(
        '-jar', default='jar',
        help='override jar command to adjust java version, e.g. '
             '/usr/lib/jvm/java-11-openjdk-amd64/bin/jar'
    )
    argParser.add_argument(
        'name',
        help='Class or module to recompile, specified by its canonical name.'
    )
    main(argParser.parse_args())
