import os

'''
    Usage:
        Update the version number in config.properties. 
        Then, run `python3 bump_manifest_version.py -p [propertyName]`, where `[propertyName]` is the name to the left of the version numbers.

        ex. python3 bump_manifest_version.py -p xtextVersion

'''



class colors: 
    RED = '\033[31m'
    ENDC = '\033[m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'

def main(args):
    import configparser
    config = configparser.RawConfigParser()
    config.optionxform = str
    config.read('config.properties')
    packageToPropertyName = dict(config.items('manifestPropertyNames'))
    propertyNameToVersion = dict(config.items('manifestVersions'))
    propertyName = args["prop"]
    packages = [package for package, prop in packageToPropertyName.items() if prop == propertyName]

    if len(packages) == 0 or propertyName not in propertyNameToVersion:
        print(f"ERROR: Property '{propertyName}' does not exist")
        return
    
    for package in packages:
        print("Updating version for package: " + colors.GREEN + package + colors.ENDC)
        os.system(generateViewCommand(package, propertyNameToVersion[propertyName]))
        c = ''
        while c != 'y' and c != 'n':
            c = input("The changes are printed to the screen. Enter [Y/y] to accept, [N/n] to reject, [Q/q] to quit: ").lower()
            if c == 'y':
                os.system(generateReplaceCommand(package, propertyNameToVersion[propertyName]))
            elif c == 'q':
                return

def generateViewCommand(packageName, version):
    return f'''
    find . -name *.MF -type f -printf '\\n%p:\\n' -exec sed -n '/{packageName}.*;bundle-version/{{
    h
    s/="[^"][^"]*"/="{version}"/g
    H
    x
    s/\\n/ >>> /
    w /dev/fd/2
    x
    }}' {{}} \;
    '''

def generateReplaceCommand(packageName, version):
    return f'''
    find . -name *.MF -type f -exec sed -i '/{packageName}.*;bundle-version/{{
    h
    s/="[^"][^"]*"/="{version}"/g
    }}' {{}} \;
    '''


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
 
    # Adding optional argument
    parser.add_argument("-p", "--prop", help = "version property to update", required = True)
    
    # Read arguments from command line
    args = parser.parse_args()
    main(vars(args))
