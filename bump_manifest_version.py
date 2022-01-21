import os

# Usage: see below. 
# 
# @author Steven Wong <housengw@berkeley.edu>
def usage(): 
    return  '''bump_manifest_version.py [-h] [-p PROP] [-t TARGET] [-a]

            update the version number in config.properties. 
            Then, run `python3 bump_manifest_version.py --all` to update dependency version for all targets (gradle, maven and manifest files).

            You can also specify a specific target and a specific property to update using `-t` and/or `-p`
            ex. python3 bump_manifest_version.py -p xtextVersion -t gradle
            '''


findPrintStyle = '%p:\\n'
replace = '\\n'
to = ' >>> '


class colors: 
    RED = '\033[31m'
    ENDC = '\033[m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    PURPLE = '\033[35m'

def main(args):
    import configparser
    config = configparser.RawConfigParser()
    config.optionxform = str
    config.read('config.properties')
    props =  dict(config.items('versions')).keys()

    if args["prop"]:
        prop = args["prop"]
        if prop not in props:
            print(colors.RED + "ERROR" + colors.ENDC + f": property {prop} does not exist")
            return
        props = [prop]

    if args["all"] and args["target"]:
        print(colors.RED + "ERROR" + colors.ENDC + f": --all and --target are mutually exclusive")
        return

    targetLower = args["target"].lower() if args["target"] else ""

    if args["all"]:
        updateGradlePackages(config, props)
        updateManifestPackages(config, props)
        updateMavenPackages(config, props)
    elif targetLower == "gradle":
        updateGradlePackages(config, props)
    elif targetLower == "maven":
        updateMavenPackages(config, props)
    elif targetLower == "manifest":
        updateManifestPackages(config, props)
    
def getInput():
    return input(colors.PURPLE + "The changes are printed to the screen. Enter [Y/y] to accept, [N/n] to reject, [Q/q] to quit: " + colors.ENDC).lower()

def updateMavenPackages(config, props):
    print(colors.GREEN + "Updating versions for >>> ./pom.xml" + colors.ENDC)
    propertyNameToVersion = dict(config.items('versions'))
    for prop in props:
        print("[MAVEN] Updating version for variable: " + colors.GREEN + prop + colors.ENDC)
        os.system(generateMavenViewCommand(prop, propertyNameToVersion[prop]))
        c = ''
        while c != 'y' and c != 'n':
            c = getInput()
            print()
            if c == 'y':
                os.system(generateMavenReplaceCommand(prop, propertyNameToVersion[prop]))
            elif c == 'q':
                return


def updateGradlePackages(config, props):
    print(colors.GREEN + "Updating versions for >>> gradle.properties" + colors.ENDC)
    propertyNameToVersion = dict(config.items('versions'))
    for prop in props:
        print("[GRADLE] Updating version for variable: " + colors.GREEN + prop + colors.ENDC)
        os.system(generateGradleViewCommand(prop, propertyNameToVersion[prop]))
        c = ''
        while c != 'y' and c != 'n':
            c = getInput()
            print()
            if c == 'y':
                os.system(generateGradleReplaceCommand(prop, propertyNameToVersion[prop]))
            elif c == 'q':
                return


def updateManifestPackages(config, props):
    print(colors.GREEN + "Updating versions for >>> manifest files" + colors.ENDC)
    packageToPropertyName = dict(config.items('manifestPropertyNames'))
    propertyNameToVersion = dict(config.items('versions'))

    for prop in props:
        manifestPackages = [package for package, propertyName in packageToPropertyName.items() if prop == propertyName]

        if len(manifestPackages) == 0 or prop not in propertyNameToVersion:
            print(colors.YELLOW + "WARNING" + colors.ENDC + f": Property '{prop}' does not exist for Manifest files.\n")
        
        for package in manifestPackages:
            print("[MANIFEST] Updating version for package: " + colors.GREEN + package + colors.ENDC)
            os.system(generateManifestViewCommand(package, propertyNameToVersion[prop]))
            c = ''
            while c != 'y' and c != 'n':
                c = getInput()
                print()
                if c == 'y':
                    os.system(generateManifestReplaceCommand(package, propertyNameToVersion[prop]))
                elif c == 'q':
                    return


def generateManifestViewCommand(matchName, version):
    return f'''
    find . -name *.MF -type f -printf '{findPrintStyle}' -exec sed -n '/{matchName}.*;bundle-version/{{
    h
    s/="[^"][^"]*"/="{version}"/g
    H
    x
    s/{replace}/{to}/
    w /dev/fd/2
    x
    }}' {{}} \;
    '''


def generateManifestReplaceCommand(matchName, version):
    return f'''
    find . -name *.MF -type f -exec sed -i '/{matchName}.*;bundle-version/{{
    h
    s/="[^"][^"]*"/="{version}"/g
    }}' {{}} \;
    '''


def generateGradleViewCommand(matchName, version):
    return f''' 
    find gradle.properties -printf '{findPrintStyle}' -exec sed -n '/{matchName}=/{{
    h
    s/=.*/={version}/g
    H
    x
    s/{replace}/{to}/
    w /dev/fd/2
    x
    }}' {{}} \;
    '''


def generateGradleReplaceCommand(matchName, version):
    return f'''
    find gradle.properties -exec sed -i '/{matchName}=/{{
    h
    s/=.*/={version}/g
    }}' {{}} \;
    '''


def generateMavenViewCommand(matchName, version):
    return f'''
    find pom.xml -printf '{findPrintStyle}' -exec sed -n '/<{matchName}>.*<\/{matchName}>/{{
    h
    s/<{matchName}>.*<\/{matchName}>/<{matchName}>{version}<\/{matchName}>/g
    H
    x
    s/{replace}/{to}/
    w /dev/fd/2
    x
    }}' {{}} \;
    '''


def generateMavenReplaceCommand(matchName, version):
    return f'''
    find pom.xml -exec sed -i '/<{matchName}>.*<\/{matchName}>/{{
    h
    s/<{matchName}>.*<\/{matchName}>/<{matchName}>{version}<\/{matchName}>/g
    }}' {{}} \;
    '''

if __name__ == '__main__':
    import argparse
    import sys

    parser = argparse.ArgumentParser(usage=usage())

    # Adding optional argument
    parser.add_argument("-p", "--prop", help="version property to update")
    parser.add_argument("-t", "--target", help="target build application to update. Can be one of 'gradle', 'maven' or 'manifest' (case insensitive)")
    parser.add_argument("-a", "--all", help="update versions for all dependencies", action="store_true")

    if len(sys.argv) == 1:
        parser.print_help()
        parser.exit()

    # Read arguments from command line
    args = parser.parse_args()
    main(vars(args))
