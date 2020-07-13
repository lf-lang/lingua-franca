#!/usr/local/bin/python
# This script is taken from
# https://www6.software.ibm.com/developerworks/education/l-pythonscript/l-pythonscript-ltr.pdf
# with some modifications.
# 
# This script can be used to generate a Makefile for C/C++ code that uses the Python C APIs.


import distutils.sysconfig
import string, sys

configopts = {}

maketemplate = """
PYLIB=%(pythonlib)s
PYINC=-I%(pythoninc)s
LIBS=%(pylibs)s
OPTS=%(pyopt)s
PROGRAMS=%(programs)s

all: $(PROGRAMS)

"""

# Add the Python library
configopts['pythonlib'] = '-L' + distutils.sysconfig.get_config_var('LIBPL') \
+ '/' + \
distutils.sysconfig.get_config_var('LIBRARY')


configopts['pythoninc'] = ''
configopts['pylibs'] = ''

# The required include directories
for dir in distutils.sysconfig.get_config_var('INCLDIRSTOMAKE').split():
    configopts['pythoninc'] += '-I%s ' % (dir,)

# The required library directories
for dir in distutils.sysconfig.get_config_var('LIBDIR').split():
    configopts['pylibs'] += '-L%s ' % (dir,)

configopts['pylibs'] += distutils.sysconfig.get_config_var('MODLIBS') \
        + ' ' + \
        distutils.sysconfig.get_config_var('LIBS') \
        + ' ' + \
        distutils.sysconfig.get_config_var('SYSLIBS')
configopts['pyopt'] = distutils.sysconfig.get_config_var('OPT')

# Add the program name to targets
targets = ''
for arg in sys.argv[1:]:
    targets += arg + ' '
configopts['programs'] = targets

# Fill the Makefile template
print(maketemplate % configopts)

# Generate the rest of the Makefile
for arg in sys.argv[1:]:
    print("%s: %s.o\n\tgcc %s.o -shared $(LIBS) $(PYLIB) -o %s" \
        % (arg, arg, arg, arg))
    print("%s.o: %s.c\n\tgcc %s.c -fPIC -c $(PYINC) $(OPTS)" \
        % (arg, arg, arg))

print("clean:\n\trm -f $(PROGRAMS) *.o *.pyc core")
