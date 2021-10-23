import os
from subprocess import run

print(os.environ['GITHUB_WORKSPACE'])
os.chdir(os.environ['GITHUB_WORKSPACE'])
print( 'Running command...' )
run( [ 'bin/build-lfc', '--help' ], check=True )

# If `cat /foo` fails above, we won't get here.
print( 'All done!' )