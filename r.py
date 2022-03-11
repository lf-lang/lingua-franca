s = '''RUN set -ex && \\
mkdir bin && \\
cmake -S src-gen -B bin && \\
cd bin && \\
make all'''
    
r = '''String.join("\\n", \n'''

for l in s.split('\n'):
    r += ('    "' + l + '",\n')

r = r[:-2] + "\n);"
print(r)


top = '«(.*?)»'
bot = '"+$1+"'