s = '''if («ref»->is_present) {
    // Put the whole token on the event queue, not just the payload.
    // This way, the length and element_size are transported.
    schedule_token(«action.name», 0, «ref»->token);
}'''
    
r = '''String.join("\\n", \n'''

for l in s.split('\n'):
    r += ('    "' + l + '",\n')

r = r[:-2] + "\n);"
print(r)


top = '«(.*?)»'
bot = '"+$1+"'

