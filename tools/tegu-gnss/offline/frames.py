import sys,re,ast
n=0
for l in open(sys.argv[1],errors='replace'):
    m=re.search(r'read\(\d+<[^>]*gnss_ipc>, "(.*)", \d+\) = (\d+)',l)
    if not m: continue
    b=ast.literal_eval('b"'+m.group(1)+'"')
    i=b.find(b'hC')
    if i<0: continue
    print(l[6:21], b[i:i+40].hex(' '))
    n+=1
    if n>=int(sys.argv[2]): break
