import sys,re,ast
reads=[]
for l in open(sys.argv[1],errors='replace'):
    m=re.search(r'read\(\d+<[^>]*gnss_ipc>, "(.*)", \d+\) = (\d+)',l)
    if m: reads.append((l[6:21],ast.literal_eval('b"'+m.group(1)+'"')))
shown=0
for i,(t,b) in enumerate(reads):
    if len(b)>=192:
        print(t,len(b))
        for k in range(0,min(len(b),320),64):
            print('  %3d'%k, b[k:k+64].hex())
        shown+=1
        if shown>=3: break
