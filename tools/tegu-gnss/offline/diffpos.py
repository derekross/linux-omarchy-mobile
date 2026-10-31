import sys,re,ast,collections
reads=[]
for l in open(sys.argv[1],errors='replace'):
    m=re.search(r'read\(\d+<[^>]*gnss_ipc>, "(.*)", \d+\) = (\d+)',l)
    if m: reads.append(ast.literal_eval('b"'+m.group(1)+'"'))
# normalise: strip to the first preamble
frames=collections.defaultdict(list)
for b in reads:
    i=b.find(b'hC')
    if i<0 or len(b)-i<40: continue
    frames[b[i:i+12]].append((i,b))
pos=collections.Counter(); tot=collections.Counter()
for k,v in frames.items():
    if len(v)<3: continue
    L=min(len(b)-i for i,b in v)
    # majority byte per frame offset
    for o in range(L):
        vals=collections.Counter(b[i+o] for i,b in v)
        maj=vals.most_common(1)[0][0]
        for i,b in v:
            ro=(i+o)%64  # offset inside the 64-byte read
            tot[ro]+=1
            if b[i+o]!=maj: pos[ro]+=1
print('read offset: bad/total')
print(' '.join('%d:%d/%d'%(o,pos[o],tot[o]) for o in range(64) if pos[o]))
