import sys,collections
sys.path.insert(0, __import__('os').path.dirname(__file__))
from chpp import reads,frames
def shl(b,k):
    n=int.from_bytes(b,'big')<<k
    return (n & ((1<<(8*len(b)))-1)).to_bytes(len(b),'big')
for path in sys.argv[1:]:
    tot=collections.Counter(); best=collections.Counter(); bad_all=0
    for b in reads(path):
        if not any(b): continue
        res={k:sum(ok for _,ok in frames(shl(b,k))) for k in range(8)}
        for k,v in res.items(): tot[k]+=v
        best[max(res.values())>0]+=1
        if max(res.values())==0 and b.find(b'hC')>=0: bad_all+=1
    print(path.split('/')[-1], 'valid frames per shift:', dict(tot), 'reads with any valid:', dict(best))
