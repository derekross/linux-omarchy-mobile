import sys,collections
sys.path.insert(0, __import__('os').path.dirname(__file__))
from chpp import reads,frames
from realign import shl
ok=collections.Counter(); bad=collections.Counter()
for b in reads(sys.argv[1]):
    if not any(b): continue
    for ln,v in frames(shl(b,1)):
        (ok if v else bad)[ln]+=1
print('valid by length', sorted(ok.items())); print('invalid by length', sorted(bad.items()))
# per-chunk shift instead of whole buffer
ok2=collections.Counter(); bad2=collections.Counter()
for b in reads(sys.argv[1]):
    if not any(b): continue
    c=b''.join(shl(b[i:i+64],1) for i in range(0,len(b),64))
    for ln,v in frames(c): (ok2 if v else bad2)[ln]+=1
print('per-64 chunk: valid', sum(ok2.values()), 'invalid', sum(bad2.values()), sorted(bad2.items())[:8])
