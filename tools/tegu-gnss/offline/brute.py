import sys,zlib,struct,itertools,collections
sys.path.insert(0, __import__('os').path.dirname(__file__))
from chpp import reads
res=collections.Counter()
for b in reads(sys.argv[1]):
    i=b.find(b'hC')
    while i>=0 and i+10<=len(b):
        ln=struct.unpack_from('<H',b,i+6)[0]
        end=i+10+ln+4
        if ln>54 and end<=len(b):
            fr=bytearray(b[i:end])
            # byte positions (in frame) that are the last byte of a 64-byte read
            pos=[p-i for p in range(63,len(b),64) if i<=p<end]
            good=None
            for combo in itertools.product([0,1],repeat=len(pos)):
                f=bytearray(fr)
                for p,c in zip(pos,combo): f[p]=(f[p]&0xfe)|c
                if zlib.crc32(bytes(f[2:10+ln]))&0xffffffff==struct.unpack_from('<I',f,10+ln)[0]:
                    good=combo; break
            res['fixed' if good is not None else 'not fixed']+=1
        i=b.find(b'hC',i+2)
print(res)
