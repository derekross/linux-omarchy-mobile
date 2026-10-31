import re,struct,zlib,sys,collections
lines=open(sys.argv[1],errors='replace').read().splitlines()
bufs=[]; cur=None
for l in lines:
    m=re.search(r'gnss_spi: (RX|TX)\((\d+)\)\+(\d+): ([0-9a-f ]*)',l)
    if not m: continue
    tag,n,off,hx=m.group(1),int(m.group(2)),int(m.group(3)),m.group(4)
    if off==0: cur=bytearray(); bufs.append((tag,n,cur))
    if cur is not None: cur+=bytes.fromhex(hx.replace(' ',''))
ok=collections.Counter(); bad=collections.Counter()
for tag,n,b in bufs:
    if tag!='RX': continue
    i=0
    while True:
        i=b.find(b'hC',i)
        if i<0 or i+10>len(b): break
        ln=struct.unpack_from('<H',b,i+6)[0]
        if i+10+ln+4<=len(b) and ln<2048:
            body=b[i+2:i+10+ln]; foot=struct.unpack_from('<I',b,i+10+ln)[0]
            (ok if zlib.crc32(body)&0xffffffff==foot else bad)[ln]+=1
        i+=2
print('RX dumps',sum(1 for t,n,b in bufs if t=='RX'),'valid',sum(ok.values()),dict(ok),'invalid',sum(bad.values()),dict(bad))
