import sys,re,ast,zlib,struct,collections
def reads(path):
    for l in open(path,errors='replace'):
        m=re.search(r'read\(\d+<[^>]*gnss_ipc>, "(.*)", \d+\) = (\d+)',l)
        if m: yield ast.literal_eval('b"'+m.group(1)+'"')
def frames(buf):
    out=[];i=0
    while True:
        i=buf.find(b'hC',i)
        if i<0 or i+10>len(buf): break
        h=buf[i+2:i+10]
        ln=struct.unpack_from('<H',h,4)[0]
        end=i+10+ln+4
        if end<=len(buf) and ln<2048:
            body=buf[i+2:i+10+ln]; foot=struct.unpack_from('<I',buf,i+10+ln)[0]
            out.append((ln, zlib.crc32(body)&0xffffffff==foot))
        i+=2
    return out
if __name__=='__main__':
    c=collections.Counter()
    for b in reads(sys.argv[1]):
        for ln,ok in frames(b): c[ok]+=1
    print(c)
