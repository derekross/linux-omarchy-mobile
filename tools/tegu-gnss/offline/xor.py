import sys,re
for l in sys.stdin:
    h=re.findall(r"0x([0-9a-f]+)",l); n=l.split("len=")[1].strip()
    f=int(h[0],16);c=int(h[1],16);x=f^c
    print("%08x %08x xor=%08x bits=%d len=%s"%(f,c,x,bin(x).count("1"),n))
