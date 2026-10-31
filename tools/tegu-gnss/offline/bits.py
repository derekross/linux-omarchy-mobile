import sys,re,ast,collections
c=collections.Counter(); first=collections.Counter()
for l in open(sys.argv[1],errors='replace'):
    m=re.search(r'read\(\d+<[^>]*gnss_ipc>, "(.*)", \d+\) = (\d+)',l)
    if not m: continue
    b=ast.literal_eval('b"'+m.group(1)+'"')
    bits=''.join(f'{x:08b}' for x in b)
    offs=[i for i in range(len(bits)-16) if bits[i:i+16]=='0110100001000011']
    for o in offs: c[o%8]+=1
    if offs: first[(offs[0]%8, offs[0]//8)]+=1
print('preamble bit offset mod 8:',sorted(c.items()))
print('first preamble (bit, byte):',first.most_common(12))
