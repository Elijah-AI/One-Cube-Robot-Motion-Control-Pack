import socket,struct,subprocess,sys,time,signal
s=socket.socket();s.bind(('127.0.0.1',0));port=s.getsockname()[1];s.close()
p=subprocess.Popen([sys.argv[1],str(port)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
peers=[]
def connect():
 s=socket.create_connection(('127.0.0.1',port),timeout=.8);peers.append(s);return s
def packet(tx,fn,a,b):return struct.pack('>HHHBBHH',tx,0,6,1,fn,a,b)
def exact(s,n):
 out=b''
 while len(out)<n:
  chunk=s.recv(n-len(out));assert chunk,'unexpected EOF';out+=chunk
 return out
def response(s):
 header=exact(s,7);size=struct.unpack('>H',header[4:6])[0];return header+exact(s,size-1)
def read(s,addr):
 s.sendall(packet(17,3,addr,1));r=response(s);assert r[7:9]==b'\x03\x02',r;return struct.unpack('>H',r[-2:])[0]
try:
 deadline=time.monotonic()+5
 while True:
  line=p.stdout.readline()
  assert line, 'test server failed to start'
  if 'READY' in line:break
  assert time.monotonic()<deadline
 idle=connect();slow=connect();slow.sendall(packet(1,6,9,1)[:5])
 c=connect();start=time.monotonic()
 # Persistent idle and half-frame clients must not starve another connection.
 assert read(c,10009)==42
 c.sendall(packet(2,6,9,0));assert response(c)==packet(2,6,9,0)
 c.sendall(packet(3,6,9,1));assert response(c)==packet(3,6,9,1)
 assert read(c,10010)==2 and read(c,10011)==1
 assert time.monotonic()-start<.8
 # Fragmented requests and two requests in one TCP segment.
 f=connect();q=packet(5,6,13,7);f.sendall(q[:8]);time.sleep(.03);f.sendall(q[8:]);assert response(f)==q
 c.sendall(packet(6,3,10009,1)+packet(7,3,10009,1));assert response(c)[-2:]==b'\x00*';assert response(c)[-2:]==b'\x00*'
 # A disconnected partial writer must never execute a callback later.
 dead=connect();dead.sendall(packet(8,6,9,1)[:10]);dead.close();time.sleep(.05)
 assert read(c,10010)==3
 c.sendall(packet(9,3,65535,1));assert response(c)[7:]==b'\x83\x02'
 c.sendall(packet(10,6,65535,1));assert response(c)[7:]==b'\x86\x02'
 # Other clients still work after malformed MBAP and partial-frame expiration.
 bad=connect();bad.sendall(struct.pack('>HHHBB',11,0,65535,1,6));time.sleep(.05)
 assert bad.recv(1)==b''
 time.sleep(1.05);assert slow.recv(1)==b''
 for _ in range(10):connect()
 for _ in range(15):assert read(c,10009)==42
 start=time.monotonic();p.send_signal(signal.SIGINT);p.wait(timeout=2);assert p.returncode==0
 assert time.monotonic()-start<1
 print('PASS: concurrent idle/fragmented clients, FC03/FC06, framing, expired/disconnected writes, active register isolation, bounded shutdown')
finally:
 for s in peers:s.close()
 if p.poll() is None:p.kill();p.wait()
