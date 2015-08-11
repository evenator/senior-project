import socket
import time
import threading
import Queue

port = 5556 #port to send on
ip = '192.168.1.1'
sock=socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
q = Queue.Queue();

def cmd_func():
	while True:
		time.sleep(.02)
		if not q.empty():
			print sock.sendto(q.get()+"\r", (ip, port))," bytes sent"
			q.task_done()
		sock.sendto("AT*COMWDG=1\r", (ip, port))

cmd_thread = threading.Thread(target = cmd_func)
cmd_thread.daemon = True
cmd_thread.start()
cmd = '\r'
while len(cmd):
	cmd = cmd.replace('^^','\r')
	q.put(cmd)
	cmd = raw_input('-->')
sock.close()

