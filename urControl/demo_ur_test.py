from socket import socket
import time

# connect ur_robot
print("connector ur_robot")
time.sleep(0.5)

# connect end-effector
sk = socket()
sk.connect(('127.0.0.1', 6666))
print("connect end-effector")

# move to p1
print("moving to p1")
time.sleep(5)
print("position reached: p1")

# main loop
while True:
    # current position: p1
    button = input("conduct grinding ? >> (y/n) ")
    if button in ['q', 'quit', 'break', 'n', 'N']:
        break

    print("enter the process")
    # move to p2
    print("moving to p2")
    time.sleep(5)
    print("position reached: p2")

    # ==========tcp===============
    # send "start" to end-effector
    msg_s = "start"
    sk.send(msg_s.encode('utf-8'))
    print("send '%s' to end-effector" % msg_s)
    print("position module enable")
    # wait end-effector to load
    msg_r = sk.recv(1024).decode('utf-8')
    print("receive '%s' from end-effector" % msg_r)
    if msg_r != "load#":
        break
    print("position module disable")
    print("force module enable")
    print("start the grinding!")
    # ============================

    # start the main grinding trajectory
    for i in range(3, 8):
        print("moving to p%d" % i)
        time.sleep(6)
        print("position reached: p%d" % i)

    # ==========tcp===============
    # send "end" to end-effector
    msg_s = "end##"
    sk.send(msg_s.encode('utf-8'))
    print("send '%s' to end-effector" % msg_s)
    # wait end-effector to unload
    msg_r = sk.recv(1024).decode('utf-8')
    print("receive '%s' from end-effector" % msg_r)
    if msg_r != "uload":
        break
    print("force module disable")
    print("end the grinding!")
    # ============================

    # move to p8
    print("moving to p8")
    time.sleep(5)
    print("position reached: p8")
    # move to p1
    print("moving to p1")
    time.sleep(5)
    print("position reached: p1")

print("quit the process")
# send "break" to end-effector
msg_s = "break"
sk.send(msg_s.encode('utf-8'))
print("send '%s' to end-effector" % msg_s)

# move to p0
print("moving to p0")
time.sleep(5)
print("position reached: p0")

# close tcp to end-effector and ur_robot
print("close end-effector TCP")
sk.close()
print("close ur_robot TCP")