# JohnDeere_Navigation_Tractor
Advanced Embedded Navigation System for John Deere tractors.


CANABLE DEBUG COMMANDS
'''bash 
sudo slcand -o -c -s6 <CAN_PORT> slcan0
sudo ip link set slcan0 up
ip -details link show slcan0
candump slcan0
'''

Notes:

ESC = 50hz 

Clock = 200hz

psc = 200-1


