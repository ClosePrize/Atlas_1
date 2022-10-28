from pymavlink import mavutil 

address = 'udpin:localhost:14551' #simulasyon
# address= '/dev/ttyACM0' #pixhawk usb
# address= '/dev/ttyTHS1' #pixhawk telem2 baudrate= 115200
vehicle= mavutil.mavlink_connection(address,baudrate=57600,autoreconnect= True)
vehicle.wait_heartbeat()
print("baglanti basarili")

def get_alt():
    message = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking= True)
    alt=message.relative_alt
    alt = alt/1000
    return al