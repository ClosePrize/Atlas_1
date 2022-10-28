from pymavlink import mavutil, mavwp          # Mustafa Çirci / Efecan Karatut

wp= mavwp.MAVWPLoader()

address = 'udpin:localhost:14551' 
vehicle= mavutil.mavlink_connection(address,baudrate=57600,autoreconnect= True)
vehicle.wait_heartbeat()
print("baglanti basarili")

def get_alt():
    message = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking= True)
    alt=message.relative_alt
    alt = alt/1000
    return alt

def takeoff(alt):
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    while True: 
        current_alt= get_alt()
        if current_alt< alt:
            print(f"Anlik irtifa {current_alt}")
        elif current_alt >=  alt:
            print("Istenilen irtifaya ulasildi ")
            break

def add_mission(seq, delay, lat,lon,alt):
    frame= mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(vehicle.target_system, vehicle.target_component,
    seq,
    frame,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,delay,0,0,0,lat,lon,alt))

    vehicle.waypoint_clear_all_send()
    vehicle.waypoint_count_send(wp.count())
    for i in range (wp.count()):
        msg= vehicle.recv_match(type=["MISSION_REQUEST"], blocking= True)
        vehicle.mav.send(wp.wp(msg.seq))
        print("Sending waypoints {0}".format(msg.seq))

def go_to(lat,lon,alt):
    vehicle.mav.mission_item_send(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,2,0,0,0,0,0,lat,lon,alt)

def back_to_home(seq):
    frame= mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(vehicle.target_system, vehicle.target_component,
    seq,
    frame,
    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,0,0,0,0,0,0,0,0,0))
    
    vehicle.waypoint_count_send(wp.count())
    for i in range (wp.count()):
        msg= vehicle.recv_match(type=["MISSION_REQUEST"], blocking= True)
        vehicle.mav.send(wp.wp(msg.seq))
        print("Sending waypoints {0}".format(msg.seq))

vehicle.set_mode("GUIDED")
vehicle.arducopter_arm()
print("arac arm edildi")
takeoff(25)
'''go_to(-35.36224995, 149.16503056, 10)'''
add_mission(0, 1, 0, 0, 25)
add_mission(1, 0, -35.36296790, 149.16540030, 35)
add_mission(2, 0, -35.36292410, 149.16519640, 40)
back_to_home(3)
vehicle.set_mode("AUTO")
