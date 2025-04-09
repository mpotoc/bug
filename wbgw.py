#!/usr/bin/env python3
# -*- coding: utf8 -*-
import os
if not ('HWSERIAL' in os.environ):
    import smbus2
import struct
import json
import logging
import requests
import time
import paho.mqtt.client as mqtt
import re
import const
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
from huawei_lte_api.Connection import Connection
from huawei_lte_api.Client import Client

fileVersion = "1.0.4"

# This is the address we setup in the Arduino Program
i2c_address = 0x04
i2c_data_count = 8
i2c_payload_length = i2c_data_count * 4
i2c_wind_sensor_address = 0x15

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.DEBUG)
logger = logging.getLogger(__name__)

# if 'MACADDR' in os.environ:
#     hwserial = str(os.environ['MACADDR'])
# else:
#     macaddr = str(os.popen("sed -e 's/://g' /sys/class/net/eth0/address 2>/dev/null || sed -e 's/://g' /sys/class/net/wlan0/address 2>/dev/null").read())[:-1]

# get HW serial number
if 'HWSERIAL' in os.environ:
    # simulated
    hwserial = str(os.environ['HWSERIAL'])
    sitl = True
    uselteapi = False
else:
    # real HW
    hwserial = str(os.popen("cat /proc/cpuinfo | grep Serial | cut -d ' ' -f 2").read())[:-1]
    sitl = False
    bus = smbus2.SMBus(1)


systeminfo = {}
systeminfo['osrelease'] = str(os.popen("cat /etc/os-release | grep PRETTY_NAME | cut -d '=' -f 2").read())[:-1].replace('"','')
systeminfo['fileVersion'] = fileVersion
systeminfo['filehash'] = str(os.popen("sha1sum /opt/wb_gateway/wbgw.py | cut -d ' ' -f 1").read())[:-1]
systeminfo['filedate'] = str(os.popen("stat -c %y /opt/wb_gateway/wbgw.py").read())[:-1]

if not sitl:
    try:
        lteapiconn = Connection('http://192.168.8.1/', timeout=10)
        lteapi = Client(lteapiconn)
        uselteapi = True
    except:
        uselteapi = False
        pass

# MQTT Stuff
mqtt_client_id='wb_{}'.format(hwserial)
mqtt_server='app.smartmark.team'
mqtt_port=1883
mqtt_transport='tcp'
mqtt_user='device'
mqtt_password='ecived369'
mqtt_topic_root = 'waterbug/{}/'.format(hwserial)


# connect to APM
if 'APMHOST' in os.environ:
    # simulated
    connection_string = 'tcp:{}:5760'.format(os.environ['APMHOST'])
    baud = None
else:
    # real HW
    connection_string = '/dev/ttyAMA0'
    baud = 38400

vehicle = connect(connection_string, baud=baud)

arm_command = -1
light_command = -1
light_timer = -1
horn_command = -1
horn_timer = -1

# mission handling
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
current_sequence = -1

gotoIsExecuting = False
gotoStoredData = {"execute":False, "client":None, "coords":"0,0"}

def gotoCommandStore(client, coords):
    global gotoStoredData
    gotoStoredData["execute"] = True
    gotoStoredData["client"] = client
    gotoStoredData["coords"] = coords

def gotoCommandClear():
    global gotoStoredData
    gotoStoredData["execute"] = False

def gotoCommandExecute(client, coords):
    global gotoIsExecuting
    gotoIsExecuting = True
    gotoCommandClear()

    logger.info('execute GOTO: {}'.format(coords))

    delimited = re.split(',',coords)

    # clear current mission
    vehicle.mode = 'HOLD'
    time.sleep(1)
    cmds.wait_ready()

    # add waypoints
    cmds.clear()
    i = 0
    lat = float(delimited[i])
    lon = float(delimited[i+1])
    while i < len(delimited)-1:
        lat = float(delimited[i])
        lon = float(delimited[i+1])
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, 0))
        logger.info(' - MAV_CMD_NAV_WAYPOINT: {},{}'.format(lat, lon))
        i += 2
    cmds.upload()
    cmds.wait_ready()
    vehicle.mode = 'AUTO'
    client.publish(mqtt_topic_root + 'apm/goto_ack', payload="{},{}".format(lat,lon), qos=0, retain=False)
    time.sleep(1)

    gotoIsExecuting = False

def updateLight():
    global light_command
    global light_timer

    if light_command == 0:
        if light_timer == 1:
            # turn light off
            setLightState(0)
            light_timer = -1
    elif light_command == 1:
        if light_timer == 1:
            # turn light on
            setLightState(1)
            light_timer = -1
    elif light_command == 2:
        if light_timer > -1:
            light_timer = (light_timer + 1) % 2
            setLightState(light_timer)

def setLightState(iState):
    if iState == 1:
        logger.debug("Light ON")
        if not sitl:
            i2cData = bytes([const.COMM_SET_LIGHT, 1])
            data = send_data(i2cData)
    else:
        logger.debug("Light OFF")
        if not sitl:
            i2cData = bytes([const.COMM_SET_LIGHT, 0])
            data = send_data(i2cData)

def updateHorn():
    global horn_command
    global horn_timer

    if horn_command == 0:
        if horn_timer == 1:
            # turn horn off
            setHornState(0)
            horn_timer = -1
    elif horn_command > 0:
        if horn_timer > -1:
            horn_timer = horn_timer - 1
            setHornState(horn_timer)
        if horn_timer < 0:
            horn_command = 0
            horn_timer = -1

def setHornState(iState):
    # Horn honks 1 sec and waits 2 seconds.
    if iState > 0:
        iState = iState % 3
    if iState == 2:
        logger.debug("Horn ON")
        if not sitl:
            data = send_data([const.COMM_SET_HORN, 1])
    elif iState == 1:
        logger.debug("Horn OFF")
        if not sitl:
            data = send_data([const.COMM_SET_HORN, 0])

def updateArm():
    global arm_command

    if arm_command == 0:
        # arm
        setArm(0)
    elif arm_command == 1:
        # disarm
        setArm(1)
    arm_command = -1

def setArm(iState):
    if iState == 1:
        logger.info('ARMING...')
        if not sitl:
            data = send_data([const.COMM_ARM_MOTORS, 1])
    else:
        logger.info('disarming...')
        if not sitl:
            data = send_data([const.COMM_ARM_MOTORS, 0])

def check_connectivity():
	connection = None
	try:
		r = requests.get("https://google.com")
		r.raise_for_status()
		logger.info("[ConnectionCheck] internet online!")
		connection = True
	except:
		logger.info("[ConnectionCheck] not online yet...")
		connection = False
	finally:
		return connection

def mqtt_log(client, userdata, level, buf):
    logger.info("[MQTT] log: ",buf)

def mqtt_connect(client, userdata, flags, rc):
    logger.info('[MQTT] connected.')

def mqtt_disconnect(client, userdata, flags, rc):
    logger.info('[MQTT] connection lost...')
    mc.reconnect()

def mqtt_msg(client, userdata, message):
    global arm_command
    global light_command
    global light_timer
    global horn_command
    global horn_timer

    logger.info('[' + message.topic +'] ' + str(message.payload.decode("utf-8")))
    if message.topic == mqtt_topic_root + 'apm/reboot':
        if(str(message.payload.decode("utf-8")) == '1'):
            logger.info('rebooting APM...')
            vehicle.reboot()

    if message.topic == mqtt_topic_root + 'apm/arm':
        if(str(message.payload.decode("utf-8")) == '1'):
            logger.info('ARMING...')
            vehicle.armed = True
            arm_command = 1
        if(str(message.payload.decode("utf-8")) == '0'):
            logger.info('disarming...')
            vehicle.armed = False
            arm_command = 0

    if message.topic == mqtt_topic_root + 'apm/setmode':
        gotoCommandClear()
        vehicle.mode = VehicleMode(str(message.payload.decode("utf-8")))

    if message.topic == mqtt_topic_root + 'apm/goto':
        m = str(message.payload.decode("utf-8"))

        logger.info('received GOTO: {}'.format(m))

        gotoCommandStore(client, m)

    if message.topic == mqtt_topic_root + 'apm/simplegoto':
        gotoCommandClear()
        m = str(message.payload.decode("utf-8"))
        lat = float(m.split(',')[0])
        lon = float(m.split(',')[1])
        vehicle.simple_goto(LocationGlobal(lat,lon,0))

    if message.topic == mqtt_topic_root + 'apm/getparam':
        m = str(message.payload.decode("utf-8"))
        rbval = "{}={}".format(m,vehicle.parameters[m])
        print(rbval)
        client.publish(mqtt_topic_root + 'apm/param', payload=rbval, qos=0, retain=False)

    if message.topic == mqtt_topic_root + 'apm/setparam':
        m = str(message.payload.decode("utf-8"))
        param = str(m.split('=')[0])
        val = float(m.split('=')[1])
        vehicle.parameters[param] = val
        rbval = "{}={}".format(param,vehicle.parameters[param])
        print(rbval)
        client.publish(mqtt_topic_root + 'apm/param', payload=rbval, qos=0, retain=False)

    if message.topic == mqtt_topic_root + 'pi/sethorn':
        m = str(message.payload.decode("utf-8"))
        param = str(m.split('=')[0])
        val = int(m.split('=')[1])
        horn_command = val
        horn_timer = 1
        # horn_command means how many times should buoy honk (1 sec honk, 2 sec pause)
        if horn_command > 0:
            horn_timer = horn_command * 3
        client.publish(mqtt_topic_root + 'pi/horn', payload=horn_command, qos=0, retain=False)
        logger.info('Horn command:' + str(horn_command))

    if message.topic == mqtt_topic_root + 'pi/setlight':
        m = str(message.payload.decode("utf-8"))
        param = str(m.split('=')[0])
        val = int(m.split('=')[1])
        light_command = val
        light_timer = 1
        client.publish(mqtt_topic_root + 'pi/light', payload=light_command, qos=0, retain=False)
        logger.info('Light command:' + str(light_command))

    if message.topic == mqtt_topic_root + 'pi/reboot':
        if(str(message.payload.decode("utf-8")) == '1'):
            # os.popen('shutdown -r now')
            os.popen('echo s>/proc/sysrq-trigger')
            os.popen('echo u>/proc/sysrq-trigger')
            os.popen('echo b>/proc/sysrq-trigger')

    if message.topic == mqtt_topic_root + 'pi/devexec':
        m = str(message.payload.decode("utf-8"))
        x = str(os.popen(m).read()[:-1])
        client.publish(mqtt_topic_root + 'pi/devexec_out', payload=x, qos=0, retain=False)

    if message.topic == mqtt_topic_root + 'pi/phonehome':
        if(str(message.payload.decode("utf-8")) == '1'):
            logger.info('[PhoneHome] starting VPN...')
            os.popen('openvpn --config /opt/wb_gateway/vpn/waterBug.conf --daemon')
        if(str(message.payload.decode("utf-8")) == '0'):
            logger.info('[PhoneHome] killing VPN...')
            os.popen('killall openvpn')
            client.publish(mqtt_topic_root + 'pi/phonehome_ip', payload='terminated', qos=0, retain=False)

def mqtt_init():
    logger.info('[MQTT] INIT...')
    global mqtt_client_id, mqtt_server, mqtt_port, mqtt_transport, mqtt_user, mqtt_password, mqtt_topic_root
    mc = mqtt.Client(client_id=mqtt_client_id, clean_session=False, userdata=None, transport=mqtt_transport)
    mc.username_pw_set(mqtt_user, password=mqtt_password)
    #mc.on_log=mqtt_log
    mc.on_connect=mqtt_connect
    mc.on_disconnect=mqtt_disconnect
    mc.on_message=mqtt_msg
    logger.info('[MQTT] checking connectivity...')
    while 1:
        if check_connectivity():
            break
        time.sleep(10)
    logger.info('[MQTT] connecting...')
    mc.connect(mqtt_server, port=mqtt_port, keepalive=30)
    mc.subscribe(mqtt_topic_root + '#', qos=0)
    return mc


# I2C communication
def get_data():
    if not sitl:
        return bus.read_i2c_block_data(i2c_wind_sensor_address, 0x00, i2c_data_count)
    else:
        return [0,0,0,0,0,0,0]

def send_data(i2cData, aResponseLength = 0):
    if not sitl:
        try:
            bus.write_i2c_block_data(i2c_address, 0x00, i2cData)
            if aResponseLength == 0:
                aResponseLength = len(i2cData)
            aResult = bus.read_i2c_block_data(i2c_address, 0x00, aResponseLength)
            return aResult
        except:
            return 0

def get_float(data, index):
    bytes = data[4*index:(index+1)*4]
    # print(bytes)
    return(struct.unpack('f', bytearray(bytes)))[0]

def main():
    global light_command
    global horn_command

    logger.info('[main] starting...')
    mc = mqtt_init()
    mc.publish(mqtt_topic_root + 'status', payload='buoy alive...', qos=0, retain=False)

    vehicle.wait_ready('autopilot_version')
    mc.publish(mqtt_topic_root + 'apm/version', payload=str(vehicle.version), qos=2, retain=False)
    mc.publish(mqtt_topic_root + 'pi/systeminfo', payload=json.dumps(systeminfo), qos=2, retain=False)
    mc.loop_start()

    @vehicle.on_message('MISSION_ITEM_REACHED')
    def on_waypoint(self, name, message):
        mc.publish(mqtt_topic_root + 'apm/mav_msg', payload=name, qos=0, retain=False)
        logger.info(name + ' {}/{}'.format(vehicle.commands.next, vehicle.commands.count))
        if (vehicle.commands.next >= vehicle.commands.count):
            vehicle.mode = 'LOITER'
            logger.info('Mission ended.')

    @vehicle.on_message('MISSION_CURRENT')
    def on_mission_current(self, name, mission_current):
        global current_sequence
        if (current_sequence != mission_current.seq):
            current_sequence = mission_current.seq
            mc.publish(mqtt_topic_root + 'apm/mis_seq', payload=mission_current.seq, qos=0, retain=False)
            logger.info('MISSION_CURRENT: {}/{}'.format(mission_current.seq, vehicle.commands.count))

    # @vehicle.on_message('*')
    # def on_message(self, name, message):
    #     info = message.to_json()
    #     # mc.publish(mqtt_topic_root + 'apm/nav_status', payload=name, qos=0, retain=False)
    #     logger.debug(info)

    # Default motor throttle is 70%
    vehicle.parameters['MOT_THR_MAX'] = 70

    counter = 0

    dataVoltage = 0.0
    dataCurrentL = 0.0
    dataCurrentR = 0.0
    dataTemperature = 0.0
    dataStatus = 0
    isFuseShutdown = 0
    intervalDivider = 4 # second is split to

    while 1:
        if counter % intervalDivider == 0: # offset I2C commands
            try:
                # data = get_data()
                data = send_data([const.COMM_GET_TELEMETRY, 1], i2c_payload_length)
                # print(data)
                dataVoltage = get_float(data, 0)
                dataCurrentL = get_float(data, 1)
                dataCurrentR = get_float(data, 2)
                dataTemperature = get_float(data, 3)
                dataStatus = get_float(data, 4)
                isFuseShutdown = get_float(data, 6)
                soc = get_float(data, 7)

                #wind_data = get_data()
                #wind_speed = get_float(wind_data, 2) * 100
                #wind_direction = get_float(wind_data, 4) * 100
            except:
                print('I2C data read exception')
                # dataVoltage = 0.0
                # dataCurrentL = 0.0
                # dataCurrentR = 0.0
                # dataTemperature = 0.0
                # dataStatus = 0

        if counter % intervalDivider == 1: # offset I2C commands
            updateLight()
        if counter % intervalDivider == 2: # offset I2C commands
            updateHorn()
        if counter % intervalDivider == 3: # offset I2C commands
            updateArm()

        if counter % (3 * intervalDivider) == 0: # report telemetry every 3s
            telem = "{},{},{},{},{},{},{:.3f},{},{:.1f},{:.1f},{:.1f},{:.1f},{},{},{},{},{},{:.1f}".format(
                vehicle.armed,
                vehicle.mode.name,
                vehicle.location.global_frame.lat,
                vehicle.location.global_frame.lon,
                vehicle.location.global_frame.alt,
                vehicle.heading,
                vehicle.groundspeed,
                vehicle.gps_0.satellites_visible,
                # vehicle.battery.voltage,
                # vehicle.battery.current,
                dataVoltage,
                dataCurrentL,
                dataCurrentR,
                dataTemperature,
                vehicle.parameters['MOT_THR_MAX'],
                vehicle.parameters['LOIT_TYPE'],
                light_command,
                horn_command,
                isFuseShutdown,
                soc#,
                #wind_speed,
                #wind_direction
                )
            mc.publish(mqtt_topic_root + 'apm/telem', payload=telem, qos=0, retain=False)

        if counter == 0: # report lte usage data every 30s
            if uselteapi:
                # ltesig = lteapi.device.signal()
                ltemonstat = lteapi.monitoring.status()
                lteinfo = lteapi.device.information()
                ltetrfc = lteapi.monitoring.traffic_statistics()
                ltetelem = "{},{},{},{},{},{},{},{},{},{}".format(
                    lteinfo['workmode'],
                    # ltesig['rssi'],
                    '{} / 5'.format(ltemonstat['SignalIcon']),
                    ltetrfc['CurrentUploadRate'],
                    ltetrfc['CurrentDownloadRate'],
                    ltetrfc['CurrentConnectTime'],
                    ltetrfc['CurrentUpload'],
                    ltetrfc['CurrentDownload'],
                    ltetrfc['TotalUpload'],
                    ltetrfc['TotalDownload'],
                    ltetrfc['TotalConnectTime'],
                )
                mc.publish(mqtt_topic_root + 'pi/lte-telem', payload=ltetelem, qos=0, retain=False)
        counter = (counter + 1) % (30 * intervalDivider)

        time.sleep(1/intervalDivider)

        global gotoIsExecuting
        global gotoStoredData
        if gotoStoredData["execute"] and not gotoIsExecuting:
            gotoCommandExecute(gotoStoredData["client"], gotoStoredData["coords"])



if __name__ == "__main__":
    main()
