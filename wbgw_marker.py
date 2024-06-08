#!/usr/bin/env python3
# -*- coding: utf8 -*-

# apt-get install gpsd gpsd-clients
# sed -i -e 's#GPSD_OPTIONS=""#GPSD_OPTIONS="-n -G -b -F /var/run/gpsd.sock"#g' /etc/default/gpsd
# sed -i -e 's#GPSD_OPTIONS=""#DEVICES="/dev/ttyUSB0"#g' /etc/default/gpsd
# systemctl restart gpsd

import gpsd
import json
import logging
import os
import requests
import time
import paho.mqtt.client as mqtt
# from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
# from pymavlink import mavutil
from huawei_lte_api.Connection import Connection
from huawei_lte_api.Client import Client


logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.DEBUG)
logger = logging.getLogger(__name__)

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


systeminfo = {}
systeminfo['osrelease'] = str(os.popen("cat /etc/os-release | grep PRETTY_NAME | cut -d '=' -f 2").read())[:-1].replace('"','')
systeminfo['filehash'] = str(os.popen("sha1sum /opt/wb_gateway/wbgw_marker.py | cut -d ' ' -f 1").read())[:-1]
systeminfo['filedate'] = str(os.popen("stat -c %y /opt/wb_gateway/wbgw_marker.py").read())[:-1]

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


# connect to GPS
gpsd.connect()


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
    logger.info('[' + message.topic +'] ' + str(message.payload.decode("utf-8")))
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



def main():
    logger.info('[main] starting...')
    mc = mqtt_init()
    mc.publish(mqtt_topic_root + 'status', payload='markerbox alive...', qos=0, retain=False)

    while 1:
        gp = gpsd.get_current()

        telem = "{},{},{},{},{},{},{:.3f},{},{},{},{},{}".format(
            None,
            'GNSS-{}'.format(gp.mode),
            gp.lat,
            gp.lon,
            gp.alt,
            gp.track,
            gp.hspeed,
            gp.sats,
            None,
            None,
            None,
            None,
            )
        mc.publish(mqtt_topic_root + 'apm/telem', payload=telem, qos=0, retain=False)

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
        time.sleep(3)



if __name__ == "__main__":
    main()
