import paho.mqtt.client as mqtt
import serial 
import json 
import time
from threading import Lock


#change comport according to your system/pipico => check for new comport (device manager, arduino ide ..) when connecting pipico -> use that one
comport = 'COM8' 

mqttout = {"N2_temp_act": 0,
       "N2_heater_run": False,
       "N2_heater_power": 0,
       "ambient_temp_act": 0,
       "ambient_humidity_act": 0,
       "decon_ambient_sensor": False}

new_mqttin = False
mqttin_lock = Lock()
mqttin = {"N2_temp_set": 0,
          "N2_heater_run": False,
          "decon_ambient_sensor": False}

##################
### INIT MQTT  ###
##################

#MQTT connections config
mqttbroker = "192.168.137.4"
port = 1883
intopic = "gas_heater_controller_in"
outtopic = "gas_heater_controller_out"


### INIT MQTT COMMUNICATION ###

def msg_callback(client, userdata, message):
    global mqttin, new_mqttin, mqttin_lock 
    try:
        in_dict = json.loads(message.payload)
        mqttin_lock.acquire() #get lock, so we don't overwrite while it is accessed 
        for key in in_dict: # go through keys and copy to our mqtt input dict
            if key in mqttin:
                mqttin[key] = in_dict[key]
                new_mqttin = True
        mqttin_lock.release()
    except:
        print("Received invalid MQTT payload")

client = mqtt.Client(protocol=mqtt.MQTTv5)  # create mqtt client, make sure to use v5 protocol 
client.connect(mqttbroker, port) #connect to broker using the defined adress and port
client.loop_start() #start the background loop to process messages 
client.subscribe([(intopic, 2)]) #subscribe to the topic with QOS2 
client.message_callback_add(intopic, msg_callback) #callback when we get a message

### INIT SERIAL ###
try:
    pipico = serial.Serial(port=comport, baudrate=9600, timeout=0.1) 
except:
    print("Failed connecting to pipico via serial")
    exit()

while True:
    #Check if we got new data from mqtt 
    if new_mqttin == True:
        mqttin_lock.acquire() #get lock, so we don't access input data while it is over-written from mqtt data (not sure if needed, but better be save)
        pipico.write(bytes(json.dumps(mqttin), "utf-8")) # write data to pipico
        new_mqttin = False
        mqttin_lock.release()

    #Check if we got new data from pipico
    serialin = pipico.readline()
    if serialin:
        try:
            datain = json.loads(serialin) 
            for key in datain: #go through keys, copy into our mqtt output dict
                if key in mqttout:
                    mqttout[key] = datain[key]
            client.publish(outtopic, json.dumps(mqttout)) #send it off
            
        except:
            print("Received invalid Controller message")
        
    time.sleep(0.05)

