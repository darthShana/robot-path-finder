import csv
import time
import paho.mqtt.client as mqtt #import the client1


broker_address = "localhost"
client = mqtt.Client("P1") #create new instance

print("connecting to broker")
client.connect(broker_address) #connect to broker

with open('raw-data.txt') as csvfile:
    readCSV = csv.reader(csvfile, delimiter=' ')
    for row in readCSV:
        time.sleep(.5)
        print(row[1], row[3], row[5],)
        client.publish("robot/sensors/orbslam", str(row[1]) + "," + str(row[2]) + "," + str(row[3]) + "," + str(row[4]) + "," + str(row[5]))

