 ###########################################################################
# 1) connect to internet
# 2) get time
# 3) report data to RPi MQTT
# 4) Send data to google sheets?
###########################################################################

DEBUG = False

if not DEBUG:
    import machine
    import utime
    import ntptime
    import network
    import esp
    from dht import DHT11
    import ubinascii
    from umqtt.simple import MQTTClient
    #from umqtt.robust import MQTTClient
else:
    import time


UTC_OFFSET = 3   # hours of differenc between UTC and local (Jerusalem) time
COMMANDS_PORT = 5641  # port at RPi to send commands
RPi_HOST = "10.0.0.17"
TEMPERATURE_LOG_DELAY = 5*60    # minimal number of seconds to wait/
                            # between subsequent temperature/humidity Logging
TEMPERATURE_SAMPLE_DELAY = 5
MOTION_LOG_DELAY = 5*60   # minimal number of seconds to wait/
                            # between subsequent motion Logging

CONFIG = {
    "broker": RPi_HOST,
    "client_id": b"chipa_ESP8266"
}


def toggleGPIO(p):
    p.value(not p.value())


def GPIO_On(p):
    p.value(1)


def GPIO_Off(p):
    p.value(0)


def getHumidityTemp():
    try:
        dhtSensor.measure()
        temper = dhtSensor.temperature()
        humid = dhtSensor.humidity()
    except:
        (temper, humid) = (None, None)

    return temper, humid


def getDateTime():
    gotTime = False
    if DEBUG:
        return time.localtime()
    else:
        utime.sleep(5)  # Sleep for 5 seconds, to make sure WiFi is connected
        if sta_if.isconnected():
            print("Connected to internet, setting time from NTP")
            ntptime.settime()
        t = rtc.datetime()
        print("RTC time is: ", t)
        year = t[0]
        print("year is: ",year)
        gotTime = year > 2016
        if gotTime:
            print(t)
        else:
            print("Could not get time")
            machine.reset()
        return gotTime, t 

        
def getRPiTime():
    import socket
    addr = socket.getaddrinfo(RPi_HOST, COMMANDS_PORT)[0][-1]
    s = socket.socket()
    print("Connecting to RPi: ", RPi_HOST)
    try:
    #TODO Something wrong here, the socket does not connect. \
    # Important, after failed connection need to open a new socket.
        s.connect(addr)
    except:
        print("Error connecting to RPi server")
        s.close()
        return None

    if DEBUG: 
        print("Handshaking with RPi")
    s.send("time".encode("utf8"))
    time.sleep(3)
    data = s.recv(1024)
    payload = data.decode("utf8")
    if data:
        print('commad received: ' + payload)
    else:
        print('No data received')
    s.close()
    return None


def pin_interrupt():
    global motionDetected, lastMotionTime
    lastMotionTime = utime.time()
    motionDetected = True


def handleDHT(temper, humid):
    global lastTemperatureLogTime
    currentTime = utime.time()
    print ("Temperature[degC]: ", temper, ", Humidity[%]: ", humid, 
           "time[s]: ", currentTime)
    if (lastTemperatureLogTime == 0) or \
            currentTime - lastTemperatureLogTime > TEMPERATURE_LOG_DELAY:
        print ("LOGGING temperature and humidity at: ", currentTime)
        lastTemperatureLogTime = currentTime
        pushSample(temper,"Temperature_Chipa")
        pushSample(humid,"Humid_Chipa")


def handlePIR():
    global lastMotionTime, lastMotionLogTime
    currentTime = utime.time()
    print ("Movement detected at: ", lastMotionTime)
    if (lastMotionLogTime == 0) or \
            (currentTime - lastMotionLogTime) > MOTION_LOG_DELAY:
        print ("LOGGING motion at: ", currentTime)
        lastMotionLogTime = currentTime
        pushSample(lastMotionTime,"Motion_Chipa")


def pushSample(sample, topic):
    global client
    client.publish(topic, str(sample))


def main():
    global motionDetected, lastMotionTime
    while True:
        #gotTime, curr_tm = getDateTime()  # get time 
        utime.sleep(TEMPERATURE_SAMPLE_DELAY)  # Sleep at least one second between each measurement

        # sample the temperature and humidity sensor
        temper, humid = getHumidityTemp()
        handleDHT(temper, humid)

        # if detected motion on PIR sensor
        if motionDetected:   
            motionDetected = False
            handlePIR()

#Generic Init
if not DEBUG:
    print ("Initializing...")
    rtc = machine.RTC()
    sta_if = network.WLAN(network.STA_IF)

    motionDetected = False
    lastMotionTime = 0
    lastMotionLogTime = 0

    lastTemperatureLogTime = 0
    dhtSensor = DHT11(machine.Pin(4))  # D2 pin on NodeMCU board. DHT signal pin
    pirSig = machine.Pin(15, machine.Pin.IN)          # D8 pin on NodeMCU. PIR signal pin
    pirSig.irq(handler=lambda p: pin_interrupt(), trigger=machine.Pin.IRQ_RISING)

    #MQTT configs
    client = MQTTClient(CONFIG['client_id'], CONFIG['broker'])
    try:
        utime.sleep(3)
        client.connect()
        print("Connected to {}".format(CONFIG['broker']))
        print ("Initializing Done")
    except:
        print("failed to connect to RPi. waiting 5sec and resetting")
        utime.sleep(5)
        machine.reset()

if DEBUG:
    if __name__ == "__main__":
        main() 
else:
    main()

#TODO
# Add logging
# Add ADC reading
