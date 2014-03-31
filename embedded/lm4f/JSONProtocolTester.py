import serial, json
import time, sys
import atexit
import traceback

SERIAL_LINE = "/dev/lm4f"
BAUD_RATE = 115200

def printError(s):
    print "ERROR: ", s

def onClose():
    connection.close()

atexit.register(onClose)

# initialize connection
connection = serial.Serial(
    port = SERIAL_LINE, 
    baudrate = BAUD_RATE, 
    timeout = .2, 
    writeTimeout = 1
    )

def processData(data):
    try:
        count = int(data["SPLMdebug"])
    except:
        printError("couldn't receive values for expected keys") 
        return    
        
    print "counter received: %d" % count

def parseLine(s):
    try :
        data = json.loads(s.strip())
        print s
    except:
        print "ERROR: problem while parsing JSON: ", s.strip()
        return

    processData(data)

counter = 0

connection.flush()

while True:
    try:
        # send message
        msg = json.dumps({"SPLM":str(counter), "SVLX":str(counter + 10)})
        print "sent: %s" % msg
        
        connection.flushOutput()
        connection.write(msg + '\n')
        
        # read response
        line = connection.readline()
        connection.flush()
        if len(line) == 0:
            printError("ERROR: timeout")
        else:
            print line
            #parseLine(line)

        counter += 1
        time.sleep(0.1)
    except:
        print 'Exception caught in Main loop'
        traceback.print_exc(file=sys.stdout)
        break

print 'Connection Closing'
connection.close()
