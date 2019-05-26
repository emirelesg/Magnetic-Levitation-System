from queue import Queue
from time import time, sleep
from os import getpid
import threading
import struct
import serial

class Controller(threading.Thread):
    """
        Thread for handling the communication to the Tiva C. 
    """
    def __init__(self, serialPort, baudRate):
        threading.Thread.__init__(self)
        self.serialPort = serialPort
        self.baudRate = baudRate
        self.daemon = True
        self.stop = threading.Event()
        self.messageQ = Queue()
        self.commandQ = Queue()
        self.currentTime = 0
        self.buffer = ''
        print(getpid(), 'Creating connection to controller...')

    def getNumber(self, value, format):
        """
            Given an array of bytes converts it to the specified format.
            Use 'f' for float.
            Use '<I' for unsigned integer.
            Use 'I' for integer.
        """

        try:
            return struct.unpack_from(format, value)[0]
        except:
            return None

    def run(self):
        """
            Main thread loop.
        """

        print(getpid(), 'Starting controller...')

        # Loop until the stop event is set.
        while not self.stop.isSet():

            # Wrap serial with a try-except event to catch serial exceptions.
            try:

                # Open serial port as controller, wrapped in a with statement for safety.
                with serial.Serial(port=self.serialPort, baudrate=self.baudRate, timeout=1) as controller:
                    
                    # Reads a single byte.
                    inByte = controller.read(1)

                    # If the byte received is a $, this means that the following 24 bytes are data.
                    if inByte == '$'.encode('ascii'):

                        # Save the time at which the values where received.
                        self.currentTime += 20 * 0.000140

                        # Read 24 bytes from the buffer and convert them to their values.
                        data = controller.read(24)
                        distance = self.getNumber(data[0:4], 'f')
                        error = self.getNumber(data[4:8], 'f')
                        dutyCycle = self.getNumber(data[8:12], 'f')
                        kp = self.getNumber(data[12:16], 'f')
                        ki = self.getNumber(data[16:20], 'f')
                        kd = self.getNumber(data[20:], 'f')

                        # Send received data back to the gui by placing a message of the queue.
                        self.messageQ.put({
                            'type': 'data',
                            'distance': distance,
                            'error': error,
                            'dutyCycle': dutyCycle,
                            'kp': kp,
                            'ki': ki,
                            'kd': kd,
                            'time': self.currentTime
                        })

                        # Clear output buffer for any characters.
                        controller.reset_output_buffer()

                    # If the received character is a < this means that the controller is requesting values.
                    elif inByte == '<'.encode('ascii'):
                        
                        # Read one more byte to confirm the request. Since the baudrate is high, a lot of random data
                        # is received. By receiving another character the request is confirmed.
                        inByte = controller.read(1)

                        # Send a message to the gui to send the most recent paramters.
                        if inByte == '>'.encode('ascii'):
                            self.messageQ.put({
                                'type': 'sendParameters'
                            })
                    
                    # Check if the gui has sent any commands.
                    while not self.commandQ.empty():

                        # Read command
                        command = self.commandQ.get()
                        self.commandQ.task_done()

                        # Check if the message is a reset request.
                        if command['type'] == 'reset':

                            # Signal a request by sending an r.
                            controller.write('r'.encode('ascii'))

                            # Reset current time.
                            self.currentTime = 0

                        # Otherwise, it must be an update to the constants.
                        else:

                            # Convert data back to bytes and send data to controller.
                            out = '$'.encode('ascii')
                            out += struct.pack('f', command['setPoint'])
                            out += struct.pack('f', command['proportional'])
                            out += struct.pack('f', command['integral'])
                            out += struct.pack('f', command['derivative'])
                            out += struct.pack('<I', command['frequency'])
                            controller.write(out)

            # Catch serial exception. This usually means that the port could not be openned.
            except serial.serialutil.SerialException:
                
                print(getpid(), 'Serial port can not be oppened...')

                # Wait before trying to open serial port again.
                sleep(1)

        print(getpid(), 'Closing connection to controller...')
