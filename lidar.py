import numpy as np
import matplotlib.pyplot as plt
import multiprocessing
import serial
import ctypes
import time
import traceback, sys

com_port = "/dev/ttyUSB0" # This is on mac os x. For Windows/Linux: 5 == "COM6" == "/dev/tty5"
baudrate = 115200
init_level = 0
lidarData = [[100000000, 0] for i in range(360)] #A list of 360 elements Angle, Distance , quality


def plot_radial(q):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar' )
    ax.set_ylim(0, 4000)

    max_points = 360
    theta = np.linspace(0, 2 * np.pi, max_points)
    data = np.random.rand(max_points) * 5.
    sc = ax.scatter(theta, data, cmap='hsv')

    while True:
        data = np.array(q.get())
        sc.set_offsets(np.c_[theta, data[:,0]])
        plt.draw()
        plt.pause(0.01)

def update_view(packet):
    """Updates the view of a sample.
Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
"""
    angle = packet.get_index() * 4

    for i, mesure in enumerate(packet.data):
        lidarData[angle] = [mesure.distance, mesure.signal_strength]
        angle += 1

    if angle  >= 359:
        q.put(lidarData)


class Data(ctypes.LittleEndianStructure):
    '''describe one mesure'''
    _fields_ = [
        ("distance", ctypes.c_int32, 14),
        ("strength_warning", ctypes.c_int32, 1),
        ("invalid_data", ctypes.c_int32, 1),
        ("signal_strength", ctypes.c_int32, 16),
    ]
    _pack_ = 1

class Packet(ctypes.LittleEndianStructure):
    '''describe packet of lidar'''
    _fields_ = [
        ("start", ctypes.c_uint8),
        ("index", ctypes.c_uint8),
        ("speed", ctypes.c_uint16),
        ('data', Data * 4),
        ("cs", ctypes.c_uint16),
    ]
    _pack_ = 1
    cs_buf = None

    def get_index(self):
        '''get index of packet'''
        return self.index - 0xA0

    def get_speed(self):
        '''get speed of motor'''
        return self.speed / 64.0

    def get_checksumm(self):
        # compute the checksum on 32 bits
        chk32 = 0
        for d in self.cs_buf:
            chk32 = (chk32 << 1) + d

        # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
        checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
        checksum = checksum & 0x7FFF # truncate to 15 bits
        return int(checksum)

    def chech_cs(self):
        return self.get_checksumm() == self.cs

buff = ctypes.create_string_buffer(ctypes.sizeof(Packet))
packet = Packet.from_buffer(buff)
packet.cs_buf = (ctypes.c_uint16 * 10).from_buffer(packet)


def read_Lidar():
    nb_errors = 0
    init_level = 0
    while True:
        try:
            if init_level == 0:
                buff[0] = ser.read()
                # start byte
                if packet.start == 0xFA :
                    init_level = 1
            elif init_level == 1:
                # position index
                buff[1] = ser.read()
                if packet.index >= 0xA0 and packet.index <= 0xF9:
                    init_level = 2
                elif buff[1] != 0xFA:
                    init_level = 0
            elif init_level == 2 :
                # fill buffer
                for i, b in enumerate(ser.read(20)):
                    buff[i + 2] = b

                # print(packet.start, packet.index, packet.speed)
                # verify that the received checksum is equal to the one computed from the data
                if packet.chech_cs():
                    # print('speed:', packet.get_speed())
                    update_view(packet)
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors +=1
                    print('errors speed:', packet.get_speed())
                init_level = 0 # reset and wait for the next packet
            else: # default, should never happen...
                init_level = 0
        except KeyboardInterrupt:
            print ("KeyBoard interrupt")
            job_for_another_core.terminate()
            job_for_another_core.join()
            break
        except Exception:
            traceback.print_exc(file=sys.stdout)

if __name__ == "__main__":

    ser = serial.Serial(com_port, baudrate)
    q = multiprocessing.Queue()
    job_for_another_core = multiprocessing.Process(target=plot_radial,args=((q,)))
    job_for_another_core.start()
    read_Lidar()
    ser.close()
    os._exit(0)
