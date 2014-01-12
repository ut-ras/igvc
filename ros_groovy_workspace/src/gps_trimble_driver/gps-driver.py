#!/usr/bin/env python3.3

import socket 
import argparse
import struct
import pprint
import rospy
import nav_msgs 

parser = argparse.ArgumentParser(description="Trimble TCP driver")
parser.add_argument('ip_address', type=str)
parser.add_argument('port_number', type=int)
args = parser.parse_args()

topic = topics.Publisher('limu_data',nav_msgs.Odometry)
trimble = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
trimble.connect((args.ip_address,args.port_number))
pp = pprint.PrettyPrinter()

typeJumpTable = { 0x01 : (['time','week','svs','flags','init_num'],">IHBHB"),
                  0x02 : (['latitude','longitude','height'],">ddd"),
                  0x03 : (['X','Y','Z'],">ddd"),
                  0x08 : (['velocityFlags','velocity','heading','up'],">Bfff"),
                  0x09 : (['PDOP','HDOP','VDOP','TDOP'],">ffff"),
                  0x0B : (['PositionRMS','VCVxx','VCVxy','VCVxz','VCVyy','VCVyz','VCVzz','UnitVariance','numEpoch'],">ffffffffH"),
                  0x0C : (['PositionRMS','sigma-E','sigma-N','cov-EN','sigma-UP','sigma-Major','sigma-Minor','Orient','UnitVarience','numEpotch'],">fffffffffH")
}
    
packetHash = {}

while True:
    header  = trimble.recv(7)
    stx, status, type, length, trans_number, page_index, max_page_index = header
    assert stx == 0x02
    assert type == 0x40
    while length > 3 :
        data = trimble.recv(2)
        record_type , record_length = data
        record_data = trimble.recv(record_length)
        if record_type in typeJumpTable :
            assert record_length == struct.calcsize(typeJumpTable[record_type][1])
            for name,value in zip(typeJumpTable[record_type][0],
                                  struct.unpack_from(typeJumpTable[record_type][1],record_data)) :
                packetHash[name] = value
        else :
            print("unknown record type : {0}".format(record_type))
        length -= record_length + 2
    trimble.recv(2) # chksum and etx
    pp.pprint(packetHash)
