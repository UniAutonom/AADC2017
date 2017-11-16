#!/usr/bin/env python

#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements. See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership. The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliancea
# with the License. You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied. See the License for the
# specific language governing permissions and limitations
# under the License.
#
# https://git1-us-west.apache.org/repos/asf?p=thrift.git;a=blob;f=tutorial/py/PythonServer.py;hb=HEAD
import SocketServer
import glob
import sys
import os
from PIL import Image
import io

import cv2
import numpy as np
import subprocess
import socket

from mock.mock import self

sys.path.append('gen-py')

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/lib")


from lib.ExtIf import ExtService
from lib.ExtIf import ttypes
from lib.ExtIf.ttypes import *

from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from thrift.server import TServer

from tf_demo import tf_demo_init, classify_image, load_image_from_file
from object_detection import Tutorial


class ExtServiceHandler:
    def __init__(self):
        # print('A')
        # init the tensorflow demo
        # tf_demo_init()
        Tutorial.create_graph()
        Tutorial.init_tf_session()

    def ping(self, sender):    
        return a
    
    def rawData(self, transport_def, raw_data, params):
        # get the stringbyte array with the image data
        transportOut = TTransport.TMemoryBuffer()
        protocolOut = TBinaryProtocol.TBinaryProtocol(transportOut)
        raw_data.write(protocolOut)       
        stringBytes = transportOut.getvalue()

        
        # get the image params
        transportOutParams = TTransport.TMemoryBuffer()
        protocolOutParams = TBinaryProtocol.TBinaryProtocol(transportOutParams)
        params.write(protocolOutParams)   
        imageparams = transportOutParams.getvalue()                                                                    
        
        #for debug you can print the image params here
        #print('Height: ' + str(params.height))
        #print('Width: ' + str(params.width))
        #print('bytesPerPixel: ' + str(params.bytesPerPixel))
       
        ##unfortunately we have some extra bits before the array when received in python
        ##seven bytes are inserted in the byte array, so the image header is invalid. we delete them manually        
        stringBytes = stringBytes[7:]
        # now we give the byte array with the image to the classify algorithm in tensor flow
        # we get a return list with strings and doubles
        return Tutorial.classify_image(stringBytes)
 
        # return result_list

   
if __name__ == '__main__':
    # subprocess.call(['/home/aadc/ADTF/config/start_PythonConfig.sh'])
    handler = ExtServiceHandler()
    processor = ExtService.Processor(handler)
    transport = TSocket.TServerSocket(port=1833)
    tfactory = TTransport.TBufferedTransportFactory()
    pfactory = TBinaryProtocol.TBinaryProtocolFactory()

    server = TServer.TSimpleServer(processor, transport, tfactory, pfactory)

    # You could do one of these for a multithreaded server
    # server = TServer.TThreadedServer(
    #     processor, transport, tfactory, pfactory)
    # server = TServer.TThreadPoolServer(
    #     processor, transport, tfactory, pfactory)


    print('Starting the server...')
    server.serve()
    print('done.')
