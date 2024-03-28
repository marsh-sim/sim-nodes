#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# Original code: https://github.com/labjack/labjack-ljm-python/blob/master/Examples/More/Stream/stream_basic.py
# Adapted into thread by Andrea Zanoni

from labjack import ljm
import socket
import struct

import sys

import threading
import time


class LJMReader(threading.Thread):
    def __init__(self, period, **kwargs):
        super().__init__()

        try:
            # LabJack Type: defaults to T4
            self.LJType = kwargs['LJType']
            if self.LJType not in ('T4', 'T7', 'T8'):
                raise ValueError
        except ValueError:
            print('LJMReader: unrecognised LabJack type. Aborting...', file=sys.stderr)
            sys.exit(100)
        except KeyError:
            self.LJType = 'T4'
            pass

        try:
            self.LJConnection = kwargs['LJConnection']
            # LabJack Connection: defaults to ETHERNET
            if self.LJConnection not in ('USB', 'ETHERNET', 'ANY'):
                raise ValueError
        except ValueError:
            print(
                'LJMReader: unrecognised LabJack connection type. Aborting...', file=sys.stderr)
            sys.exit(101)
        except KeyError:
            self.LJConnection = 'ETHERNET'
            pass

        try:
            self.aScanListNames = kwargs['aScanListNames']
            # LabJack signals to be output. For now, no check on channel name
            # is perfomed, so it is up to the user to get it right
        except KeyError:
            # By default, we output AIN0-3
            self.aScanListNames = ['AIN0', 'AIN1', 'AIN2', 'AIN3']
            pass

        try:
            # Address of the server to send the reads to
            self.serverAddress = kwargs['serverAddress']
            # here we should do more checks, but we'll trust the user for now...
        except KeyError:
            # Defaults to rpc-lnx2 and port 9011
            self.serverAddress = ('192.168.1.2', 9011)
            pass

        try:
            # same as above
            self.ScanRate = kwargs['ScanRate']
        except KeyError:
            # defaults to 1000 Hz
            self.ScanRate = 256
            pass

        self.LJhandle = ljm.openS(self.LJType, self.LJConnection, 'ANY')
        if self.LJhandle:
            info = ljm.getHandleInfo(self.LJhandle)
            print('LJMReader: Found LabJack with Device type: {}, Connection type: {},'.format(
                info[0], info[1]))
            print('LJMReader: Serial number: {}, IP address: {}, Port: {}, Max bytes per MB: {}'.format(
                info[2], ljm.numberToIP(info[3]), info[4], info[5]))
        else:
            print(
                'LJMReader: Could not find any LabJack device. Aborting...', file=sys.stderr)
            sys.exit(1)

        try:
            aNames = ['STREAM_SETTLING_US', 'STREAM_RESOLUTION_INDEX']
            aValues = [0, 0]

            s_out_bufsize = struct.calcsize('d'*len(self.aScanListNames))
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            ljm.eWriteNames(self.LJhandle, len(aNames), aNames, aValues)

            aScanList = ljm.namesToAddresses(
                len(self.aScanListNames), self.aScanListNames)[0]

            # Configure and start the LabJackstream, with scansPerRead = 1
            scanRate = ljm.eStreamStart(
                self.LJhandle, \
                # scansPerRead = 1
                1, \
                len(self.aScanListNames), \
                aScanList, \
                self.ScanRate)
            if scanRate == self.ScanRate:
                print(
                    'LJMReader: Stream started with a scan rate of {} Hz.'.format(scanRate))
            else:
                print('LJMReader: WARNING stream started with a scan rate of {} Hz.'.format(
                    scanRate), file=sys.stderr)

            self.period = period
            self.i = 0
            self.t0 = time.time()
            self.start()

        except ljm.LJMError:
            ljme = sys.exc_info()[1]
            print("LJMReader: LJM ERROR " + ljme, file=sys.stderr)
            # Close LabJack handle
            ljm.close(self.LJhandle)
        except Exception:
            e = sys.exc_info()[1]
            print("labjack_stream.py: ERROR {}".format(e))
            # Close LabJack handle
            ljm.close(self.LJhandle)

    def sleep(self):
        self.i += 1
        delta = self.t0 + self.period*self.i - time.time()
        if delta > 0:
            time.sleep(delta)

    def run(self):
        while True:
            self.read_and_send()
            self.sleep()

    def read_and_send(self):
        ret = ljm.eStreamRead(self.LJhandle)
        aData = ret[0]

        # Skipped samples are indicated with -9999.0 values
        curSkip = aData.count(-9999.0)
        if curSkip:
            print("LJMReader: WARNING {} skipped samples at read #{}".format(
                curSkip, curRead), file=sys.stderr)

        # Send data to socket
        # AIN0 = aData[0]
        # AIN1 = aData[1]
        # AIN2 = aData[2]
        # AIN3 = aData[3]

        buf_out = struct.pack('d'*len(aData), *aData)
        self.s.sendto(buf_out, self.serverAddress)


ljmr = LJMReader(1/256.)
