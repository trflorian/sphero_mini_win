import asyncio
import struct
import sys
import time

from sphero_constants import *
from bleak import BleakClient

class SpheroMini:
    def __init__(self, address):
        self.sequence = 0
        self.address = address
        self.notificationPacket = []   
        self.clear_notification()

    async def connect(self):
        self.client = BleakClient(self.address)
        await self.client.connect(timeout=5.0)
        connected = await self.client.is_connected()
        print("Connected: {0}".format(connected))

        # cancel if not connected
        if not connected:
            return False
        
        # get device name
        try:
            DEVICE_NAME_UUID = "00002A00-0000-1000-8000-00805f9b34fb"
            device_name = await self.client.read_gatt_char(DEVICE_NAME_UUID)
            print("Device Name: {0}".format("".join(map(chr, device_name))))
        except Exception:
            pass

        self.API_V2_characteristic = "00010002-574f-4f20-5370-6865726f2121"
        AntiDOS_characteristic  = "00020005-574f-4f20-5370-6865726f2121"
        DFU_characteristic = "00020002-574f-4f20-5370-6865726f2121"
        DFU2_characteristic = "00020004-574f-4f20-5370-6865726f2121"
        services = await self.client.get_services()
        API_descriptor = services.get_characteristic(self.API_V2_characteristic).descriptors[0]
        DFU_descriptor = services.get_characteristic(DFU_characteristic).descriptors[0]

        # Unlock code: prevent the sphero mini from going to sleep again after 10 seconds
        print("[INIT] Writing AntiDOS characteristic unlock code")
        await self.client.write_gatt_char(AntiDOS_characteristic,"usetheforce...band".encode(),response=True)

        # Enable DFU notifications:
        print("[INIT] Configuring DFU descriptor")
        # current_value = await client.read_gatt_descriptor(DFU_descriptor.handle)
        # print("current dfu descriptor: ",current_value)
        await self.client.write_gatt_descriptor(DFU_descriptor.handle,struct.pack('<bb', 0x01, 0x00))

        # No idea what this is for. Possibly a device ID of sorts? Read request returns '00 00 09 00 0c 00 02 02':
        print("[INIT] Reading DFU2 characteristic")
        await self.client.read_gatt_char(DFU2_characteristic)

        # Enable API notifications:
        print("[INIT] Configuring API dectriptor")
        await self.client.write_gatt_descriptor(API_descriptor.handle,struct.pack('<bb', 0x01, 0x00))
        
        # enable notifications on api characteristic
        await self.client.start_notify(self.API_V2_characteristic, self.handleNotification)

        print("[INIT] Initialization complete\n")

        return True

    async def _send(self, characteristic=None, devID=None, commID=None, payload=[]):
        '''
        A generic "send" method, which will be used by other methods to send a command ID, payload and
        appropriate checksum to a specified device ID. Mainly useful because payloads are optional,
        and can be of varying length, to convert packets to binary, and calculate and send the
        checksum. For internal use only.
        Packet structure has the following format (in order):
        - Start byte: always 0x8D
        - Flags byte: indicate response required, etc
        - Virtual device ID: see sphero_constants.py
        - Command ID: see sphero_constants.py
        - Sequence number: Seems to be arbitrary. I suspect it is used to match commands to response packets (in which the number is echoed).
        - Payload: Could be varying number of bytes (incl. none), depending on the command
        - Checksum: See below for calculation
        - End byte: always 0xD8
        '''
        sendBytes = [sendPacketConstants["StartOfPacket"],
                    sum([flags["resetsInactivityTimeout"], flags["requestsResponse"]]),
                    devID,
                    commID,
                    self.sequence] + payload # concatenate payload list

        self.sequence += 1 # Increment sequence number, ensures we can identify response packets are for this command
        if self.sequence > 255:
            self.sequence = 0

        # Compute and append checksum and add EOP byte:
        # From Sphero docs: "The [checksum is the] modulo 256 sum of all the bytes
        #                   from the device ID through the end of the data payload,
        #                   bit inverted (1's complement)"
        # For the sphero mini, the flag bits must be included too:
        checksum = 0
        for num in sendBytes[1:]:
            checksum = (checksum + num) & 0xFF # bitwise "and to get modulo 256 sum of appropriate bytes
        checksum = 0xff - checksum # bitwise 'not' to invert checksum bits
        sendBytes += [checksum, sendPacketConstants["EndOfPacket"]] # concatenate

        # Convert numbers to bytes
        output = b"".join([x.to_bytes(1, byteorder='big') for x in sendBytes])

        #send to specified characteristic:
        await self.client.write_gatt_char(characteristic,output,response=True)


    async def wake(self):
        '''
        Bring device out of sleep mode (can only be done if device was in sleep, not deep sleep).
        If in deep sleep, the device should be connected to USB power to wake.
        '''
        
        print("[SEND {}] Waking".format(self.sequence))
        
        await self._send(
                    characteristic=self.API_V2_characteristic,
                    devID=deviceID['powerInfo'],
                    commID=powerCommandIDs["wake"],
                    payload=[]) # empty payload  

    async def roll(self, speed=None, heading=None):
        '''
        Start to move the Sphero at a given direction and speed.
        heading: integer from 0 - 360 (degrees)
        speed: Integer from 0 - 255
        Note: the zero heading should be set at startup with the resetHeading method. Otherwise, it may
        seem that the sphero doesn't honor the heading argument
        '''
        print("[SEND {}] Rolling with speed {} and heading {}".format(self.sequence, speed, heading))
    
        if abs(speed) > 255:
            print("WARNING: roll speed parameter outside of allowed range (-255 to +255)")

        if speed < 0:
            speed = -1*speed+256 # speed values > 256 in the send packet make the spero go in reverse

        speedH = (speed & 0xFF00) >> 8
        speedL = speed & 0xFF
        headingH = (heading & 0xFF00) >> 8
        headingL = heading & 0xFF
        await self._send(characteristic = self.API_V2_characteristic,
                  devID = deviceID['driving'],
                  commID = drivingCommands["driveWithHeading"],
                  payload = [speedL, headingH, headingL, speedH])

    async def setLEDColor(self, red = None, green = None, blue = None):
        '''
        Set device LED color based on RGB vales (each can  range between 0 and 0xFF)
        '''
        print("[SEND {}] Setting main LED colour to [{}, {}, {}]".format(self.sequence, red, green, blue))
        
        await self._send(characteristic = self.API_V2_characteristic,
                  devID = deviceID['userIO'], # 0x1a
                  commID = userIOCommandIDs["allLEDs"], # 0x0e
                  payload = [0x00, 0x0e, red, green, blue])

    def clear_notification(self):
        self.notification_ack = "DEFAULT ACK"
        self.notification_seq = -1

    def bits_to_num(self, bits):
        '''
        This helper function decodes bytes from sensor packets into single precision floats. Encoding follows the
        the IEEE-754 standard.
        '''
        num = int(bits, 2).to_bytes(len(bits) // 8, byteorder='little')
        num = struct.unpack('f', num)[0]
        return num

    
    async def getAcknowledgement(self, ack):
        #wait up to 10 secs for correct acknowledgement to come in, including sequence number!
        start = time.time()
        while(1):
            await asyncio.sleep(0.01)
            # TODO self.p.waitForNotifications(1)
            
            if self.notification_seq == self.sequence-1: # use one less than sequence, because _send function increments it for next send. 
                print("[RESP {}] {}".format(self.sequence-1, self.notification_ack))
                self.clear_notification()
                break
            elif self.notification_seq >= 0:
                print("Unexpected ACK. Expected: {}/{}, received: {}/{}".format(
                    ack, self.sequence, self.notification_ack.split()[0],
                    self.notification_seq),
                    file=sys.stderr)
            if time.time() > start + 10:
                print("Timeout waiting for acknowledgement: {}/{}".format(ack, self.sequence), file=sys.stderr)
                break

    def handleNotification(self, sender, data):
        '''
        This method acts as an interrupt service routine. When a notification comes in, this
        method is invoked, with the variable 'cHandle' being the handle of the characteristic that
        sent the notification, and 'data' being the payload (sent one byte at a time, so the packet
        needs to be reconstructed)  
        The method keeps appending bytes to the payload packet byte list until end-of-packet byte is
        encountered. Note that this is an issue, because 0xD8 could be sent as part of the payload of,
        say, the battery voltage notification. In future, a more sophisticated method will be required.
        '''

        for data_byte in data: # parse each byte separately (sometimes they arrive simultaneously)

            self.notificationPacket.append(data_byte) # Add new byte to packet list

            # If end of packet (need to find a better way to segment the packets):
            if data_byte == sendPacketConstants['EndOfPacket']:
                # Once full the packet has arrived, parse it:
                # Packet structure is similar to the outgoing send packets (see docstring in sphero_mini._send())
                
                # Attempt to unpack. Might fail if packet is too badly corrupted
                try:
                    start, flags_bits, devid, commcode, seq, *notification_payload, chsum, end = self.notificationPacket
                except ValueError:
                    print("Warning: notification packet unparseable", self.notificationPacket, file=sys.stderr)
                    self.notificationPacket = [] # Discard this packet
                    return # exit

                # Compute and append checksum and add EOP byte:
                # From Sphero docs: "The [checksum is the] modulo 256 sum of all the bytes
                #                   from the device ID through the end of the data payload,
                #                   bit inverted (1's complement)"
                # For the sphero mini, the flag bits must be included too:
                checksum_bytes = [flags_bits, devid, commcode, seq] + notification_payload
                checksum = 0 # init
                for num in checksum_bytes:
                    checksum = (checksum + num) & 0xFF # bitwise "and to get modulo 256 sum of appropriate bytes
                checksum = 0xff - checksum # bitwise 'not' to invert checksum bits
                if checksum != chsum: # check computed checksum against that recieved in the packet
                    print("Warning: notification packet checksum failed", self.notificationPacket, file=sys.stderr)
                    self.notificationPacket = [] # Discard this packet
                    return # exit

                # Check if response packet:
                if flags_bits & flags['isResponse']: # it is a response

                    # Use device ID and command code to determine which command is being acknowledged:
                    if devid == deviceID['powerInfo'] and commcode == powerCommandIDs['wake']:
                        self.notification_ack = "Wake acknowledged" # Acknowledgement after wake command
                        
                    elif devid == deviceID['driving'] and commcode == drivingCommands['driveWithHeading']:
                        self.notification_ack = "Roll command acknowledged"

                    elif devid == deviceID['driving'] and commcode == drivingCommands['stabilization']:
                        self.notification_ack = "Stabilization command acknowledged"

                    elif devid == deviceID['userIO'] and commcode == userIOCommandIDs['allLEDs']:
                        self.notification_ack = "LED/backlight color command acknowledged"

                    elif devid == deviceID['driving'] and commcode == drivingCommands["resetHeading"]:
                        self.notification_ack = "Heading reset command acknowledged"

                    elif devid == deviceID['sensor'] and commcode == sensorCommands["configureCollision"]:
                        self.notification_ack = "Collision detection configuration acknowledged"

                    elif devid == deviceID['sensor'] and commcode == sensorCommands["configureSensorStream"]:
                        self.notification_ack = "Sensor stream configuration acknowledged"

                    elif devid == deviceID['sensor'] and commcode == sensorCommands["sensorMask"]:
                        self.notification_ack = "Mask configuration acknowledged"

                    elif devid == deviceID['sensor'] and commcode == sensorCommands["sensor1"]:
                        self.notification_ack = "Sensor1 acknowledged"

                    elif devid == deviceID['sensor'] and commcode == sensorCommands["sensor2"]:
                        self.notification_ack = "Sensor2 acknowledged"

                    elif devid == deviceID['powerInfo'] and commcode == powerCommandIDs['batteryVoltage']:
                        V_batt = notification_payload[2] + notification_payload[1]*256 + notification_payload[0]*65536
                        V_batt /= 100 # Notification gives V_batt in 10mV increments. Divide by 100 to get to volts.
                        self.notification_ack = "Battery voltage:" + str(V_batt) + "v"
                        self.v_batt = V_batt

                    elif devid == deviceID['systemInfo'] and commcode == SystemInfoCommands['mainApplicationVersion']:
                        version = '.'.join(str(x) for x in notification_payload)
                        self.notification_ack = "Firmware version: " + version
                        self.firmware_version = notification_payload
                                                
                    else:
                        self.notification_ack = "Unknown acknowledgement" #print(self.notificationPacket)
                        print(self.notificationPacket, "===================> Unknown ack packet")

                    self.notification_seq = seq

                else: # Not a response packet - therefore, asynchronous notification (e.g. collision detection, etc):
                    
                    # Collision detection:
                    if devid == deviceID['sensor'] and commcode == sensorCommands['collisionDetectedAsync']:
                        # The first four bytes are data that is still un-parsed. the remaining unsaved bytes are always zeros
                        _, _, _, _, _, _, axis, _, Y_mag, _, X_mag, *_ = notification_payload
                        if axis == 1: 
                            dir = "Left/right"
                        else:
                            dir = 'Forward/back'
                        print("Collision detected:")
                        print("\tAxis:", dir)
                        print("\tX_mag:", X_mag)
                        print("\tY_mag:", Y_mag)

                        if self.collision_detection_callback is not None:
                            self.notificationPacket = [] # need to clear packet, in case new notification comes in during callback
                            self.collision_detection_callback()

                    # Sensor response:
                    elif devid == deviceID['sensor'] and commcode == sensorCommands['sensorResponse']:
                        # Convert to binary, pad bytes with leading zeros:
                        val = ''
                        for byte in notification_payload:
                            val += format(int(bin(byte)[2:], 2), '#010b')[2:]
                        
                        # Break into 32-bit chunks
                        nums = []
                        while(len(val) > 0):
                            num, val = val[:32], val[32:] # Slice off first 16 bits
                            nums.append(num)
                        
                        # convert from raw bits to float:
                        nums = [self.bits_to_num(num) for num in nums]

                        # Set sensor values as class attributes:
                        for name, value in zip(self.configured_sensors, nums):
                            setattr(name, value)
                        
                    # Unrecognized packet structure:
                    else:
                        self.notification_ack = "Unknown asynchronous notification" #print(self.notificationPacket)
                        print(self.notificationPacket, "===================> Unknown async packet")
                        
                self.notificationPacket = [] # Start new payload after this byte