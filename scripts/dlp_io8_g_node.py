#!/usr/bin/env python

# Written by: Steven Kay, Autonomous Systems Engineer
# steven.kay@stfc.ac.uk, +44 (0)1235778759
# Autonomous Systems Group, RAL Space
# May 2017


from __future__ import division
import rospy
import serial
import sys
import time
from dlp_io8_g.msg import dlp_io8_msg

class DLP_IO8_G:
    def __init__(self,device,baud):


        ## Device location, baud rate
        self.device_name=device
        self.baud = baud
        
        ## Create 2D list for DLP-IO8-G commands
        self.dlp_cmd = [0 for x in range(9)]

        ## Commands from DLP-IO8-G data sheet
        self.dlp_cmd[0]= '\x27' #ping_device
        self.dlp_cmd[1]= '\x5A' #ch1_volts
        self.dlp_cmd[2]= '\x58' #ch2_volts
        self.dlp_cmd[3]= '\x43' #ch3_volts
        self.dlp_cmd[4]= '\x56' #ch4_volts
        self.dlp_cmd[5]= '\x42' #ch5_volts
        self.dlp_cmd[5]= '\x42' #ch5_volts
        self.dlp_cmd[6]= '\x4E' #ch6_volts
        self.dlp_cmd[7]= '\x4D' #ch7_volts
        self.dlp_cmd[8]= '\x2C' #ch8_volts

    
    def initialise(self):
        ## Run through initialization process
        try:
            fail = False
            rospy.loginfo("Openning port to DLP ADC!")

            ## Open connected device at specificed USB port
            self.USB=serial.Serial(self.device_name,self.baud,timeout=2)

        except Exception, e:
            rospy.logfatal("Exception Raised" + str(e))
            fail = True
            pass

        if fail == False:
            ## Check to make sure the device opened properly
            port_check=self.USB.isOpen()
            if port_check:
                ## Send the check device command and ensure that the correct identifier is returned
                self.USB.write(self.dlp_cmd[0])
                deviceCheck = self.USB.read()

                if deviceCheck=='\x51':
                    self.USB.write('\x5C')
                    return 0
            else:
                return 1
        else:
            return 2



    def getVoltage(self,ch_num):

        self.USB.write(self.dlp_cmd[ch_num])
        voltageCount = self.USB.read(2)

        voltageValue = (5/1023)*(ord(voltageCount[1]) + (ord(voltageCount[0]) << 8))
        return voltageValue


    def disconnect(self):
        self.USB.close()
        rospy.loginfo("DLP Disconnected")




def main():

    ## Initialise msg format from 'dlp_io8_g_msg.msg' file
    voltage_data = dlp_io8_msg()

    ## Initialise Node 'dlp_io8_g'
    rospy.init_node('dlp_io8_g', anonymous=True)

    ## Load device kocation data, see top for further detail
    device_loc = rospy.get_param('~path')

    ## Load Device Baud Rate with default 115200 baud
    baud = rospy.get_param('~baud','115200')

    ## Load desired sampling rate with default 5 Hz
    sample_rate = rospy.get_param('~sample_rate','5')
    
    ## Initialise Publisher with default queue size 1
    pub = rospy.Publisher('voltage_levels', dlp_io8_msg, queue_size=1)
    
    ## Set publishing rate with sample_rate parameter, default 5Hz
    rate = rospy.Rate(sample_rate)    

    ## Instantiate DLP_io8_G Class and setup device details
    dlp = DLP_IO8_G(device_loc,baud)

    ## Establish connection with device
    conn_status = 2
    ## Attempt to connect, if unsuccessful, keep trying every second
    while conn_status != 0:
        ## Run initialise function, if;
        ## conn_status = 0, connected
        ## if conn_status = 1, read error
        ## if conn_status == 2, device location not found
        conn_status = dlp.initialise()
        if conn_status == 0:
            rospy.loginfo("Device Connected!")
        elif conn_status == 1:
            rospy.logwarn("Device Ping Unsuccessful")
            time.sleep(1)
        elif conn_status == 2:
            rospy.logfatal("Device Not Found - Will retry every second")
            time.sleep(1)


    while not rospy.is_shutdown():

        voltage_data.header.stamp = rospy.Time.now()
        voltage_data.ch1 = dlp.getVoltage(1)
        voltage_data.ch2 = dlp.getVoltage(2)
        voltage_data.ch3 = dlp.getVoltage(3)
        voltage_data.ch4 = dlp.getVoltage(4)
        voltage_data.ch5 = dlp.getVoltage(5)
        voltage_data.ch6 = dlp.getVoltage(6)
        voltage_data.ch7 = dlp.getVoltage(7)
        voltage_data.ch8 = dlp.getVoltage(8)

        pub.publish(voltage_data)
        rate.sleep()

    dlp.disconnect()



if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass



