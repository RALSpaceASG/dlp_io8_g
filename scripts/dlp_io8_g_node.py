#!/usr/bin/env python

from __future__ import division
import rospy
import serial
import sys
from dlp_io8_msg.msg import monitorMessage


class DLP:
    def __init__(self,device,baud):

        
        ## Device location, baud rate
        self.device_name=device
        self.baud = baud
        
        ## Create 2D list for DLP-IO8-G commands
        self.DLP_CMDS = [0 for x in range(9)]

        ## Commands from DLP-IO8-G data sheet
        self.DLP_CMDS[0]= '\x27' #ping_device
        self.DLP_CMDS[1]= '\x5A' #ch1_volts
        self.DLP_CMDS[2]= '\x58' #ch2_volts
        self.DLP_CMDS[3]= '\x43' #ch3_volts
        self.DLP_CMDS[4]= '\x56' #ch4_volts
        self.DLP_CMDS[5]= '\x42' #ch5_volts
        self.DLP_CMDS[5]= '\x42' #ch5_volts
        self.DLP_CMDS[6]= '\x4E' #ch6_volts
        self.DLP_CMDS[7]= '\x4D' #ch7_volts
        self.DLP_CMDS[8]= '\x2C' #ch8_volts


    
    
    def initialize(self):
        ## Run through initialization process
        try:
            rospy.loginfo("Openning port to DLP ADC!")

            ## Open connected device at specificed USB port
            self.USB=serial.Serial(self.device_name,self.baud,timeout=2)

            ## Check to make sure the device opened properly
            port_check=self.USB.isOpen()
            if port_check:

                ## Send the check device command and ensure that the correct identifier is returned
                self.USB.write(self.DLP_CMDS[0])
                deviceCheck=self.USB.read()


            if deviceCheck=='\x51':

                self.USB.write('\x5C')
                rospy.loginfo("Device connected!")
                return 0
           
            else:
                rospy.logfatal("Device ID not recognised")
                return 1

        except Exception, e:
            rospy.logfatal("Exception Raised")
            print str(e)
            return 2

    def getVoltage(self,ch_num):

        self.USB.write(self.DLP_CMDS[ch_num])
        voltageCount = self.USB.read(2)

        voltageValue = (5/1023)*(ord(voltageCount[1]) + (ord(voltageCount[0]) << 8))
        return voltageValue


    def disconnect(self):
        self.USB.close()
        rospy.loginfo("DLP Disconnected")




def main():

    voltage_data = monitorMessage()
    rospy.init_node('rimmer_battery_monitor', anonymous=True)

    device_loc = rospy.get_param('~path')
    baud = rospy.get_param('~baud')
    sample_rate = rospy.get_param('~sample_rate')
    
    pub = rospy.Publisher('rimmer_voltage_level', monitorMessage, queue_size=10)
    rate = rospy.Rate(sample_rate)    


    dlp = DLP(device_loc,baud)
    dlp.initialize()

    while not rospy.is_shutdown():

        voltage_data.ch1 = dlp.getVoltage(1)
        voltage_data.ch2 = dlp.getVoltage(2)
        voltage_data.ch3 = dlp.getVoltage(3)
        voltage_data.ch4 = dlp.getVoltage(4)
        voltage_data.ch5 = dlp.getVoltage(5)
        voltage_data.ch6 = dlp.getVoltage(6)

        # rospy.loginfo(voltage_data)
        pub.publish(voltage_data)
        rate.sleep()

    dlp.disconnect()



if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass



