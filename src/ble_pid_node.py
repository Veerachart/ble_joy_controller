#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, Float64
from bluepy.btle import Scanner, DefaultDelegate, Peripheral, BTLEException
from struct import pack, unpack
import random
import sys,os
import time
import csv
pathname = os.path.dirname(os.path.dirname(sys.argv[0]))

class MyDelegate(DefaultDelegate):
    def __init__(self, controller):
        DefaultDelegate.__init__(self)
        self.controller = controller

    def handleNotification(self, cHandle, data):
        #if cHandle == self.controller.battChar.getHandle():
            #print "Battery received"
            #if not self.controller.battNotified:
        #    self.controller.batt_pub.publish(ord(data))
            #self.controller.battNotified = True
        if cHandle == self.controller.orientationChar.getHandle():
            #if not self.controller.orientationNotified:
            (yaw,) = unpack("<h", data)
            #rospy.loginfo(yaw)
            self.controller.orientation_pub.publish(yaw)
            #self.controller.orientationNotified = True
            #(x, ) = unpack("<h", data)
            #ospy.loginfo(x)
        #if cHandle == self.controller.orientationChar2.getHandle():
        #    rospy.loginfo("1: %s", data)
        #if cHandle == self.controller.orientationChar.getHandle():
        #    rospy.loginfo("2: %s", data)
            
class BLEJoyController:
    def __init__(self, save_name=""):
        self.prev_fwd = 0
        self.prev_turn = 0
        self.prev_vert = 0
        self.prev_side = 0
        
        self.fwd = 0
        self.turn = 0
        self.vert = 0
        self.side = 0
        
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback)
        #self.batt_pub = rospy.Publisher("batt", Int16, queue_size=1)
        self.orientation_pub = rospy.Publisher("yaw", Int16, queue_size=1)
        self.blimp = Peripheral("84:68:3e:03:eb:aa")
        #self.blimp = Peripheral("84:68:3e:03:ee:df")
        self.initConnections()
        
        #self.onOff = False
        
        #self.battService = self.blimp.getServiceByUUID(0x180F)
        #self.battChar = self.battService.getCharacteristics(0x2A19)[0]

        #self.orientationService = self.blimp.getServiceByUUID("BEEF")
        #self.orientationChar = self.orientationService.getCharacteristics("FEED")[0]
        
        #self.orientationService = self.blimp.getServiceByUUID("19b10000e8f2537e4f6cd104768a1216")
        #self.orientationChar = self.orientationService.getCharacteristics("FFF2")[0]
        #self.orientationChar2 = self.orientationService.getCharacteristics("FFF1")[0]
        
        #self.commandService = self.blimp.getServiceByUUID("19b10000e8f2537e4f6cd104768a1214")
        #self.commandChar = self.commandService.getCharacteristics("19b10001e8f2537e4f6cd104768a1214")[0]
        
        self.uxSub = rospy.Subscriber("u_x", Float64, self.uxCallback)
        self.uySub = rospy.Subscriber("u_y", Float64, self.uyCallback)
        self.uzSub = rospy.Subscriber("u_z", Float64, self.uzCallback)
        self.desiredYawSub = rospy.Subscriber("cmd_yaw", Int16, self.yawCallback)
        
        #print self.commandChar.getHandle(), 
        #print self.orientationChar.getHandle(), 
        #print self.orientationChar2.getHandle()
        #print self.battChar.getHandle()
        
        #self.enable_notify("301c9b44-a61b-408a-a8bf-5efcd95a3486")
        #self.enable_notify(self.battChar.uuid)
        #self.enable_notify("FEED")
        #self.enable_notify("FFF2")
        #self.enable_notify("FFF1")
        self.save = len(save_name) > 0
        if self.save:
            self.file_name = pathname+"/log/"+time.strftime("%m%d_%H%M")+"-commands_"+save_name+".csv"
            self.f = open(self.file_name, 'wb')
            self.writer = csv.writer(self.f)
            self.writer.writerow("time,u_x,u_y,u_z,desired_yaw".split(','))

    
    def initConnections(self):
        self.commandService = self.blimp.getServiceByUUID("301c9b20-a61b-408a-a8bf-5efcd95a3486")
        self.commandChar = self.commandService.getCharacteristics("301c9b21-a61b-408a-a8bf-5efcd95a3486")[0]
        
        self.orientationService = self.blimp.getServiceByUUID("301c9b40-a61b-408a-a8bf-5efcd95a3486")
        self.orientationChar = self.orientationService.getCharacteristics("301c9b44-a61b-408a-a8bf-5efcd95a3486")[0]
        
        self.blimp.setDelegate(MyDelegate(self))
        self.enable_notify(self.orientationChar.uuid)
    
    def joyCallback(self, msg):
        if abs(msg.axes[3]) > 0.005 or abs(msg.axes[2]) > 0.005 or abs(msg.axes[1] > 0.005) or abs(msg.axes[0]) > 0.005:
            self.fwd = int(round(64*msg.axes[3]))
            self.turn = int(round(64*msg.axes[2]))
            self.vert = int(round(-64*msg.axes[1]))
            self.side = int(round(-64*msg.axes[0]))
    
    def uxCallback(self, msg):
        self.prev_fwd = self.fwd
        self.fwd = int(round(msg.data))
        
        
    def uyCallback(self, msg):
        self.prev_side = self.side
        self.side = int(round(msg.data))
        
        
    def uzCallback(self, msg):
        self.prev_vert = self.vert
        self.vert = int(round(msg.data))
        
        
    def yawCallback(self, msg):
        self.prev_turn = self.turn
        self.turn = msg.data
        
        
    def writeJoy(self):
        if (self.fwd != self.prev_fwd) or (self.turn != self.prev_turn) or (self.vert != self.prev_vert) or (self.side != self.prev_side):
            # Value changed, need to send new message to the blimp
            cmd = pack(">hhhh", self.fwd, self.turn, self.vert, self.side)
            self.commandChar.write(cmd)
            self.prev_fwd = self.fwd
            self.prev_turn = self.turn
            self.prev_vert = self.vert
            self.prev_side = self.side
#            rospy.loginfo("Command sent")
            if self.save:
                try:
                    data = "%.6f,%d,%d,%d,%d" % (rospy.Time.now().to_time(), self.fwd, self.side, self.vert, self.turn)
                    self.writer.writerow(data.split(','))
                except csv.Error as e:
                    sys.exit('File %s, line %d: %s' % (self.file_name, self.writer.line_num, e))
#        cmd = pack(">hhhh", random.randint(-255,255), random.randint(-255,255), random.randint(-255,255), random.randint(-255,255))
#        self.commandChar.write(cmd)
        #rospy.loginfo("Command sent")
            
            
    def enable_notify(self,  chara_uuid):
        setup_data = b"\x01\x00"
        notify = self.blimp.getCharacteristics(uuid=chara_uuid)[0]
        print notify
        notify_handle = notify.getHandle() + 1
        print notify_handle
        self.blimp.writeCharacteristic(notify_handle, setup_data, withResponse=False)


if __name__ == '__main__':
    rospy.init_node('ble_joy_node')
    
    ble_joy = None
    
    if rospy.has_param("~save_name"):
        ble_joy = BLEJoyController(rospy.get_param("~save_name"))
    else:
        ble_joy = BLEJoyController()
    
    r = rospy.Rate(25)
    counter = 0
    
    #print ble_joy.orientationChar.read()
    while not rospy.is_shutdown():
        try:
            ble_joy.blimp.waitForNotifications(0.035)
            #ble_joy.blimp.waitForNotifications(0.01)
            #if ble_joy.blimp.waitForNotifications(0.019):
            #    rospy.loginfo("Receive something")
            #else:
            #    rospy.loginfo("Waiting")
            #counter += 1
            #if counter == 20:
            ble_joy.writeJoy()
            #    counter = 0;
            r.sleep()
        except BTLEException as e:
            rospy.logerr("Error: %s", e)
            # reconnect
            ble_joy.blimp.connect("84:68:3e:03:eb:aa")
            #ble_joy.blimp.connect("84:68:3e:03:ee:df")
            ble_joy.initConnections()
