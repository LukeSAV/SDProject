//
//  SecondInterfaceController.swift
//  Watch Extension
//
//  Created by Luke Armbruster on 3/27/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import WatchKit
import Foundation
import CoreBluetooth

class SecondInterfaceController: WKInterfaceController, WKCrownDelegate {
    
    var connectedDevice:CBPeripheral!
    var manager:CBCentralManager!
    var charTopic:CBCharacteristic!
    var value = 0.0
    var left_speed = 90
    var right_speed = 90
    var left_offset = 0
    var right_offset = 0
    var syncTimer:Timer? = nil

    @IBAction func driveLeftBtn() {
        left_speed += 1
        left_offset += 1
        if(left_speed > 180) {
            left_speed = 180
            left_offset = 0
        }
        writeLeftSpeed()
    }
    
    @IBAction func reverseLeftBtn() {
        left_speed -= 1
        left_offset -= 1
        if(left_speed < 0) {
            left_speed = 0
            left_offset = 0
        }
        writeLeftSpeed()
    }
    
    @IBAction func driveRightBtn() {
        right_speed += 1
        right_offset += 1
        // Keep within bounds
        if(right_speed > 180) {
            right_speed = 180
            right_offset = 0
        }
        writeRightSpeed()
    }
    
    @IBAction func reverseRightBtn() {
        right_speed -= 1
        right_offset -= 1
        // Keep within bounds
        if(right_speed < 0) {
            right_speed = 0
            right_offset = 0
        }
        writeRightSpeed()
    }
    
    @IBOutlet var leftLbl: WKInterfaceLabel!
    @IBOutlet var rightLbl: WKInterfaceLabel!
    
    func writeLeftSpeed() {
        let mystr = "L" + String(left_speed) + "\n"
        leftLbl.setText(mystr)
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }
    
    func writeRightSpeed() {
        let mystr = "R" + String(right_speed) + "\n"
        rightLbl.setText(mystr)
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }

    @objc func sync() {
        let mystr = "S\n"
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }
    
    override func awake(withContext context: Any?) {
        print("Reached second interface")
        super.awake(withContext: nil)
        connectedDevice = (context as! NSDictionary)["connectedDevice"] as? CBPeripheral
        manager = (context as! NSDictionary)["manager"] as? CBCentralManager
        charTopic = (context as! NSDictionary)["charTopic"] as? CBCharacteristic
        crownSequencer.delegate = self
        crownSequencer.focus()
        syncTimer = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(self.sync), userInfo: nil, repeats: true)
    }
    

    override func willActivate() {
        super.willActivate()
        crownSequencer.focus()
    }
    
    override func didDeactivate() {
        super.didDeactivate()
        if(syncTimer != nil) {
            syncTimer!.invalidate()
        }
        /*if(connectedDevice != nil) {
            manager.cancelPeripheralConnection(connectedDevice)
        }*/
    }
    
    func crownDidRotate(_ crownSequencer: WKCrownSequencer?, rotationalDelta: Double) {
        value += rotationalDelta
        if(value > 1) {
            //let val_diff = value - 1.0
            value = 1.0
            /*if(right_offset < 0) {
                right_offset += Int(val_diff * 90);
            }
            if(left_offset < 0) {
                left_offset += Int(val_diff * 90);
            }*/
        } else if(value < -1.0) {
            //let val_diff = value + 1.0
            value = -1.0
            /*if(right_offset > 0) {
                right_offset += Int(val_diff * 90);
            }
            if(left_offset > 0) {
                left_offset += Int(val_diff * 90);
            }*/
        }

        // Allows user to touch buttons and still have crown move left and right in lock step to increase speed
        left_speed = Int(90 + value * 90) + left_offset // Crown goes from -1 to 1, speed goes from 0 to 180
        right_speed = Int(90 + value * 90) + right_offset
        
        // Prevents button presses followed by crown movement to move user above specified range
        if(left_speed > 180) {
            left_speed = 180
            left_offset = 0
        } else if(left_speed < 0) {
            left_speed = 0
            left_offset = 0
        }
        if(right_speed > 180) {
            right_speed = 180
            right_offset = 0
        } else if(right_speed < 0) {
            right_speed = 0
            right_offset = 0
        }
        writeLeftSpeed()
        writeRightSpeed()
    }
}

