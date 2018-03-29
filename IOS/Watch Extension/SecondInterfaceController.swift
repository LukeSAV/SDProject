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

    @IBAction func driveLeftBtn() {
        left_speed += 1
        writeLeftSpeed()
    }
    
    @IBAction func reverseLeftBtn() {
        left_speed -= 1
        writeLeftSpeed()
    }
    
    @IBAction func driveRightBtn() {
        right_speed += 1
        writeRightSpeed()
    }
    
    @IBAction func reverseRightBtn() {
        right_speed -= 1
        writeRightSpeed()
    }
    
    func writeLeftSpeed() {
        let mystr = "L" + String(left_speed) + "\n"
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }
    
    func writeRightSpeed() {
        let mystr = "R" + String(right_speed) + "\n"
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
        Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(self.sync), userInfo: nil, repeats: true)
    }
    

    override func willActivate() {
        super.willActivate()
        crownSequencer.focus()
    }
    
    override func didDeactivate() {
        super.didDeactivate()
    }
    
    func crownDidRotate(_ crownSequencer: WKCrownSequencer?, rotationalDelta: Double) {
        value += rotationalDelta
        if(value > 1) {
            value = 1.0
        } else if(value < -1.0) {
            value = -1.0
        }
        left_speed = Int(90 + value * 90) // Crown goes from -1 to 1, speed goes from 0 to 180
        right_speed = left_speed
        writeLeftSpeed()
        writeRightSpeed()
    }
}

