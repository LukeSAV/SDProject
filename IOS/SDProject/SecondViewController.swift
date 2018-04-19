//
//  SecondViewController.swift
//  SDProject
//
//  Created by Luke Armbruster on 2/3/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import UIKit
import CoreBluetooth
import CDJoystick

class SecondViewController: UIViewController {
    var connectedDevice:CBPeripheral!
    var charTopic:CBCharacteristic!
    var syncTimer:Timer? = nil

    @IBAction func stopBtn(_ sender: Any) {
        connectedDevice.writeValue("L90\n".data(using: String.Encoding.utf8)!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
        connectedDevice.writeValue("R90\n".data(using: String.Encoding.utf8)!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }
    
    
    @IBOutlet weak var robotJS: CDJoystick!
    
    @objc func sync() {
        let mystr = "S\n"
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        syncTimer = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(self.sync), userInfo: nil, repeats: true)
        robotJS.trackingHandler = { joystickData in
            let x_vel = Double(joystickData.velocity.x)
            let y_vel = Double(joystickData.velocity.y) * -1.0
            let angle = Double(joystickData.angle / .pi)
            
            var left_base = 0.0 // Base value from angle
            var right_base = 0.0 // Base value from angle
            var multi = 0.0 // Multiplier from distance

            if angle <= 0.25 {
                left_base = 90
                right_base = 90 - (angle / 0.25 * 45)
            } else if angle <= 0.5 {
                left_base = 90 - ((angle - 0.25) / 0.25 * 90)
                right_base = 45 - ((angle - 0.25) / 0.25 * 45)
            } else if angle <= 0.75 {
                left_base = 0 - ((angle - 0.5) / 0.25 * 90)
                right_base = 0 - ((angle - 0.5) / 0.25 * 45)
            } else if angle <= 1.0 {
                left_base = -90
                right_base = -45 - ((angle - 0.75) / 0.25 * 45)
            } else if angle <= 1.25 {
                left_base = -90 + ((angle - 1.0) / 0.25 * 45)
                right_base = -90
            } else if angle <= 1.5 {
                left_base = -45 + ((angle - 1.25) / 0.25 * 45)
                right_base = -90 + ((angle - 1.25) / 0.25 * 90)
            } else if angle <= 1.75 {
                left_base = (angle - 1.5) / 0.25 * 45
                right_base = (angle - 1.5) / 0.25 * 90
            } else if angle <= 2.0 {
                left_base = 45 + ((angle - 1.75) / 0.25 * 45)
                right_base = 90
            }

            multi = ((x_vel * x_vel) + (y_vel * y_vel)).squareRoot()
            if multi > 1 {
                multi = 1 // Reduce to circle
            }
            let left_str = "L" + String(Int(left_base * multi + 90)) + "\n"
            let right_str = "R" + String(Int(right_base * multi + 90)) + "\n"
            let left_data = String(left_str).data(using: String.Encoding.utf8)
            let right_data = String(right_str).data(using: String.Encoding.utf8)

            self.connectedDevice.writeValue(left_data!, for: self.charTopic, type: CBCharacteristicWriteType.withoutResponse)
            self.connectedDevice.writeValue(right_data!, for: self.charTopic, type: CBCharacteristicWriteType.withoutResponse)
            
        }

        // Do any additional setup after loading the view.
        //let mystr = "SUCCESS"
        //let data = mystr.data(using: String.Encoding.utf8)
        //connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    


    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destinationViewController.
        // Pass the selected object to the new view controller.
    }
    */

}
