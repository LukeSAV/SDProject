//
//  SecondViewController.swift
//  SDProject
//
//  Created by Luke Armbruster on 2/3/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import UIKit
import CoreBluetooth

class SecondViewController: UIViewController {
    var connectedDevice:CBPeripheral!
    var charTopic:CBCharacteristic!
    var leftTimer:Timer!
    var rightTimer:Timer!
    
    @IBOutlet weak var leftBtn: UIButton!
    @IBOutlet weak var rightBtn: UIButton!
    
    @objc func leftButtonDown(sender: AnyObject) {
        print("Left")
        leftTimer = Timer.scheduledTimer(timeInterval: 0.3, target: self, selector: #selector(moveLeftMotor), userInfo: nil, repeats: true)
    }
    @objc func rightButtonDown(sender: AnyObject) {
        print("Right")
        rightTimer = Timer.scheduledTimer(timeInterval: 0.3, target: self, selector: #selector(moveRightMotor), userInfo: nil, repeats: true)
    }

    @objc func leftButtonUp(sender: AnyObject) {
        leftTimer.invalidate()
    }
    @objc func rightButtonUp(sender: AnyObject) {
        rightTimer.invalidate()
    }
    
    @objc func moveLeftMotor() {
        let mystr = "L"
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }
    @objc func moveRightMotor() {
        let mystr = "R"
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        let mystr = "SUCCESS"
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
        leftBtn.layer.cornerRadius = 5
        rightBtn.layer.cornerRadius = 5
        
        leftBtn.addTarget(self, action:#selector(leftButtonDown(sender:)), for: .touchDown)
        leftBtn.addTarget(self, action:#selector(leftButtonUp(sender:)), for: [.touchUpInside, .touchUpOutside])
        
        rightBtn.addTarget(self, action:#selector(rightButtonDown(sender:)), for: .touchDown)
        rightBtn.addTarget(self, action:#selector(rightButtonUp(sender:)), for: [.touchUpInside, .touchUpOutside])
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
