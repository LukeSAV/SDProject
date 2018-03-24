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
    //var leftTimer:Timer!
    //var rightTimer:Timer!
    
    
    @IBOutlet weak var leftSlider: UISlider!
    @IBOutlet weak var rightSlider: UISlider!
    
    @IBAction func leftSliderAct(_ sender: UISlider) {
        var myStr = "L" + String(Int(leftSlider.value)) + "\n"
        var data = myStr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }
    
    
    @IBAction func rightSliderAct(_ sender: UISlider) {
        var myStr = "R" + String(Int(rightSlider.value)) + "\n"
        var data = myStr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
    }

    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        let mystr = "SUCCESS"
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
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
