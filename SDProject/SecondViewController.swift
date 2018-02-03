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
    
    @IBOutlet weak var leftBtn: UIButton!
    @IBOutlet weak var rightBtn: UIButton!
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        let mystr = "SUCCESS"
        let data = mystr.data(using: String.Encoding.utf8)
        connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
        leftBtn.layer.cornerRadius = 5
        rightBtn.layer.cornerRadius = 5
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
