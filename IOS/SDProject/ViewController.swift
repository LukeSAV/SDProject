//
//  ViewController.swift
//  SDProject
//
//  Created by Luke Armbruster on 1/27/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import UIKit
import CoreBluetooth

class ViewController: UIViewController, CBCentralManagerDelegate, CBPeripheralDelegate {

    var manager:CBCentralManager!
    var bluetoothReady = false;
    var connectedDevice:CBPeripheral!
    var charTopic:CBCharacteristic!
    private var uuidSerialService = "8D164112-D095-90E1-C251-78C2D97DE669"
    private var txSerialService = "8D164113-D095-90E1-C251-78C2D97DE669"
    private var rxSerialService = "8D164114-D095-90E1-C251-78C2D97DE669"

    @IBOutlet weak var btConnectedLbl: UILabel!
    @IBOutlet weak var connectControllerBtn: UIButton!
    
    @IBOutlet weak var goToControllerBtn: UIButton!
    @IBAction func connectController(_ sender: UIButton) {
        if(bluetoothReady) {
            discoverDevices()
        }
        if charTopic != nil && connectedDevice != nil {
            let mystr = "Already Connected\n"
            let data = mystr.data(using: String.Encoding.utf8)
            connectedDevice.writeValue(data!, for: charTopic, type: CBCharacteristicWriteType.withoutResponse)
        }
    }
    @IBOutlet weak var iphoneImg: UIImageView!
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        connectControllerBtn.layer.cornerRadius = 5
        goToControllerBtn.layer.cornerRadius = 5
        btConnectedLbl.isHidden = true
        //goToControllerBtn.isEnabled = false
        goToControllerBtn.alpha = 0.5
        manager = CBCentralManager(delegate: self, queue: nil)
    }
    
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        if segue.identifier == "controllerSeg" {
            if let secondVC = segue.destination as? SecondViewController {
                secondVC.charTopic = charTopic
                secondVC.connectedDevice = connectedDevice
            }
            
        }
    }

    func scanForPeripherals(withServices: [CBUUID]?, options: [String : Any]? = nil) {
        print("Bluetooth found")
    }

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        var consoleMsg = ""
        switch (central.state) {
        case.poweredOff:
            consoleMsg = "BLE powered off"
        case.poweredOn:
            consoleMsg = "BLE powered on"
            bluetoothReady = true;
        case.resetting:
            consoleMsg = "BLE resetting"
        case.unauthorized:
            consoleMsg = "BLE is unauthorized"
        case.unknown:
            consoleMsg = "BLE is unknown"
        case.unsupported:
            consoleMsg = "BLE not supported"
        }
        print("\(consoleMsg)")
    }
    
    func discoverDevices() {
        manager.scanForPeripherals(withServices: [CBUUID(string:"FFE0")], options: nil)
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        connectedDevice = peripheral
        peripheral.delegate = self
        manager.connect(peripheral, options: nil)
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        manager.stopScan()
        peripheral.discoverServices(nil)
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if peripheral.services != nil {
            for service in peripheral.services! {
                peripheral.discoverCharacteristics([CBUUID(string:"FFE1")], for: service)
                print("Service found: \(service.uuid)")
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        let mystr = "BT Device Connected"
        let data = mystr.data(using: String.Encoding.utf8)
        for characteristic in service.characteristics! {
            charTopic = characteristic
            print("Characteristic found")
            peripheral.writeValue(data!, for: characteristic, type: CBCharacteristicWriteType.withoutResponse)
            btConnectedLbl.isHidden = false
            goToControllerBtn.isEnabled = true
            goToControllerBtn.alpha = 1
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }


}

