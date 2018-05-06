//
//  InterfaceController.swift
//  Watch Extension
//
//  Created by Luke Armbruster on 3/26/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import WatchKit
import Foundation
import CoreBluetooth

class InterfaceController: WKInterfaceController, CBCentralManagerDelegate, CBPeripheralDelegate {
    var manager:CBCentralManager!
    var charTopic:CBCharacteristic!
    var connectedDevice:CBPeripheral!
    var bluetoothReady = false;
    
    @IBOutlet var BTConnLbl: WKInterfaceLabel!
    @IBOutlet var nextBtn: WKInterfaceButton!
    /*
    * @brief Search for Bluetooth devices if watch is ready
    */
    @IBAction func BTConnBtn() {
        if(bluetoothReady) {
            discoverDevices()
        } else {
            print("Bluetooth not ready")
        }
        if charTopic != nil && connectedDevice != nil {
            print("Device alread connected")
        }
    }
    override func awake(withContext context: Any?) {
        super.awake(withContext: context)
        // Hide labels and deactivate button until BT is connected
        BTConnLbl.setHidden(true)
        nextBtn.setAlpha(0.3)
        nextBtn.setEnabled(false)
        // Start CBCentralManager for watch
        manager = CBCentralManager(delegate: self, queue: nil)
        // Cancel existing connection when interface initiates
        if(connectedDevice != nil) {
            manager.cancelPeripheralConnection(connectedDevice)
        }

    }
    
    override func willActivate() {
        // This method is called when watch view controller is about to be visible to user
        super.willActivate()
        if(connectedDevice != nil) {
            manager.cancelPeripheralConnection(connectedDevice)
        }
        BTConnLbl.setHidden(true)
        nextBtn.setAlpha(0.3)
        nextBtn.setEnabled(false)
    }
    
    override func didDeactivate() {
        // This method is called when watch view controller is no longer visible
        super.didDeactivate()
    }
    
    /*
    *  @brief Segues to second interface controller and sends BT info in dictionary
    */
    override func contextForSegue(withIdentifier segueIdentifier: String) -> Any? {
        if segueIdentifier == "Next" {
            return ["manager": manager, "connectedDevice": connectedDevice, "charTopic": charTopic]
        }
        else {
            return ["manager": "", "connectedDevice": "", "charTopic": ""]
        }
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
    
    /*
    *  @brief Searches for bluetooth devices to connect to matching CBUUID
    */
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
        for characteristic in service.characteristics! {
            charTopic = characteristic
            print("Characteristic found")
            BTConnLbl.setHidden(false)
            nextBtn.setEnabled(true)
            nextBtn.setAlpha(1)
        }
    }
}
