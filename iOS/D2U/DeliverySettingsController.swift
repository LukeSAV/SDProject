//
//  DeliverySettingsController.swift
//  D2U
//
//  Created by Luke Armbruster on 12/23/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import MapKit
import UIKit

class DeliverySettingsController: UIViewController {
    
    @IBOutlet weak var deliverBtn: UIButton!
    
    @IBOutlet weak var currentLocationLbl: UILabel!
    @IBOutlet weak var currentLandmarkLbl: UILabel!
    @IBOutlet weak var currentDistanceLbl: UILabel!
    @IBOutlet weak var etaLabel: UILabel!
    
    var syncTimer: Timer!
    
    private func disableDeliveryBtn() {
        //deliverBtn.isEnabled = false
        deliverBtn.alpha = 1.0
        deliverBtn.setTitle("STOP DELIVERY", for: .normal)
    }
    
    private func enableDeliveryBtn() {
        //deliverBtn.isEnabled = true
        deliverBtn.alpha = 1.0
        deliverBtn.setTitle("DELIVER", for: .normal)

    }
    
    @IBAction func deliverBtn(_ sender: UIButton) {
        if DeliveryInformation.deliveryInformation.delivering {
            enableDeliveryBtn()
        } else {
            disableDeliveryBtn()
        }
        ref.updateChildValues(["Delivery Requested" : !DeliveryInformation.deliveryInformation.delivering]) // Signal the device user requests delivery
        DeliveryInformation.deliveryInformation.delivering = !DeliveryInformation.deliveryInformation.delivering
    }
    
    private func handleViewingEvents() {
        if DeliveryInformation.deliveryInformation.delivering {
            self.disableDeliveryBtn()
        }
        self.currentLocationLbl.text = String(format: "%.4f, %.4f", DeliveryInformation.deliveryInformation.curCoord?.latitude ?? 0.0, DeliveryInformation.deliveryInformation.curCoord?.longitude ?? 0.0)
        self.currentLandmarkLbl.text = DeliveryInformation.deliveryInformation.nearestLandmark
        self.currentDistanceLbl.text = DeliveryInformation.deliveryInformation.distanceRemaining
        self.etaLabel.text = DeliveryInformation.deliveryInformation.eta
    }
    
    @objc func sync() {
        handleViewingEvents()
    }
    
    override func viewDidLoad() {
        handleViewingEvents()
        syncTimer = Timer.scheduledTimer(timeInterval: 1.0, target: self, selector: #selector(self.sync), userInfo: nil, repeats: true) // Update UI stuff about every second
    }
}
