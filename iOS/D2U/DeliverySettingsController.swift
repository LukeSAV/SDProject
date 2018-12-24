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
        deliverBtn.isEnabled = false
        deliverBtn.alpha = 0.5
    }
    
    private func enableDeliveryBtn() {
        deliverBtn.isEnabled = true
        deliverBtn.alpha = 1.0
    }
    
    @IBAction func deliverBtn(_ sender: UIButton) {
        disableDeliveryBtn()
        ref.updateChildValues(["Delivery Requested" : true]) // Signal the device user requests delivery
    }
    
    private func handleViewingEvents() {
        ref.child("Delivery Requested").observeSingleEvent(of: .value, with: { (snapshot) in
            let value = snapshot.value as! Bool
            if value == false {
                self.enableDeliveryBtn()
            }
            else {
                self.disableDeliveryBtn()
            }
        })
        ref.child("Current Position").observeSingleEvent(of: .value, with: { (snapshot) in
            let value = snapshot.value as! String // GPGGA string to convert to lat/lon
            let gpggaArray = value.split(separator: ",")
            if gpggaArray.count >= 8  { // At least has all of the things I care about
                let latitudeStr = gpggaArray[2]
                let latIndex = latitudeStr.index(latitudeStr.startIndex, offsetBy: 2) // First two digits are degrees, rest are minutes
                let latitude = (Double(latitudeStr[..<latIndex]) ?? 40.0) + (Double(latitudeStr[latIndex...]) ?? 25.160) / 60.0
                let longitudeStr = gpggaArray[4]
                let lonIndex = longitudeStr.index(longitudeStr.startIndex, offsetBy: 3) // First three digits are degrees, rest are minutes
                let longitude = ((Double(longitudeStr[..<lonIndex]) ?? 86.0) + (Double(longitudeStr[lonIndex...]) ?? 54.400) / 60.0) * (gpggaArray[5].prefix(1) == "W" ? -1.0 : 1.0) // Multiply by -1 if west
                
                self.currentLocationLbl.text = String(format: "%.4f, %.4f", latitude, longitude)
            }
        })
        ref.child("Nearest Landmark").observeSingleEvent(of: .value, with: { (snapshot) in
            let value = snapshot.value as! String
            self.currentLandmarkLbl.text = value
        })
        ref.child("Distance To Destination").observeSingleEvent(of: .value, with: { (snapshot) in
            let value = snapshot.value as! String
            self.currentDistanceLbl.text = value
        })
        ref.child("ETA").observeSingleEvent(of: .value, with: { (snapshot) in
            let value = snapshot.value as! String
            self.etaLabel.text = value
        })
    }
    
    @objc func sync() {
        handleViewingEvents()
    }
    
    override func viewDidLoad() {
        handleViewingEvents()
        syncTimer = Timer.scheduledTimer(timeInterval: 1.0, target: self, selector: #selector(self.sync), userInfo: nil, repeats: true) // Update UI stuff about every second
    }
}
