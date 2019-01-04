//
//  DeliveryLocation.swift
//  
//
//  Created by Luke Armbruster on 12/23/18.
//

import MapKit

class DeliveryInformation {
    
    static let deliveryInformation = DeliveryInformation() // Singleton
    
    var endCoord:CLLocationCoordinate2D? // Destination coordinate
    var curCoord:CLLocationCoordinate2D? // Current coordinate of the robot
    
    let deliverLocColor:UIColor = UIColor(red: 255.0/255.0, green: 100.0/255.0, blue: 115.0/255.0, alpha: 1.0) // Color used to show location to be delivered to
    let userLocColor:UIColor = UIColor(red: 0.0/255.0, green: 100.0/255.0, blue: 115.0/255.0, alpha: 1.0)
    
    
    
    let PMUFrontLawnLat = 40.424449
    let PMUFrontLawnLon = -86.911106
    
    private init() {
        setDeliveryLocation(lat: PMUFrontLawnLat, lon: PMUFrontLawnLon)
    }
    
    public func setDeliveryLocation(lat:Double, lon:Double) {
        self.endCoord = CLLocationCoordinate2D(latitude: lat, longitude: lon)
    }
    
    public func setCurrentLocation(lat:Double, lon:Double) {
        self.curCoord = CLLocationCoordinate2D(latitude: lat, longitude: lon)
    }
    
}
