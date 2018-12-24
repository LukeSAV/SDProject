//
//  DeliveryLocation.swift
//  
//
//  Created by Luke Armbruster on 12/23/18.
//

import MapKit

class DeliveryLocation {
    
    static let deliveryLocation = DeliveryLocation()
    
    var coord:CLLocationCoordinate2D?
    
    let PMUFrontLawnLat = 40.424449
    let PMUFrontLawnLon = -86.911106
    
    private init() {
        
    }
    
    public func setDeliveryLocation(lat:Double, lon:Double) {
        self.coord = CLLocationCoordinate2D(latitude: lat, longitude: lon)
    }
    
}
