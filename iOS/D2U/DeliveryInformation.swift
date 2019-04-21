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
    var nextWaypointCoord:CLLocationCoordinate2D? // Coordinate of the next waypoint for the robot
    var delivering = false
    var nearestLandmark = ""
    var distanceRemaining = ""
    var eta = ""
    var sideOfLine = ""
    var distanceFromLine = 0.0
    
    let deliverLocColor:UIColor = UIColor(red: 255.0/255.0, green: 100.0/255.0, blue: 115.0/255.0, alpha: 1.0) // Color used to show location to be delivered to
    let userLocColor:UIColor = UIColor(red: 0.0/255.0, green: 100.0/255.0, blue: 115.0/255.0, alpha: 1.0)
    let robotLocColor:UIColor = UIColor(red: 77.0/255.0, green: 64.0/255.0, blue: 56.0/255.0, alpha: 1.0)
    
    let defaultCoord = CLLocationCoordinate2D(latitude: 40.425021, longitude: -86.914325) // Default to a special location
    
    let PMUFrontLawnLat = 40.428388 // Changed to Hovde
    let PMUFrontLawnLon = -86.914188
    
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
