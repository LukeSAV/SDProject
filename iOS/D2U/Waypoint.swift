//
//  Waypoint.swift
//  D2U
//
//  Created by Luke Armbruster on 12/24/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//
//  This class is derived from MKPointAnnotation to apply color to dropped pins.

import MapKit

class Waypoint: MKPointAnnotation {
    
    var color:UIColor = UIColor(red: 255.0/255.0, green: 100.0/255.0, blue: 115.0/255.0, alpha: 1.0)
    
    init(coordinate:CLLocationCoordinate2D) {
        super.init()
        self.coordinate = coordinate
    }
    
    init(coordinate:CLLocationCoordinate2D, color:UIColor) {
        super.init()
        self.coordinate = coordinate
        self.color = color
    }
}
