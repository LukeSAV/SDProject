//
//  MapMainSettings.swift
//  D2U
//
//  Created by Luke Armbruster on 12/24/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import Foundation
import UIKit
class MapMainSettings {
    
    enum MapCenter {
        case user
        case robot
        case destination
        case free
    }
    
    static let mapMainSettings = MapMainSettings()
    
    let destinationPin = UIImage(named: "Destination_pin.png")
    let robotPin = UIImage(named: "Robot_pin.png")
    let userPin = UIImage(named: "WalkingMan_pin.png")
    let nextWaypointPin = UIImage(named: "NextWaypoint.png")
    var mapCenter = MapCenter.free
    
    private init() {
        
    }
}
