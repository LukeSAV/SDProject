//
//  MapMainSettings.swift
//  D2U
//
//  Created by Luke Armbruster on 12/24/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import Foundation

class MapMainSettings {
    
    enum MapCenter {
        case user
        case robot
        case destination
        case free
    }
    
    static let mapMainSettings = MapMainSettings()
    var mapCenter = MapCenter.free
    
    private init() {
        
    }
}
