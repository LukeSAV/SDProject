//
//  CDJoystick.swift
//  CDJoystick
//
//  Created by Cole Dunsby on 2015-12-21.
//  Copyright © 2016 Cole Dunsby. All rights reserved.
//

import WatchKit
import Foundation

class WatchDriver: NSObject, WKCrownDelegate {
    public var crownRotationalDelta = 0.0

    public func moveUp () {

    }
    public func moveDown() {
        
    }
    
    func crownDidRotate(_ crownSequencer: WKCrownSequencer?, rotationalDelta: Double) {
        crownRotationalDelta += rotationalDelta
        print(crownRotationalDelta)
    }
}
