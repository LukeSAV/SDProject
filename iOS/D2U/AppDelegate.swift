//
//  AppDelegate.swift
//  D2U
//
//  Created by Luke Armbruster on 12/14/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import UIKit
import Firebase
import MapKit
@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate {

    var window: UIWindow?
    var syncTimer: Timer!

    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        // Override point for customization after application launch.
        FirebaseApp.configure()
        ref = Database.database().reference() // Reference to the D2U database specified in pod file
        /* THIS MAY NEED TO CHANGE TO HANDLE POSSIBILITY OF APP CRASHING */
        ref.updateChildValues(["Delivery Requested" : false]) // Ensure that delivery is not currently active
        syncTimer = Timer.scheduledTimer(timeInterval: 1.0, target: self, selector: #selector(self.sync), userInfo: nil, repeats: true) // Poll firebase changes about every second
        return true
    }

    func applicationWillResignActive(_ application: UIApplication) {
        // Sent when the application is about to move from active to inactive state. This can occur for certain types of temporary interruptions (such as an incoming phone call or SMS message) or when the user quits the application and it begins the transition to the background state.
        // Use this method to pause ongoing tasks, disable timers, and invalidate graphics rendering callbacks. Games should use this method to pause the game.
    }

    func applicationDidEnterBackground(_ application: UIApplication) {
        // Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later.
        // If your application supports background execution, this method is called instead of applicationWillTerminate: when the user quits.
    }

    func applicationWillEnterForeground(_ application: UIApplication) {
        // Called as part of the transition from the background to the active state; here you can undo many of the changes made on entering the background.
    }

    func applicationDidBecomeActive(_ application: UIApplication) {
        // Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
    }

    func applicationWillTerminate(_ application: UIApplication) {
        // Called when the application is about to terminate. Save data if appropriate. See also applicationDidEnterBackground:.
    }

    @objc func sync() {
        ref.child("Delivery Requested").observeSingleEvent(of: .value, with: { (snapshot) in
            DeliveryInformation.deliveryInformation.delivering = snapshot.value as! Bool
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
                DeliveryInformation.deliveryInformation.setCurrentLocation(lat: latitude, lon: longitude)
                print(gpggaArray)
            }
            
        })
        ref.child("Nearest Landmark").observeSingleEvent(of: .value, with: { (snapshot) in
            DeliveryInformation.deliveryInformation.nearestLandmark = snapshot.value as! String
        })
        ref.child("Distance To Destination").observeSingleEvent(of: .value, with: { (snapshot) in
            DeliveryInformation.deliveryInformation.distanceRemaining = snapshot.value as! String
        })
        ref.child("ETA").observeSingleEvent(of: .value, with: { (snapshot) in
            DeliveryInformation.deliveryInformation.eta = snapshot.value as! String
        })
        ref.child("Next Waypoint").observeSingleEvent(of: .value, with: { (snapshot) in
            let value = snapshot.value as! String
            let waypointArray = value.split(separator: ",")
            if waypointArray.count > 2 {
                DeliveryInformation.deliveryInformation.nextWaypointCoord = CLLocationCoordinate2D(latitude: Double(waypointArray[1])!, longitude: Double(waypointArray[2])!)
            }
            
        })
        ref.child("Side of Line").observeSingleEvent(of: .value, with: { (snapshot) in
            let value = snapshot.value as? String
            if value != nil {
                let lineArray = value!.split(separator: ",")
                if lineArray.count > 1 {
                    DeliveryInformation.deliveryInformation.sideOfLine = String(lineArray[0])
                    DeliveryInformation.deliveryInformation.distanceFromLine = Double(lineArray[1])!
                }
            }
        })
    }
    
}

