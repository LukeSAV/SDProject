//
// MapMainController.swift
//  D2U
//
//  Created by Luke Armbruster on 12/14/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import FirebaseDatabase
import MapKit
import UIKit

var ref:DatabaseReference! // This is initialized in the AppDelegate class
let defaultCoord = CLLocationCoordinate2D(latitude: 40.425021, longitude: -86.914325) // Default to a special location
class MapMainController: UIViewController, CLLocationManagerDelegate, MKMapViewDelegate {

    @IBOutlet weak var mapView: MKMapView!
    @IBOutlet weak var centerOnUserBtn: UIButton!
    @IBOutlet weak var centerOnRobotBtn: UIButton!
    @IBOutlet weak var centerOnDestinationBtn: UIButton!
    @IBOutlet weak var freeFloatBtn: UIButton!
    
    
    @IBAction func centerOnUserBtn(_ sender: UIButton) {
        MapMainSettings.mapMainSettings.mapCenter = MapMainSettings.MapCenter.user
        centerMap()
    }
    
    @IBAction func centerOnRobotBtn(_ sender: UIButton) {
        MapMainSettings.mapMainSettings.mapCenter = MapMainSettings.MapCenter.robot
        centerMap()
    }
    
    @IBAction func centerOnDestinationBtn(_ sender: UIButton) {
        MapMainSettings.mapMainSettings.mapCenter = MapMainSettings.MapCenter.destination
        centerMap()
    }
    
    @IBAction func freeFloatBtn(_ sender: UIButton) {
        MapMainSettings.mapMainSettings.mapCenter = MapMainSettings.MapCenter.free
        centerMap()
    }
    
    
    let locationManager:CLLocationManager = CLLocationManager() // Create the location manager when the VC gets loaded
    var userLocation:Waypoint = Waypoint(coordinate: defaultCoord)// Used to show current user location (all calculations for the robot are done relative to the destination) - initially same as approximate center of Purdue campus
    
    override func viewDidLoad() {
        super.viewDidLoad()
        locationManager.delegate = self // Set the CLLocationManager delegate to this object
        
        // Set up MapView object
        mapView.delegate = self // Set the MKMapView delegate to this object
        mapView.mapType = MKMapType.satellite // Satellite view
        mapView.setRegion(MKCoordinateRegion(center: defaultCoord, latitudinalMeters: 2500, longitudinalMeters: 2500), animated: true) // Center on Purdue campus
        
        // Make a compass that's always visible
        mapView.showsCompass = false
        let staticCompass = MKCompassButton(mapView: mapView)
        staticCompass.compassVisibility = .visible
        staticCompass.translatesAutoresizingMaskIntoConstraints = false
        mapView.addSubview(staticCompass)
        staticCompass.trailingAnchor.constraint(equalTo: mapView.trailingAnchor, constant: -8).isActive = true
        staticCompass.topAnchor.constraint(equalTo: mapView.topAnchor, constant: 8).isActive = true
        
        // Make scaleview
        let scaleView = MKScaleView(mapView: mapView)
        scaleView.scaleVisibility = .visible
        scaleView.translatesAutoresizingMaskIntoConstraints = false
        mapView.addSubview(scaleView)
        scaleView.leadingAnchor.constraint(equalTo: mapView.leadingAnchor, constant: 8).isActive = true
        scaleView.bottomAnchor.constraint(equalTo: mapView.bottomAnchor, constant: -80).isActive = true
        locationManager.requestAlwaysAuthorization() // Gives app location data at any time
        locationManager.startUpdatingLocation() // Gets updates from the LocationManager
        
        centerMap()
    }
    
    
    
    /*
     Called every time the device location is updated
     */
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        //let location = CLLocationCoordinate2D(latitude: 40.425021, longitude: -86.914325)
        userLocation = Waypoint(coordinate: locations[0].coordinate, color: DeliveryInformation.deliveryInformation.userLocColor)
        self.drawUserPin()
    }

    private func drawUserPin() {
        self.mapView.annotations.forEach {
            if $0 is Waypoint {
                let waypoint = $0 as? Waypoint
                if waypoint?.color == DeliveryInformation.deliveryInformation.userLocColor { // This is a user pin
                    self.mapView.removeAnnotation($0)
                }
            }
        }
        self.mapView.addAnnotation(userLocation)
    }
    
    func centerMap() {
        var centerCoord:CLLocationCoordinate2D
        switch MapMainSettings.mapMainSettings.mapCenter {
        case .user:
            centerCoord = userLocation.coordinate
            centerOnUserBtn.setBackgroundImage(UIImage(named: "WalkingMan_gold.png"), for: UIControl.State.normal)
            centerOnRobotBtn.setBackgroundImage(UIImage(named: "Robot_gray.png"), for: UIControl.State.normal)
            centerOnDestinationBtn.setBackgroundImage(UIImage(named: "Destination_gray.png"), for: UIControl.State.normal)
            freeFloatBtn.setBackgroundImage(UIImage(named: "Earth_gray.png"), for: UIControl.State.normal)
            print("Centering on user")
        case .robot:
            centerCoord = DeliveryInformation.deliveryInformation.curCoord ?? defaultCoord
            centerOnUserBtn.setBackgroundImage(UIImage(named: "WalkingMan_gray.png"), for: UIControl.State.normal)
            centerOnRobotBtn.setBackgroundImage(UIImage(named: "Robot_gold.png"), for: UIControl.State.normal)
            centerOnDestinationBtn.setBackgroundImage(UIImage(named: "Destination_gray.png"), for: UIControl.State.normal)
            freeFloatBtn.setBackgroundImage(UIImage(named: "Earth_gray.png"), for: UIControl.State.normal)
            print("Centering on robot")
        case .destination:
            centerCoord = DeliveryInformation.deliveryInformation.endCoord ?? defaultCoord
            centerOnUserBtn.setBackgroundImage(UIImage(named: "WalkingMan_gray.png"), for: UIControl.State.normal)
            centerOnRobotBtn.setBackgroundImage(UIImage(named: "Robot_gray.png"), for: UIControl.State.normal)
            centerOnDestinationBtn.setBackgroundImage(UIImage(named: "Destination_gold.png"), for: UIControl.State.normal)
            freeFloatBtn.setBackgroundImage(UIImage(named: "Earth_gray.png"), for: UIControl.State.normal)
            print("Centering on destination")
        case .free:
            centerOnUserBtn.setBackgroundImage(UIImage(named: "WalkingMan_gray.png"), for: UIControl.State.normal)
            centerOnRobotBtn.setBackgroundImage(UIImage(named: "Robot_gray.png"), for: UIControl.State.normal)
            centerOnDestinationBtn.setBackgroundImage(UIImage(named: "Destination_gray.png"), for: UIControl.State.normal)
            freeFloatBtn.setBackgroundImage(UIImage(named: "Earth_gold.png"), for: UIControl.State.normal)
            print("Free roam")
            return
        }
        mapView.setRegion(MKCoordinateRegion(center: centerCoord, latitudinalMeters: 50, longitudinalMeters: 50), animated: true)
    }
}

