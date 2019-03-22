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
    
    @IBOutlet weak var distanceFromLineTxt: UITextField!
    
    
    var syncTimer: Timer!
    let locationManager:CLLocationManager = CLLocationManager() // Create the location manager when the VC gets loaded
    var userLocation:Waypoint = Waypoint(coordinate: defaultCoord, image: MapMainSettings.mapMainSettings.userPin!)// Used to show current user location (all calculations for the robot are done relative to the destination) - initially same as approximate center of Purdue campus
    
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
        
        syncTimer = Timer.scheduledTimer(timeInterval: 2.0, target: self, selector: #selector(self.sync), userInfo: nil, repeats: true) // Update UI stuff about every 2 seconds
    }
    
    
    
    /*
     Called every time the device location is updated
     */
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        //let location = CLLocationCoordinate2D(latitude: 40.425021, longitude: -86.914325)
        userLocation = Waypoint(coordinate: locations[0].coordinate, image: MapMainSettings.mapMainSettings.userPin!)
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
    
    /*
     Called when adding a new waypoint.
     */
    func mapView(_ mapView: MKMapView, viewFor annotation: MKAnnotation) -> MKAnnotationView? {
        if let annotation = annotation as? Waypoint {
            var annotationView:MKAnnotationView?
            if annotation.image == MapMainSettings.mapMainSettings.userPin {
                annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: "userPin")
                if annotationView == nil {
                    annotationView = MKAnnotationView(annotation: annotation, reuseIdentifier: "userPin")
                }
            }
            else if annotation.image == MapMainSettings.mapMainSettings.robotPin {
                annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: "robotPin")
                if annotationView == nil {
                    annotationView = MKAnnotationView(annotation: annotation, reuseIdentifier: "robotPin")
                }
            }
            else if annotation.image == MapMainSettings.mapMainSettings.destinationPin {
                annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: "destinationPin")
                if annotationView == nil {
                    annotationView = MKAnnotationView(annotation: annotation, reuseIdentifier: "destinationPin")
                }
            }
            else if annotation.image == MapMainSettings.mapMainSettings.nextWaypointPin {
                annotationView = mapView.dequeueReusableAnnotationView(withIdentifier: "nextWaypointPin")
                if annotationView == nil {
                    annotationView = MKAnnotationView(annotation: annotation, reuseIdentifier: "nextWaypointPin")
                }
            }
            else {
                let pinAnnotationView = MKPinAnnotationView(annotation: annotation, reuseIdentifier: "None")
                pinAnnotationView.pinTintColor = UIColor.black
                return pinAnnotationView
            }
            annotationView!.annotation = annotation
            annotationView!.canShowCallout = true
            annotationView!.image = annotation.image
            var transform = CGAffineTransform.identity
            transform = transform.translatedBy(x: 0, y: -20) // "ish" to make the pin points look good on the map
            transform = transform.scaledBy(x: 0.04, y: 0.04)
            annotationView!.transform = transform
            return annotationView
        }
        return nil
    }
    
    private func drawPin(pinType: UIImage, waypoint: Waypoint) {
        /* dequeueReusableAnnotation view seems to be broken and this is fairly cheap */
        self.mapView.annotations.forEach {
            if $0 is Waypoint {
                let testWaypoint = $0 as? Waypoint
                if testWaypoint?.image == pinType { // Pin type matches. Remove old one.
                    self.mapView.removeAnnotation($0)
                }
            }
        }
        self.mapView.addAnnotation(waypoint)
    }
    
    @objc func sync() {
        self.drawPin(pinType: MapMainSettings.mapMainSettings.userPin!, waypoint: userLocation)
        let robotLoc = Waypoint(coordinate: DeliveryInformation.deliveryInformation.curCoord ?? defaultCoord, image: MapMainSettings.mapMainSettings.robotPin!)
        self.drawPin(pinType: MapMainSettings.mapMainSettings.robotPin!, waypoint: robotLoc)
        let destinationLoc = Waypoint(coordinate: DeliveryInformation.deliveryInformation.endCoord ?? defaultCoord, image: MapMainSettings.mapMainSettings.destinationPin!)
        self.drawPin(pinType: MapMainSettings.mapMainSettings.destinationPin!, waypoint: destinationLoc)
        
        let nextWaypointLoc = Waypoint(coordinate: DeliveryInformation.deliveryInformation.nextWaypointCoord ?? defaultCoord, image: MapMainSettings.mapMainSettings.nextWaypointPin!)
        self.drawPin(pinType: MapMainSettings.mapMainSettings.nextWaypointPin!, waypoint: nextWaypointLoc)
        
        centerMap()
        
        distanceFromLineTxt.text = String(DeliveryInformation.deliveryInformation.distanceFromLine)
    }
}

