//
//  ViewController.swift
//  D2U
//
//  Created by Luke Armbruster on 12/14/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import UIKit
import FirebaseDatabase

class ViewController: UIViewController {

    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        
        let ref = Database.database().reference()
        
        ref.observe(DataEventType.value, with: { (snapshot) in
            let postDict = snapshot.value as? [String : AnyObject] ?? [:]
            print(snapshot.value)
        })
    }


}

