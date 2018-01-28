//
//  ViewController.swift
//  SDProject
//
//  Created by Luke Armbruster on 1/27/18.
//  Copyright © 2018 Luke Armbruster. All rights reserved.
//

import UIKit

class ViewController: UIViewController {

    @IBOutlet weak var connectControllerBtn: UIButton!
    @IBOutlet weak var connectVideoBtn: UIButton!
    @IBOutlet weak var iphoneImg: UIImageView!
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        connectControllerBtn.layer.cornerRadius = 5;
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }


}

