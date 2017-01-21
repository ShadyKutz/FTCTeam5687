package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by stephen on 15/12/16.
 */

@Autonomous(name = "Red (R TURN, L BEACON)", group = "Real")
public class RedBeaconAutonomous extends BaseBeaconAutonmous {
    public RedBeaconAutonomous()
    {
        super(AllianceColor.Right, BeaconSide.LEFT);
    }

}


