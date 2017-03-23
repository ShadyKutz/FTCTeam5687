package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Blue (L TURN, L BEACON)", group = "Real")
@Disabled
public class BlueBeaconAutonomous extends BaseBeaconAutonmous {
    public BlueBeaconAutonomous()
    {
        super(AllianceColor.Left, BeaconSide.LEFT);
    }

}

