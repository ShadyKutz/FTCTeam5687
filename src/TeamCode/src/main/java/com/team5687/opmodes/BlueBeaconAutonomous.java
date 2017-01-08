package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue (L TURN, L BEACON)", group = "Real")
public class BlueBeaconAutonomous extends BaseBeaconAutonmous {
    public BlueBeaconAutonomous()
    {
        super(AllianceColor.BLUE, BeaconSide.LEFT);
    }

}

