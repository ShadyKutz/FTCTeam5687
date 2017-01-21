package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue (L TURN, R BEACON)", group = "Real")
public class RightBlueBeaconAutonomous extends BaseBeaconAutonmous {
    public RightBlueBeaconAutonomous()
    {
        super(AllianceColor.Left, BeaconSide.RIGHT);
    }

}
