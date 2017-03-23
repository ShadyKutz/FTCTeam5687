package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Red (R TURN, R BEACON)", group = "Real")
public class RightRedBeaconAutonomous extends BaseBeaconAutonmous {
    public RightRedBeaconAutonomous()
    {
        super(AllianceColor.Right, BeaconSide.RIGHT);
    }

}
