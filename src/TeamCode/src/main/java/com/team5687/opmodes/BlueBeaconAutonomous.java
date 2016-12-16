package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Beacon", group = "Real")
public class BlueBeaconAutonomous extends BaseBeaconAutonmous {
    public BlueBeaconAutonomous()
    {
        super(AllianceColor.BLUE);
    }

}
