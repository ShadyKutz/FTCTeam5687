package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by stephen on 15/12/16.
 */

@Autonomous(name = "Red Beacon", group = "Real")
public class RedBeaconAutonomous extends BaseBeaconAutonmous {
    public RedBeaconAutonomous()
    {
        super(AllianceColor.RED);
    }

}

