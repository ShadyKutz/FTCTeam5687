package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "REDFLIPPER", group = "Real")
public class FLIPPER_RED extends ballarmandflick {
    public FLIPPER_RED()
    {
        super(AllianceColor.Left, BeaconSide.LEFT);
    }

}