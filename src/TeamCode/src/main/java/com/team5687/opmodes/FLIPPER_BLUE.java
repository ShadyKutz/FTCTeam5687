package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUEFLIPPER", group = "Real")
public class FLIPPER_BLUE extends ballarmandflick {
    public FLIPPER_BLUE()
    {
        super(AllianceColor.Right, BeaconSide.RIGHT);
    }

}