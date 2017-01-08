package com.team5687.helpers;

import com.team5687.Constants;

public class GeneralHelpers {
    public static double CalculateDistanceEncode(double inches) {
        return (inches * Constants.COUNTS_PER_INCH);
    }
}
