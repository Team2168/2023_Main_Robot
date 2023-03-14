package org.team2168;

import org.team2168.Constants.Joysticks;
import org.team2168.utils.F310;
import org.team2168.utils.LinearInterpolator;


/*NOTE: This is the OI used during 2022, updates, removals, and changes will be made to this code over time but for now, things are being
kept the same. */

public class OI {

    //the three objects being created are the driver, opeartor, and test joysticks which have different doubles and get commands

    public final F310 driverJoystick = new F310(Joysticks.DRIVER_JOYSTICK);
    public final F310 operatorJoystick = new F310(Joysticks.OPERATOR_JOYSTICK);
    public final F310 testJoystick = new F310(Joysticks.PID_TEST_JOYSTICK);

    private LinearInterpolator driverJoystickInterpolator;
    private LinearInterpolator operatorJoystickInterpolator;
    private LinearInterpolator gunStyleXInterpolator;
    private LinearInterpolator gunStyleYInterpolator;
    private LinearInterpolator testJoystickXInterpolator;
    private static OI instance = null;

    /**
     *  A double list used for the driver joystick to interpolate
     */
    
    
    private double[][] driverJoystickInterpolation = {
        {-1.00, -1.00},
        {-0.05,  0.00},  //set neutral deadband to 5%
        {+0.05,  0.00},
        {+1.00, +1.00}  
    };

    /**
     * A double list used by the wheel of the gun style controller for interpolation (presumbly)
     */

    private double[][] gunStyleXInterpolation = {
        {-1.00, -0.70},  //scale down turning to max 70%
        {-0.05,  0.00},  //set neutral deadband to 5%
        {+0.05,  0.00},
        {+1.00, +0.70}  
    };
    /**
     * A double list used by the trigger of the gun style controller for interpolation (presumbly)
     */

    private double[][] gunStyleYInterpolation = {
		{-1.00, -1.00}, //can limit speed by changing second number
		{-0.15,  0.00},
		{+0.15,  0.00},
		{+1.00, +1.00}
	};
    /**
     * A double list used for the operator joystick to interpolate
     */

    private double[][] operatorJoystickInterpolation = {
        {-1.00, -0.075},
        {-0.05,  0.0},  //set neutral deadband to -10%
        {+0.05,  0.0},
        {+1.00, +0.075}  
    };
    /**
     * A double list used for the test joystick to interpolate 
     */

    private double[][] testJoystickInterpolation = {
        {-1.00, -0.50},
        {-0.01, 0.00},
        {+0.01, 0.00},
        {+1.00, +0.50}
    };

    private OI() {
        driverJoystickInterpolator = new LinearInterpolator(driverJoystickInterpolation);
        gunStyleXInterpolator = new LinearInterpolator(gunStyleXInterpolation);
        gunStyleYInterpolator = new LinearInterpolator(gunStyleYInterpolation);
        operatorJoystickInterpolator = new LinearInterpolator(operatorJoystickInterpolation);
        testJoystickXInterpolator = new LinearInterpolator(testJoystickInterpolation);
    }

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }

    public double getDriverJoystickX() {
        return driverJoystickInterpolator.interpolate(driverJoystick.getRightStickRaw_X());
    }

    public double getDriverJoystickY() {
        return driverJoystickInterpolator.interpolate(driverJoystick.getRightStickRaw_Y());
    }

    public double getLeftDriverJoystickX() {
        return driverJoystickInterpolator.interpolate(driverJoystick.getLeftStickRaw_X());
    }

    public double getLeftDriverJoystickY() {
        return driverJoystickInterpolator.interpolate(driverJoystick.getLeftStickRaw_Y());
    }

    public double getGunStyleWheel() {
        return gunStyleXInterpolator.interpolate(driverJoystick.getLeftStickRaw_X());
    }

    public double getGunStyleTrigger() {
        return gunStyleYInterpolator.interpolate(driverJoystick.getLeftStickRaw_Y());
    }

    public double getLeftOperatorJoystickY() {
        return operatorJoystickInterpolator.interpolate(operatorJoystick.getLeftStickRaw_Y());
    }

    public double getLeftOperatorJoystickX() {
        return operatorJoystickInterpolator.interpolate(operatorJoystick.getLeftStickRaw_X());
    }

    public double getRightOperatorJoystickY() {
        return operatorJoystickInterpolator.interpolate(operatorJoystick.getRightStickRaw_Y());
    }

    public double getRightOperatorJoystickX() {
        return operatorJoystickInterpolator.interpolate(operatorJoystick.getRightStickRaw_X());
    }



    public double getTestJoystickX() {
        return testJoystickXInterpolator.interpolate(testJoystick.getLeftStickRaw_X());
    }
    
}