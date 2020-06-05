package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;


/**
 * Drives the robot in percent mode, based on the joystick values.
 */

public class Drivetrain_Percent implements Behavior {

    private static final Logger sLogger = LogManager.getLogger(Drivetrain_Percent.class);
    private static final Set<String> sSubsystems = Set.of("ss_drivetrain");

    private final InputValues fSharedInputValues;
    private final OutputValues fSharedOutputValues;
    private final String fXAxis;
    private final String fYAxis;

    private String mStateName;

    public Drivetrain_Percent(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        fSharedInputValues = inputValues;
        fSharedOutputValues = outputValues;
        fXAxis = robotConfiguration.getString("global_drivetrain", "x");
        fYAxis = robotConfiguration.getString("global_drivetrain", "y");

        mStateName = "Unknown";
    }

    @Override
    public void initialize(String stateName, Config config) {
        sLogger.debug("Entering state {}", stateName);

        mStateName = stateName;
    }

    @Override
    public void update() {
        double xAxis = fSharedInputValues.getNumeric(fXAxis);
        double yAxis = fSharedInputValues.getNumeric(fYAxis);

        // Set the motor speed to the joystick values
        double leftMotorSpeed = yAxis + xAxis;
        double rightMotorSpeed = yAxis - xAxis;

        //Because the joystick values combined can exceed the range the motors except (-1 to 1) limit them to with in that range
        if (leftMotorSpeed > 1) {
            rightMotorSpeed = rightMotorSpeed - (leftMotorSpeed - 1);
            leftMotorSpeed = 1;
        } else if (leftMotorSpeed < -1) {
            rightMotorSpeed = rightMotorSpeed - (1 + leftMotorSpeed);
            leftMotorSpeed = -1;
        } else if (rightMotorSpeed > 1) {
            leftMotorSpeed = leftMotorSpeed - (rightMotorSpeed - 1);
            rightMotorSpeed = 1;
        } else if (rightMotorSpeed < -1) {
            leftMotorSpeed = leftMotorSpeed - (1 + rightMotorSpeed);
            rightMotorSpeed = -1;
        }

        // Set the motors
        fSharedOutputValues.setNumeric("opn_drivetrain_left", "percent", leftMotorSpeed);
        fSharedOutputValues.setNumeric("opn_drivetrain_right", "percent", rightMotorSpeed);
    }


    @Override
    public void dispose() {
        sLogger.trace("Leaving state {}", mStateName);
        fSharedOutputValues.setNumeric("opn_drivetrain_left", "percent", 0.0);
        fSharedOutputValues.setNumeric("opn_drivetrain_right", "percent", 0.0);
        fSharedOutputValues.setBoolean("opb_drivetrain_gear_shifter", false);
    }

    @Override
    public boolean isDone() {
        return true;
    }

    @Override
    public Set<String> getSubsystems() {
        return sSubsystems;
    }
}