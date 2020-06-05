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

public class Drivetrain_Swerve implements Behavior {

    private static final Logger sLogger = LogManager.getLogger(Drivetrain_Swerve.class);
    private static final Set<String> sSubsystems = Set.of("ss_drivetrain");

    private final InputValues fSharedInputValues;
    private final OutputValues fSharedOutputValues;
    private final String fXAxis_left_js;
    private final String fYAxis_left_js;
    private final String fXAxis_right_js;
    private final String fYAxis_right_js;

    private final double fRobotLength;
    private final double fRobotWidth;
    private final double fRadius;

    private String mStateName;

    public Drivetrain_Swerve(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        fSharedInputValues = inputValues;
        fSharedOutputValues = outputValues;
        fXAxis_left_js = robotConfiguration.getString("global_drivetrain", "x_left_js");
        fYAxis_left_js = robotConfiguration.getString("global_drivetrain", "y_left_js");
        fXAxis_right_js = robotConfiguration.getString("global_drivetrain", "x_right_js");
        fYAxis_right_js = robotConfiguration.getString("global_drivetrain", "y_right_js");

        fRobotLength = robotConfiguration.getDouble("global_drivetrain", "robot_length");
        fRobotWidth = robotConfiguration.getDouble("global_drivetrain", "robot_width");
        fRadius = Math.sqrt ((fRobotLength * fRobotLength) + (fRobotWidth * fRobotWidth));


        mStateName = "Unknown";
    }

    @Override
    public void initialize(String stateName, Config config) {
        sLogger.debug("Entering state {}", stateName);

        mStateName = stateName;
    }

    @Override
    public void update() {
        double xAxis_left_js = fSharedInputValues.getNumeric(fXAxis_left_js);
        double yAxis_left_js = -1 * fSharedInputValues.getNumeric(fYAxis_left_js);
        double xAxis_right_js = fSharedInputValues.getNumeric(fXAxis_right_js);
        double yAxis_right_js = fSharedInputValues.getNumeric(fYAxis_right_js);

        double a = xAxis_left_js - xAxis_right_js * (fRobotLength / fRadius);
        double b = xAxis_left_js + xAxis_right_js * (fRobotLength / fRadius);
        double c = yAxis_left_js - xAxis_right_js * (fRobotWidth / fRadius);
        double d = yAxis_left_js + xAxis_right_js * (fRobotWidth / fRadius);

        double frontRightMotorSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftMotorSpeed = Math.sqrt ((b * b) + (c * c));
        double backLeftMotorSpeed = Math.sqrt ((a * a) + (c * c));
        double backRightMotorSpeed = Math.sqrt ((a * a) + (d * d));

        double frontRightMotorAngle = Math.atan2 (b, d) * 180 / Math.PI;
        double frontLeftMotorAngle = Math.atan2 (b, c) * 180 / Math.PI;
        double backLeftMotorAngle = Math.atan2 (a, c) * 180 / Math.PI;
        double backRightMotorAngle = Math.atan2 (a, d) * 180 / Math.PI;

        double maxSpeed = frontRightMotorSpeed;
        if(frontLeftMotorSpeed > maxSpeed) {
            maxSpeed = frontLeftMotorSpeed;
        }
        if(backLeftMotorSpeed > maxSpeed) {
            maxSpeed = backLeftMotorSpeed;
        }
        if(backRightMotorSpeed > maxSpeed) {
            maxSpeed = backRightMotorSpeed;
        }
        if(maxSpeed >  1) {
            frontRightMotorSpeed /= maxSpeed; 
            frontLeftMotorSpeed /= maxSpeed;
            backLeftMotorSpeed /= maxSpeed;
            backRightMotorSpeed /= maxSpeed;
        }


        // Set the motors
        fSharedOutputValues.setNumeric("opn_drivetrain_front_right_speed", "percent", frontRightMotorSpeed);
        fSharedOutputValues.setNumeric("opn_drivetrain_front_left_speed", "percent", frontLeftMotorSpeed);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_left_speed", "percent", backLeftMotorSpeed);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_right_speed", "percent", backRightMotorSpeed);

        fSharedOutputValues.setNumeric("opn_drivetrain_front_right_angle", "percent", frontRightMotorAngle);
        fSharedOutputValues.setNumeric("opn_drivetrain_front_left_angle", "percent", frontLeftMotorAngle);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_left_angle", "percent", backLeftMotorAngle);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_right_angle", "percent", backRightMotorAngle);

    }


    @Override
    public void dispose() {
        sLogger.trace("Leaving state {}", mStateName);
        fSharedOutputValues.setNumeric("opn_drivetrain_front_right_speed", "percent", 0);
        fSharedOutputValues.setNumeric("opn_drivetrain_front_left_speed", "percent", 0);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_left_speed", "percent", 0);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_right_speed", "percent", 0);
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