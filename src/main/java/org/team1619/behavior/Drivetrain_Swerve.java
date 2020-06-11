package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.*;


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
    private final double fDiameter;

    private double mPreviousfrontRightMotorAngle;
    private double mPreviousfrontLeftMotorAngle;
    private double mPreviousBackLefttMotorAngle;
    private double mPreviousBackRightMotorAngle;


    private final String fNavx;
    private Map<String, Double> fNavxValues = new HashMap<>();


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
        fDiameter = Math.sqrt ((fRobotLength * fRobotLength) + (fRobotWidth * fRobotWidth));

        mPreviousfrontRightMotorAngle = 0;
        mPreviousfrontLeftMotorAngle = 0;
        mPreviousBackLefttMotorAngle = 0;
        mPreviousBackRightMotorAngle = 0;

        fNavx = robotConfiguration.getString("global_drivetrain", "navx");

        mStateName = "Unknown";

    }

    @Override
    public void initialize(String stateName, Config config) {
        sLogger.debug("Entering state {}", stateName);

        mStateName = stateName;
    }

    @Override
    public void update() {

        // Read joysticks
        double strafe = fSharedInputValues.getNumeric(fXAxis_left_js);
        double forward = 1 * fSharedInputValues.getNumeric(fYAxis_left_js);
        double rotate = fSharedInputValues.getNumeric(fXAxis_right_js);
        double yAxis_right_js = -1 * fSharedInputValues.getNumeric(fYAxis_right_js);

        // Get heading from the Navx
        fNavxValues = fSharedInputValues.getVector(fNavx);
        double heading = fNavxValues.getOrDefault("yaw", 0.0);
        fSharedInputValues.setNumeric("opn_swerve_navx_heading", heading);

        // Field centric steering - adjust joysticks based on Navx heading
        if (fSharedInputValues.getBooleanRisingEdge("ipb_driver_start")) {
            fSharedInputValues.setBoolean("ipb_swerve_field_centric", !fSharedInputValues.getBoolean("ipb_swerve_field_centric"));
        }
        if (fSharedInputValues.getBoolean("ipb_swerve_field_centric")) {
            double temp = forward * Math.cos(heading) + strafe * Math.sin(heading);
            strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
            forward = temp;
        }

        // Use the right joystick to point the robot in a specific direction instead of spinning continuously
        if (fSharedInputValues.getBoolean("ipb_driver_right_stick_button")){
            // Calculate the direction the joystick is pointing
            double rightJoystickDirection = Math.atan2(rotate, yAxis_right_js) * 180 / Math.PI;
            fSharedInputValues.setNumeric("opn_swerve_right_joystick_direction", rightJoystickDirection);
            // Adjust rotation based on how far it needs to spin to get to the correct orientation
            double headingDiff = heading - rightJoystickDirection;
            fSharedInputValues.setNumeric("opn_swerve_heading_difference", rightJoystickDirection);
            rotate = headingDiff / 180;
            //todo - need way to increase roation value when close to zero to cause movement
        } else{
            fSharedInputValues.setNumeric("opn_swerve_right_joystick_direction", -9999);
            fSharedInputValues.setNumeric("opn_swerve_heading_difference", -9999);
        }

        // Output values for debugging
        fSharedInputValues.setNumeric("opn_swerve_forward", forward);
        fSharedInputValues.setNumeric("opn_swerve_strafe", strafe);
        fSharedInputValues.setNumeric("opn_swerve_rotate", rotate);


        double a = strafe - rotate * (fRobotLength / fDiameter);
        double b = strafe + rotate * (fRobotLength / fDiameter);
        double c = forward - rotate * (fRobotWidth / fDiameter);
        double d = forward + rotate * (fRobotWidth / fDiameter);

        // Calculate the wheel speed
        double frontRightMotorSpeed = Math.sqrt ((b * b) + (c * c));
        double frontLeftMotorSpeed = Math.sqrt ((b * b) + (d * d));
        double backLeftMotorSpeed = Math.sqrt ((a * a) + (d * d));
        double backRightMotorSpeed = Math.sqrt ((a * a) + (c * c));

        // Calculate the wheel angle
        double frontRightMotorAngle = Math.atan2 (b, c) * 180 / Math.PI;
        double frontLeftMotorAngle = Math.atan2 (b, d) * 180 / Math.PI;
        double backLeftMotorAngle = Math.atan2 (a, d) * 180 / Math.PI;
        double backRightMotorAngle = Math.atan2 (a, c) * 180 / Math.PI;

        // Normalize the wheel speed so they never exceed 1.0
        Double[] speeds = { frontRightMotorSpeed, frontLeftMotorSpeed, backLeftMotorSpeed, backRightMotorSpeed };
        double maxSpeed = Collections.max(Arrays.asList(speeds));
        if(maxSpeed >  1.0) {
            frontRightMotorSpeed /= maxSpeed;
            frontLeftMotorSpeed /= maxSpeed;
            backLeftMotorSpeed /= maxSpeed;
            backRightMotorSpeed /= maxSpeed;
        }

        // If rotation is greater than 90 degrees, rotate the other direction and reverse wheel speed
//        if(Math.abs(mPreviousfrontRightMotorAngle - frontRightMotorAngle) > 90){
//            frontRightMotorAngle = 180 - frontRightMotorAngle;
//            if(frontRightMotorAngle > 360){
//                frontRightMotorAngle = frontRightMotorAngle - 360;
//            }
//            frontRightMotorSpeed = -1 * frontRightMotorSpeed;
//        }

        // Rotate around one wheel
        if (fSharedInputValues.getBoolean("ipb_driver_dpad_up")){
            // Spin around left front wheel
            frontRightMotorSpeed = fRobotWidth / fDiameter;
            frontLeftMotorSpeed = 0;
            backLeftMotorSpeed = fRobotLength / fDiameter;
            backRightMotorSpeed = 1;
            frontRightMotorAngle = 0;
            frontLeftMotorAngle = 0;
            backLeftMotorAngle = 90;
            backRightMotorAngle = 45;
        } else if (fSharedInputValues.getBoolean("ipb_driver_dpad_right")){
            // Spin around right front wheel
            frontRightMotorSpeed = 0;
            frontLeftMotorSpeed = fRobotWidth / fDiameter;
            backLeftMotorSpeed = 1;
            backRightMotorSpeed = fRobotLength / fDiameter;
            frontRightMotorAngle = 0;
            frontLeftMotorAngle = 180;
            backLeftMotorAngle = 135;
            backRightMotorAngle = 90;

        } else if (fSharedInputValues.getBoolean("ipb_driver_dpad_down")){
            // Spin around right back wheel
            frontRightMotorSpeed = fRobotLength / fDiameter;
            frontLeftMotorSpeed = 1;
            backLeftMotorSpeed = fRobotWidth / fDiameter;
            backRightMotorSpeed = 0;
            frontRightMotorAngle = -90;
            frontLeftMotorAngle = -135;
            backLeftMotorAngle = 180;
            backRightMotorAngle = 0;
        } else if (fSharedInputValues.getBoolean("ipb_driver_dpad_left")){
            // Spin around left back wheel
            frontRightMotorSpeed = 1;
            frontLeftMotorSpeed = fRobotLength / fDiameter;
            backLeftMotorSpeed = 0;
            backRightMotorSpeed = fRobotWidth / fDiameter;
            frontRightMotorAngle = -45;
            frontLeftMotorAngle = -90;
            backLeftMotorAngle = 0;
            backRightMotorAngle = 0;
        }

        // Set the motors
        fSharedOutputValues.setNumeric("opn_drivetrain_front_right_speed", "percent", frontRightMotorSpeed);
        fSharedOutputValues.setNumeric("opn_drivetrain_front_left_speed", "percent", frontLeftMotorSpeed);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_left_speed", "percent", backLeftMotorSpeed);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_right_speed", "percent", backRightMotorSpeed);

        // set the angle
        fSharedOutputValues.setNumeric("opn_drivetrain_front_right_angle", "position", frontRightMotorAngle);
        fSharedOutputValues.setNumeric("opn_drivetrain_front_left_angle", "position", frontLeftMotorAngle);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_left_angle", "position", backLeftMotorAngle);
        fSharedOutputValues.setNumeric("opn_drivetrain_back_right_angle", "position", backRightMotorAngle);

        // Store angle
        mPreviousfrontRightMotorAngle = frontRightMotorAngle;
        mPreviousfrontLeftMotorAngle = frontLeftMotorAngle;
        mPreviousBackLefttMotorAngle = backLeftMotorAngle;
        mPreviousBackRightMotorAngle = backRightMotorAngle;
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