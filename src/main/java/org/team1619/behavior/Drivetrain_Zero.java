package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;

/**
 * Zeros the drivetrain.
 */

public class Drivetrain_Zero implements Behavior {

    private static final Logger sLogger = LogManager.getLogger(Drivetrain_Zero.class);
    private static final Set<String> sSubsystems = Set.of("ss_drivetrain");

    private final InputValues fSharedInputValues;
    private final OutputValues fSharedOutputValues;
    private final Timer fTimeoutTimer;

    private int mTimeoutTime;
    private double mZeroingThreshold;
    private String mStateName;


    public Drivetrain_Zero(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        fSharedInputValues = inputValues;
        fSharedOutputValues = outputValues;
        fTimeoutTimer = new Timer();

        mTimeoutTime = 1000;
        mZeroingThreshold = 0.0;
        mStateName = "unknown";
    }

    @Override
    public void initialize(String stateName, Config config) {
        sLogger.debug("Entering state {}", stateName);

        mStateName = stateName;
        mTimeoutTime = config.getInt("timeout_time");
        mZeroingThreshold = config.getDouble("zeroing_threshold");

        fTimeoutTimer.reset();
        fTimeoutTimer.start(mTimeoutTime);

        fSharedOutputValues.setNumeric("opn_drivetrain_left", "percent", 0.0);
        fSharedOutputValues.setNumeric("opn_drivetrain_right", "percent", 0.0);
    }

    @Override
    public void update() {

        if (!fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed")) {

            fSharedOutputValues.setOutputFlag("opn_drivetrain_left", "zero");
            fSharedOutputValues.setOutputFlag("opn_drivetrain_right", "zero");

            if (Math.abs(fSharedInputValues.getNumeric("ipn_drivetrain_left_primary_position")) < mZeroingThreshold && Math.abs(fSharedInputValues.getNumeric("ipn_drivetrain_right_primary_position")) < mZeroingThreshold) {
                sLogger.debug("Drivetrain Zero -> Zeroed");
                fSharedOutputValues.setNumeric("opn_drivetrain_left", "percent", 0);
                fSharedOutputValues.setNumeric("opn_drivetrain_right", "percent", 0);
                fSharedInputValues.setBoolean("ipb_odometry_has_been_zeroed", false);
                fSharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", true);
            }
        }
    }

    @Override
    public void dispose() {
        sLogger.trace("Leaving state {}", mStateName);
        fSharedOutputValues.setNumeric("opn_drivetrain_left", "percent", 0);
        fSharedOutputValues.setNumeric("opn_drivetrain_right", "percent", 0);
    }

    @Override
    public boolean isDone() {
        if (fTimeoutTimer.isDone() && !fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed")) {
            fTimeoutTimer.reset();
            sLogger.error("Drivetrain Zero -> Timed Out");
        }
        return fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed") || fTimeoutTimer.isDone();
    }

    @Override
    public Set<String> getSubsystems() {
        return sSubsystems;
    }

}
