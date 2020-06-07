package org.team1619.state;

import org.uacr.robot.AbstractRobotStatus;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

/**
 * Sets flags and does global math and logic for competition bot
 */

public class RobotStatus extends AbstractRobotStatus {

    private static final Logger sLogger = LogManager.getLogger(RobotStatus.class);

    public RobotStatus(InputValues inputValues, RobotConfiguration robotConfiguration) {
        super(inputValues, robotConfiguration);

    }

    @Override
    public void initialize() {
        // Zero
        if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed")) {
            fSharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", false);
            fSharedInputValues.setInputFlag("ipv_navx", "zero");
        }

        fSharedInputValues.setBoolean("ipb_swerve_field_centric", true);
        fSharedInputValues.setBoolean("ipb_swerve_spin_or_point", true);

    }

    @Override
    public void update() {

        if (!fSharedInputValues.getBoolean("ipb_robot_has_been_zeroed") &&
                fSharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed")) {

            fSharedInputValues.setBoolean("ipb_robot_has_been_zeroed", true);
        }
    }

    @Override
    public void dispose() {

    }
}
