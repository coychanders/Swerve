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
 * Example behavior to copy for other behaviors
 */

public class Behavior_Example implements Behavior {

    private static final Logger sLogger = LogManager.getLogger(Behavior_Example.class);
    private static final Set<String> sSubsystems = Set.of("nameofsubsystem");

    private final InputValues fSharedInputValues;
    private final OutputValues fSharedOutputValues;
    private final String fWhatThisButtonDoes;

    private double mConfigurationValue;

    public Behavior_Example(InputValues inputValues, OutputValues outputValues, Config config, RobotConfiguration robotConfiguration) {
        fSharedInputValues = inputValues;
        fSharedOutputValues = outputValues;
        fWhatThisButtonDoes = robotConfiguration.getString("global_subsystem", "what_this_button_does");

        mConfigurationValue = 0.0;
    }

    @Override
    public void initialize(String stateName, Config config) {
        sLogger.debug("Entering state {}", stateName);

        mConfigurationValue = config.getDouble("configuration_value");
    }

    @Override
    public void update() {
        boolean whatThisButtonDoes = fSharedInputValues.getBoolean(fWhatThisButtonDoes);
    }

    @Override
    public void dispose() {

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