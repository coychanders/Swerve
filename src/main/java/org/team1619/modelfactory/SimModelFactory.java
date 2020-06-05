package org.team1619.modelfactory;

import org.team1619.robot.AbstractSimModelFactory;
import org.uacr.shared.abstractions.*;
import org.uacr.utilities.injection.Inject;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

public class SimModelFactory extends AbstractSimModelFactory {

    private static final Logger sLogger = LogManager.getLogger(SimModelFactory.class);

    @Inject
    public SimModelFactory(EventBus eventBus, InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration, ObjectsDirectory objectsDirectory) {
        super(eventBus, inputValues, outputValues, robotConfiguration, objectsDirectory);
        registerModelFactory(new AbstractSimModelFactory(eventBus, inputValues, outputValues, robotConfiguration, objectsDirectory));
        registerModelFactory(new ModelFactory_Behaviors(inputValues, outputValues, robotConfiguration, objectsDirectory));
    }
}