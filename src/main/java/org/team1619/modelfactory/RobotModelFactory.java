package org.team1619.modelfactory;

import org.team1619.robot.AbstractRobotModelFactory;
import org.uacr.robot.AbstractModelFactory;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.ObjectsDirectory;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.injection.Inject;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

public class RobotModelFactory extends AbstractModelFactory {

    private static final Logger sLogger = LogManager.getLogger(RobotModelFactory.class);

    @Inject
    public RobotModelFactory(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration, ObjectsDirectory objectsDirectory) {
        super(inputValues, outputValues, robotConfiguration, objectsDirectory);
        registerModelFactory(new AbstractRobotModelFactory(inputValues, outputValues, robotConfiguration, objectsDirectory));
        registerModelFactory(new ModelFactory_Behaviors(inputValues, outputValues, robotConfiguration, objectsDirectory));
    }

}