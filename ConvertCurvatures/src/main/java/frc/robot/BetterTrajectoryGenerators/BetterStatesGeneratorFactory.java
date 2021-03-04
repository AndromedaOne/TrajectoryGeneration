package frc.robot.BetterTrajectoryGenerators;

import java.util.List;

import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

public class BetterStatesGeneratorFactory {
    
    public static BetterStatesGeneratorBase create(List<State> jsonPathPoints) {
        return new TranslatedPolynomialBetterStates(jsonPathPoints);
    }
}
