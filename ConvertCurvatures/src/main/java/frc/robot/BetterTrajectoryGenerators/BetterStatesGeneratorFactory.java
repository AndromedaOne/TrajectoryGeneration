package frc.robot.BetterTrajectoryGenerators;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryParameterizer;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

public class BetterStatesGeneratorFactory {
    private static BetterStatesGeneratorBase betterStatesGeneratorBase;
    
    public static BetterStatesGeneratorBase create(List<State> jsonPathPoints) {
        betterStatesGeneratorBase = new TranslatedPolynomialBetterStates(jsonPathPoints);
        return betterStatesGeneratorBase;
    }

    public static Trajectory createTrajectory(List<State> states) {
        return new Trajectory(states);
    }

    public static Trajectory createTrajectory(List<State> states, TrajectoryConfig config) {
        List<PoseWithCurvature> points = new ArrayList<PoseWithCurvature>();
        for(State state : states) {
            PoseWithCurvature poseWithCurvature = new PoseWithCurvature(state.poseMeters, state.curvatureRadPerMeter);
            points.add(poseWithCurvature);
        }

        return TrajectoryParameterizer.timeParameterizeTrajectory(points, 
        config.getConstraints(),
        config.getStartVelocity(),
        config.getEndVelocity(),
        config.getMaxVelocity(),
        config.getMaxAcceleration(),
        config.isReversed());    
    }

}
