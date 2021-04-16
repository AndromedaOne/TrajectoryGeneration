package frc.robot.BetterTrajectoryGenerators;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

public class AveragedCurvatures extends BetterStatesGeneratorBase {

    public AveragedCurvatures(List<State> jsonPathPoints) {
        super(jsonPathPoints);
    }

    @Override
    public List<State> getBetterStates() {
        int count = 0;
        List<State> betterStates = new ArrayList<State>();
        for(State point : m_states) {
            double newCurvature = 0;
            
            if(count == 0 || count == m_states.size() - 1) {
              newCurvature = point.curvatureRadPerMeter;
            }else if(Math.abs(point.curvatureRadPerMeter) > 1.0e-6) {
              newCurvature = point.curvatureRadPerMeter;
            }else {
              State previousPoint = m_states.get(count - 1);
              State nextPoint = m_states.get(count + 1);
              double totalTime = nextPoint.timeSeconds - previousPoint.timeSeconds;
              double curvatureWeightedAverage = previousPoint.curvatureRadPerMeter * (point.timeSeconds - previousPoint.timeSeconds) / totalTime + nextPoint.curvatureRadPerMeter * (nextPoint.timeSeconds - point.timeSeconds) / totalTime;
              newCurvature = curvatureWeightedAverage;
            }
            State betterPoint = new State(
                point.timeSeconds,
                point.velocityMetersPerSecond,
                point.accelerationMetersPerSecondSq,
                point.poseMeters,
                newCurvature
            );
            
            betterStates.add(betterPoint);
            count++;
          }
        return betterStates;
    }
    
}
