package frc.robot.BetterTrajectoryGenerators;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.Logging.Logging;

public class TranslatedPolynomialBetterStates extends PolynomialCurveApproximation {

    public static final int pointsOffEndsToRemove = 1;

    public TranslatedPolynomialBetterStates(List<State> states) {
        super(states);
    }

    protected List<State> generateBetterStates() {
        List<State> betterStates = new ArrayList<State>();
        PolynomialFunction xDerivative = m_xApprox.polynomialDerivative();
        PolynomialFunction yDerivative = m_yApprox.polynomialDerivative();
        PolynomialFunction xSecondDerivative = xDerivative.polynomialDerivative();
        PolynomialFunction ySecondDerivative = yDerivative.polynomialDerivative();
        

        int count = 0;
        double previousCurvature = 0;
        for (State state : m_states) {
            double newCurvature = 0;
            Pose2d newPoint = state.poseMeters;
            if(count < pointsOffEndsToRemove || count > m_states.size() - 1 - pointsOffEndsToRemove ) {
                newCurvature = state.curvatureRadPerMeter;
            }else{
                double time = state.timeSeconds;
                double dx = xDerivative.value(time);
                double dy = yDerivative.value(time);
                double dxSquared = xSecondDerivative.value(time);
                double dySquared = ySecondDerivative.value(time);
                newCurvature = CalculatedCurvatures.getCurvature(dx, dy, dxSquared, dySquared);
                if(Math.abs(state.curvatureRadPerMeter) > 1.0e-6) {
                    newCurvature *= Math.signum(state.curvatureRadPerMeter);
                }else {
                    newCurvature *= Math.signum(previousCurvature);
                }

                double newX = m_xApprox.value(state.timeSeconds);
                double newY = m_yApprox.value(state.timeSeconds);
                double newRotation = Math.atan2(dy,  dx);
                if(state.velocityMetersPerSecond < 0) {
                    newRotation += Math.PI;
                }
                newPoint = new Pose2d(newX, newY, new Rotation2d(newRotation));
                
                if(Math.abs(state.curvatureRadPerMeter - previousCurvature) < Math.abs(newCurvature - previousCurvature)) {
                    newCurvature = state.curvatureRadPerMeter;
                    newPoint = state.poseMeters;
                }
            }   
            State betterState = new State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                newPoint,
                newCurvature
            );
            betterStates.add(betterState);
            count++;
            previousCurvature = newCurvature;
        }

        return betterStates;
    }
    
}
