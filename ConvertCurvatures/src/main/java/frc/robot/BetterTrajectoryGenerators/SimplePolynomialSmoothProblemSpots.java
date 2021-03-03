package frc.robot.BetterTrajectoryGenerators;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;

import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

public class SimplePolynomialSmoothProblemSpots extends PolynomialCurveApproximation {
    public SimplePolynomialSmoothProblemSpots(List<State> states) {
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
        for(State state : m_states) {
            double newCurvature = 0;
            if(count == 0 || count == m_states.size() - 1 || Math.abs(state.curvatureRadPerMeter) > 1.0e-6) {
                newCurvature = state.curvatureRadPerMeter;
            }else{
                double time = state.timeSeconds;
                double dx = xDerivative.value(time);
                double dy = yDerivative.value(time);
                double dxSquared = xSecondDerivative.value(time);
                double dySquared = ySecondDerivative.value(time);
                newCurvature = CalculatedCurvatures.getCurvature(dx, dy, dxSquared, dySquared);
                newCurvature *= Math.signum(previousCurvature);
            }
             
            State betterState = new State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                state.poseMeters,
                newCurvature
            );
            betterStates.add(betterState);
            count++;
            previousCurvature = newCurvature;
        }

        return betterStates;
    }
}
