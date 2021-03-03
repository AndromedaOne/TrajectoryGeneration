package frc.robot.BetterTrajectoryGenerators;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;

import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

public class SimplePolynomialBetterStates extends PolynomialCurveApproximation {
    
    public SimplePolynomialBetterStates(List<State> states) {
        super(states);
    }

    protected List<State> generateBetterStates() {
        List<State> betterStates = new ArrayList<State>();
        PolynomialFunction xDerivative = m_xApprox.polynomialDerivative();
        PolynomialFunction yDerivative = m_yApprox.polynomialDerivative();
        PolynomialFunction xSecondDerivative = xDerivative.polynomialDerivative();
        PolynomialFunction ySecondDerivative = yDerivative.polynomialDerivative();

        int count = 0;
        for(State state : m_states) {
            double newCurvature = 0;
            if(count == 0 || count == m_states.size() - 1) {
                newCurvature = state.curvatureRadPerMeter;
            }else{
                double time = state.timeSeconds;
                double dx = xDerivative.value(time);
                double dy = yDerivative.value(time);
                double dxSquared = xSecondDerivative.value(time);
                double dySquared = ySecondDerivative.value(time);
                if(time > 2.27 && time < 2.26) {
                    System.out.println("time: " + time);
                    System.out.println("dx: " + dx);
                    System.out.println("dy: " + dy);
                    System.out.println("dxSquared: " + dxSquared);
                    System.out.println("dySquared: " + dySquared);
                }
                newCurvature = CalculatedCurvatures.getCurvature(dx, dy, dxSquared, dySquared);
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
        }

        return betterStates;
    }
}
