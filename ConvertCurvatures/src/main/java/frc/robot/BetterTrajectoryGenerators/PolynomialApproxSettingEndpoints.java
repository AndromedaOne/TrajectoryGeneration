package frc.robot.BetterTrajectoryGenerators;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.analysis.differentiation.FiniteDifferencesDifferentiator;
import org.apache.commons.math3.analysis.differentiation.UnivariateDifferentiableFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;

import org.apache.commons.math3.fitting.WeightedObservedPoints;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import frc.robot.Logging.Logging;

public abstract class PolynomialApproxSettingEndpoints extends PolynomialCurveApproximation {

    public PolynomialApproxSettingEndpoints(List<State> states) {
        super(states);
    }

    @Override
    protected PolynomialFunction getPolynomialApproximation(ValueGetter valueGetter, int order, boolean debugMode) {

        if(debugMode) {
            System.out.println("order: " + order);
        }
        int orderOfPolynomialApproximation = order; 
        double[] coefficients = new double[orderOfPolynomialApproximation + 1];
        coefficients[0] = valueGetter.get(0);
        coefficients[1] = 0;

        int orderOfPenUltimateThetaFunction = orderOfPolynomialApproximation - 3;
        double[] penUltimateThetaCoefficients = new double[orderOfPenUltimateThetaFunction + 1];
        final double deltaY = valueGetter.get(m_states.size() - 1) - valueGetter.get(0);
        final double finalTime = m_states.get(m_states.size() - 1).timeSeconds;
        double denominator = (1.0 - (orderOfPolynomialApproximation - 1.0) / orderOfPolynomialApproximation) * Math.pow(finalTime, orderOfPolynomialApproximation - 1);
        
        penUltimateThetaCoefficients[0] = deltaY / denominator;
        for(int i = 1; i < penUltimateThetaCoefficients.length; i++) {
            penUltimateThetaCoefficients[i] = ((i+1.0)/(orderOfPolynomialApproximation) - 1.0) * Math.pow(finalTime, i+1.0) / denominator;
        }

        int orderOfFinalThetaFunction = orderOfPenUltimateThetaFunction;
        double[] finalThetaCoefficients = new double[orderOfFinalThetaFunction + 1];
        denominator = (1 - orderOfPolynomialApproximation / (orderOfPolynomialApproximation - 1.0)) * Math.pow(finalTime, orderOfPolynomialApproximation);
        finalThetaCoefficients[0] = deltaY / denominator;

        for(int i = 1; i < penUltimateThetaCoefficients.length; i++) {
            finalThetaCoefficients[i] = ((i+1.0)/(orderOfPolynomialApproximation - 1.0) - 1.0) * Math.pow(finalTime, i+1.0) / denominator;
        }

        double[] outputData = new double[m_states.size()];
        for(int i = 0; i < outputData.length; i++) {
            outputData[i] = valueGetter.get(i) - coefficients[0] 
                - penUltimateThetaCoefficients[0] * Math.pow(m_states.get(i).timeSeconds, orderOfPolynomialApproximation - 1) 
                - finalThetaCoefficients[0] * Math.pow(m_states.get(i).timeSeconds, orderOfPolynomialApproximation);
        }

        double[][] inputData = new double[m_states.size()][orderOfPolynomialApproximation - 3];

        for(int i = 0; i < inputData.length; i++) {
            for(int j = 0; j < inputData[i].length; j++) {
                inputData[i][j] = Math.pow(m_states.get(i).timeSeconds, j+2) 
                    + penUltimateThetaCoefficients[j+1] * Math.pow(m_states.get(i).timeSeconds, orderOfPolynomialApproximation - 1)
                    + finalThetaCoefficients[j+1] * Math.pow(m_states.get(i).timeSeconds, orderOfPolynomialApproximation);
            }
        }

        RealMatrix inputMatrix = MatrixUtils.createRealMatrix(inputData);
        RealMatrix outputVector = MatrixUtils.createColumnRealMatrix(outputData);
        RealMatrix inverseMTransposeM = new LUDecomposition(inputMatrix.transpose().multiply(inputMatrix)).getSolver().getInverse();
        RealMatrix middleThetas = inverseMTransposeM.multiply(inputMatrix.transpose().multiply(outputVector));
        double[] coeffForMiddleParameters = middleThetas.getColumn(0);
        int count = 2;
        for(double coeff : coeffForMiddleParameters) {
            coefficients[count] = coeff;
            count++;
        }

        double penUltimateCoefficient = penUltimateThetaCoefficients[0];
        for(int i = 0; i < coeffForMiddleParameters.length; i++) {
            penUltimateCoefficient += penUltimateThetaCoefficients[i+1] * coeffForMiddleParameters[i];
        }

        double finalCoefficient = finalThetaCoefficients[0];
        for(int i = 0; i < coeffForMiddleParameters.length; i++) {

            finalCoefficient += finalThetaCoefficients[i+1] * coeffForMiddleParameters[i];
        }

        coefficients[coefficients.length - 2] = penUltimateCoefficient;
        coefficients[coefficients.length - 1] = finalCoefficient;

        if(debugMode){
            prettyPrint(coefficients);
        }
        return new PolynomialFunction(coefficients);
    }

    private void prettyPrint(double[] arr) {
        for(double x : arr) {
            System.out.println(x);
        }
    }
    
}
