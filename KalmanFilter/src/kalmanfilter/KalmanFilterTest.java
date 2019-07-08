
package kalmanfilter;

import java.util.Locale;
import java.util.Random;
import org.ejml.simple.SimpleMatrix;

/*NOTE: in what follows, lowcase letters (i.e. x) represent vectors
and uppercase letters (i.e. A) represent matrices (for SimpleMatrix variables)
*/

public class KalmanFilterTest {
    
        public static void main(String[] args) {
        
        // discrete time interval [seconds]
        double deltaT = 1d;
             
        // PROCESS NOISE 
        
        // in kalman filters the mean for process noise is assumed to be 0
        final double accelNoiseXMean = 0d;
        final double accelNoiseYMean = 0d;
        
        // standard deviations of process noise (noise in acceleration) [meter/sec^2] 
        // !! - WARNING: it is assumed that noise in the process is interely related to acceleration
        // (due to friction ecc). See definition of Q and pNoise
        double accelNoiseXDeviation = 100d;//3d;
        double accelNoiseYDeviation = 100d;//3d;
        
        //MEASUREMENT NOISE
        
        // in kalman filters the mean for measurement noise is assumed to be 0
        final double measurementNoiseXMean = 0d;
        final double measurementNoiseYMean = 0d;
        
        // standard deviations of measurement noise [meters] 
        double measurementNoiseXDeviation = 0d;//6d;
        double measurementNoiseYDeviation = 0d;//6d;
          
        // state transition matrix
        // A = [ 1 0 dt 0]
        //     [ 0 1 0 dt]
        //     [ 0 0 1 0 ]
        //     [ 0 0 0 1 ]
        // 4X4
        SimpleMatrix A = new SimpleMatrix(new double[][] { 
        { 1, 0, deltaT, 0 },
        { 0, 1, 0, deltaT },
        {0, 0, 1, 0},
        {0, 0, 0, 1} });
        
        // control input matrix
        // B = [ dt^2/2 0 ]
        //     [ 0 dt^2/2 ]
        //     [ dt   0   ]
        //     [ 0    dt  ]
        // 4x2
        SimpleMatrix B = new SimpleMatrix(new double[][] { 
        { (Math.pow(deltaT, 2)/2), 0 },
        { 0, (Math.pow(deltaT, 2)/2) },
        { deltaT, 0 },
        { 0, deltaT } });
        
        // measurement matrix 
        // H = [ 1 0 0 0]
        //     [ 0 1 0 0]
        // 2x4   ---> the sensor measures (x,y) position
        SimpleMatrix H = new SimpleMatrix(new double[][] { 
        { 1, 0, 0, 0},
        { 0, 1, 0, 0} });
        
        // process noise covariance matrix
        // 4x4
        double dt4_4 = Math.pow(deltaT, 4) / 4;
        double dt2 = Math.pow(deltaT, 2);             
        double accelNoiseXVariance = Math.pow(accelNoiseXDeviation, 2);
        double accelNoiseYVariance = Math.pow(accelNoiseYDeviation, 2);
        SimpleMatrix Q = new SimpleMatrix(new double[][] {
        { dt4_4*accelNoiseXVariance, 0, 0, 0 },
        { 0, dt4_4*accelNoiseYVariance, 0, 0 },
        { 0, 0, dt2*accelNoiseXVariance, 0 },
        { 0, 0, 0, dt2*accelNoiseYVariance } });   
        
        //--------------------------------------------
        
        // measurement covariance matrix
        // R = [ measurementNoiseXDeviation^2 0 ]
        //     [ 0 measurementNoiseYDeviation^2 ]
        // 2x2
        double measurementNoiseXVariance = Math.pow(measurementNoiseXDeviation, 2);
        double measurementNoiseYVariance = Math.pow(measurementNoiseYDeviation, 2);
        SimpleMatrix R = new SimpleMatrix(new double[][] { 
        { measurementNoiseXVariance, 0 },
        { 0, measurementNoiseYVariance } });
        
        // control input vector
        // increase x velocity by 0.1 m/s and y velocity by -0.5 m/s  per cycle
        // (constant acceleration) 
        // 2x1
        SimpleMatrix u = new SimpleMatrix(new double[][] { { 0.1d }, { -0.5d } });
        
        //----------------------------------------------
        
        // INIT KALMAN FILTER
        
        // x = [ 0 ]
        //     [ 0 ]
        //     [ 0 ]
        //     [ 0 ]
        // 4x1
        // initially, estimed state can be an arbitrary vector
        SimpleMatrix x = new SimpleMatrix(new double[][] { { 0 }, { 0 }, { 0 }, { 0 } });
               
        // P0 = I
        // 4x4
        // initially, state covariance P0 can be an arbitrary matrix, and slowly P
        // will converge to the actual state covariance matrix
        SimpleMatrix P0 = SimpleMatrix.identity(4);
        // P0 = 0
        // for when estimated and real initial state are the same
        //SimpleMatrix P0 = new SimpleMatrix(4, 4);
        
        KalmanFilter kf = new KalmanFilter();
        kf.configureProcessModel(A, B, Q);
        kf.configureMeasurementModel(H, R);
        //set initial state
        kf.setStateAndCovariance(x, P0);
        
        //----------------------------------------------
        
        // REAL PROCESS NOISE AND MEASUREMENT NOISE

        // base matrix to generate acceleration noise with distribution N(0, accelNoiseDeviation^2)
        // (according with the definition of Q)
        // 4x1
        SimpleMatrix basePNoise = new SimpleMatrix(new double[][] {
        { Math.pow(deltaT, 2) / 2 }, { Math.pow(deltaT, 2) / 2 }, { deltaT }, { deltaT } });
        
        // the vector that stores gaussian random values 
        // 4x1
        SimpleMatrix processGaussian = new SimpleMatrix(4, 1);
        
        SimpleMatrix pNoise;
        
        // measurement noise vector (sensor/observation/evidence noise)
        // it will be filled with values from distribution N(0, measurementNoiseDeviation^2) 
        // (according with the definition of R)
        // 2x1
        SimpleMatrix  mNoise = new SimpleMatrix(2, 1);
        
        // the measurement vector
        SimpleMatrix z; 
        
        Random rand = new Random();
 
        // the initial real state (posX, posY, velX, velY)
        // wrong start, but not too much
        SimpleMatrix xReal = new SimpleMatrix(new double[][] { { 5 }, { 20 }, { 3 }, { -3 } });
        // perfect start
        //SimpleMatrix xReal = new SimpleMatrix(new double[][] { { 0 }, { 0 }, { 0 }, { 0 } });
        // wrong start
        //SimpleMatrix xReal = new SimpleMatrix(new double[][] { { 220 }, { -137 }, { 17 }, { 28 } });
        
        int consecutiveCount = 0; 
        int maxConsecutive = 0;
        int totalCount = 0;
        int targetCount = 10;
        double positionTolerance = 9d;//80d;
        double speedTolerance = 9d;//200d;
        boolean breakOnTarget = false;
        
        //number of iterations
        int numIterations = 20;
        
        boolean registerData = (numIterations == 20);
             
        float real_x[] = null;
        float est_x[] = null;
        float meas_x[] = null;
        if(registerData){
            real_x = new float[numIterations];
            est_x = new float[numIterations];
            meas_x = new float[numIterations];
        }
                 
        // iterate for numIterations steps
        for (int i = 0; i < numIterations; i++) {
            
            // SIMULATE THE REAL PROCESS
            
            double accelNoiseX = accelNoiseXMean + accelNoiseXDeviation * rand.nextGaussian();
            double accelNoiseY = accelNoiseYMean + accelNoiseYDeviation * rand.nextGaussian();
            processGaussian.set(0, 0, accelNoiseX);
            processGaussian.set(1, 0, accelNoiseY);
            processGaussian.set(2, 0, accelNoiseX);
            processGaussian.set(3, 0, accelNoiseY);
            pNoise = basePNoise.elementMult(processGaussian);

            // x = A xReal + B u + pNoise
            xReal = A.mult(xReal).plus(B.mult(u)).plus(pNoise);
            
            // SIMULATE THE MEASUREMENT
            
            mNoise.set(0,0, measurementNoiseXMean + measurementNoiseXDeviation * rand.nextGaussian());
            mNoise.set(1,0, measurementNoiseYMean + measurementNoiseYDeviation * rand.nextGaussian());

            // z = H x + mNoise
            // 2x1 = 2x4 * 4x1 + 2x1
            z = H.mult(xReal).plus(mNoise);
            
            // KALMAN FILTER UPDATE

            // compute new predicted state
            kf.predict(u);
            
            // correct the new predicted state using the measurement
            kf.correct(z);
            
            if(registerData){
                // collect data to plot in octave
                real_x[i] = (float)xReal.get(0,0); 
                est_x[i] = (float)kf.getStateEstimate().get(0,0);
                meas_x[i] = (float)z.get(0,0);
            }
            
            System.out.println(">> ITERATION: " + i);
            System.out.println("measured position x" + z.get(0,0));
            System.out.println("estimated position x: " + kf.getStateEstimate().get(0,0));
            System.out.println("estimated position y: " + kf.getStateEstimate().get(1,0));
            System.out.println("estimated velocity x: " + kf.getStateEstimate().get(2,0));
            System.out.println("estimated velocity y: " + kf.getStateEstimate().get(3,0));
            System.out.println("real position x: " + xReal.get(0,0));
            System.out.println("real position y: " + xReal.get(1,0));
            System.out.println("real velocity x: " + xReal.get(2,0));
            System.out.println("real velocity y: " + xReal.get(3,0));
            System.out.println("---------------------------------");
            
            /*
            double error_sum = Math.abs(kf.getStateEstimation().get(0,0)-xReal.get(0,0)) +
               Math.abs(kf.getStateEstimation().get(1,0)-xReal.get(1,0)) +
               Math.abs(kf.getStateEstimation().get(2,0)-xReal.get(2,0)) +
               Math.abs(kf.getStateEstimation().get(3,0)-xReal.get(3,0));
            
            System.out.println("error sum:" + error_sum);
            */
            
            if(i >= 50 &&
               Math.abs(kf.getStateEstimate().get(0,0)-xReal.get(0,0)) < positionTolerance &&
               Math.abs(kf.getStateEstimate().get(1,0)-xReal.get(1,0)) < positionTolerance &&
               Math.abs(kf.getStateEstimate().get(2,0)-xReal.get(2,0)) < speedTolerance &&
               Math.abs(kf.getStateEstimate().get(3,0)-xReal.get(3,0)) < speedTolerance){
                consecutiveCount++;
                totalCount++;
            }
            else{
                if(consecutiveCount > maxConsecutive)
                    maxConsecutive = consecutiveCount;
                consecutiveCount = 0;
            }
            
            if(breakOnTarget && consecutiveCount >= targetCount)
                break;
        }
        
        System.out.println("iterations satisfing tolerance:" + totalCount);
        System.out.println("max consecutive:" + maxConsecutive);
        
        
        if(registerData){
            Locale.setDefault(Locale.US); // for dot floating point notation

            System.out.print("real_x = [");
            for(int i = 0; i < numIterations-1; i++){
                System.out.print(String.format("%.2f", real_x[i]) + ",");
            }
            System.out.println(String.format("%.2f", real_x[numIterations-1]) + "];");

            System.out.print("est_x = [");
            for(int i = 0; i < numIterations-1; i++){
                System.out.print(String.format("%.2f", est_x[i]) + ",");
            }
            System.out.println(String.format("%.2f", est_x[numIterations-1]) + "];");

            System.out.print("meas_x = [");
            for(int i = 0; i < numIterations-1; i++){
                System.out.print(String.format("%.2f", meas_x[i]) + ",");
            }
            System.out.println(String.format("%.2f", meas_x[numIterations-1]) + "];");
        }

    }
       
}
