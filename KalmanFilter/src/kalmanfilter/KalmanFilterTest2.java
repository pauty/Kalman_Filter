
package kalmanfilter;

import java.util.Locale;
import java.util.Random;
import org.ejml.simple.SimpleMatrix;

/*NOTE: in what follows, lowcase letters (i.e. x) represent vectors
and uppercase letters (i.e. A) represent matrices (for SimpleMatrix variables)
*/

// NON-LINEAR PROCESS TEST
public class KalmanFilterTest2 {
    
        public static void main(String[] args) {
        
        // discrete time interval [seconds]
        double deltaX = .4d;
             
        // PROCESS NOISE 
        
        // in kalman filters the mean for process noise is assumed to be 0
        final double processNoiseMean = 0d;
        
        // standard deviations of process noise (noise in acceleration) [meter/sec^2] 
        // !! - WARNING: it is assumed that noise in the process is interely related to acceleration
        // (due to friction ecc). See definition of Q and pNoise
        double processNoiseDeviation = 1d;//3d;
        
        //MEASUREMENT NOISE
        
        // in kalman filters the mean for measurement noise is assumed to be 0
        final double measurementNoiseMean = 0d;
        
        // standard deviations of measurement noise [meters] 
        double measurementNoiseDeviation = 0.01d; //6d
          
        // state transition matrix
        // A = [ 0 2 ]
        //     [ 0 1 ]   
        // 2x2
        SimpleMatrix A = new SimpleMatrix(new double[][] { 
        { 0, 2 },
        { 0, 1 } });
        
        // control input matrix
        // B = I 
        // 4x2
        SimpleMatrix B =  SimpleMatrix.identity(2);
        
        // measurement matrix 
        // H = [ 1 0 ]
        // 1x2   ---> the sensor measures (x,y) position
        SimpleMatrix H = new SimpleMatrix(new double[][] { { 1, 0 } });
        
        // process noise covariance matrix
        // 2x2            
        double processNoiseVariance = Math.pow(processNoiseDeviation, 2);
        SimpleMatrix Q = new SimpleMatrix(new double[][] {
        { processNoiseVariance*processNoiseVariance, processNoiseVariance*processNoiseDeviation },
        { processNoiseVariance*processNoiseDeviation, processNoiseVariance } });   
        
        //--------------------------------------------
        
        // measurement covariance matrix
        // R = [ measurementNoiseXDeviation^2 0 ]
        //     [ 0 measurementNoiseYDeviation^2 ]
        // 1x1
        double measurementNoiseVariance = Math.pow(measurementNoiseDeviation, 2);
        SimpleMatrix R = new SimpleMatrix(new double[][] { { measurementNoiseVariance } });
        
        // control input vector
        // increase x velocity by 0.1 m/s and y velocity by -0.5 m/s  per cycle
        // (constant acceleration) 
        // 2x1
        SimpleMatrix u = new SimpleMatrix(new double[][] { { 2*deltaX -1 }, { deltaX } });
        
        //----------------------------------------------
        
        // INIT KALMAN FILTER
        
        // x = [ 0 ]
        //     [ 0 ]
        // 2x1
        // initially, estimed state can be an arbitrary vector
        SimpleMatrix x = new SimpleMatrix(new double[][] { { 0 }, { 0 } });
               
        // P0 = I
        // 2x2
        // initially, state covariance P0 can be an arbitrary matrix, and slowly P
        // will converge to the actual state covariance matrix
        // SimpleMatrix P0 = SimpleMatrix.identity(4);
        // P0 = 0
        // for when estimated and real initial state are the same
        SimpleMatrix P0 = new SimpleMatrix(2, 2);
        
        KalmanFilter kf = new KalmanFilter();
        kf.configureProcessModel(A, B, Q);
        kf.configureMeasurementModel(H, R);
        //set initial state
        kf.setStateAndCovariance(x, P0);
        
        //----------------------------------------------
        
        // REAL PROCESS NOISE AND MEASUREMENT NOISE
        
        SimpleMatrix pNoise = new SimpleMatrix(2, 1);
        
        // measurement noise vector (sensor/observation/evidence noise)
        // it will be filled with values from distribution N(0, measurementNoiseDeviation^2) 
        // (according with the definition of R)
        // 1x1
        SimpleMatrix  mNoise = new SimpleMatrix(1, 1);
        
        // the measurement vector
        SimpleMatrix z; 
        
        Random rand = new Random();
 
        // the initial real state (perfect start)
        SimpleMatrix xReal = new SimpleMatrix(new double[][] { { 0 }, { 0 } });
    
        int consecutiveCount = 0; 
        int maxConsecutive = 0;
        int totalCount = 0;
        int targetCount = 10;
        double positionTolerance = 9d;//80d;
        double speedTolerance = 9d;//200d;
        boolean breakOnTarget = false;
        
        //number of iterations
        int numIterations = 100;
        
        boolean registerData = (numIterations == 100);
             
        float real_x[] = null;
        float est_x[] = null;    
        float real_x_squared[] = null;
        float est_x_squared[] = null;
        float meas_x[] = null;
        if(registerData){
            real_x_squared = new float[numIterations];
            est_x_squared = new float[numIterations];
            real_x = new float[numIterations];
            est_x = new float[numIterations];
            meas_x = new float[numIterations];
        }
                 
        // iterate for numIterations steps
        for (int i = 0; i < numIterations; i++) {
            
            // SIMULATE THE REAL PROCESS
            
            double xBaseNew = xReal.get(1,0) + deltaX + processNoiseMean + processNoiseDeviation * rand.nextGaussian();
            xReal.set(1, 0, xBaseNew);
            xReal.set(0, 0, Math.pow(xBaseNew, 2));
            
            // SIMULATE THE MEASUREMENT
            
            mNoise.set(0, 0, measurementNoiseMean + measurementNoiseDeviation * rand.nextGaussian());
            
            // z = H xReal + mNoise
            // 1x1
            z = H.mult(xReal).plus(mNoise);
            
            // KALMAN FILTER UPDATE

            // compute new predicted state
            kf.predict(u);
            
            // correct the new predicted state using the measurement
            kf.correct(z);
            
            if(registerData){
                // collect data to plot in octave
                real_x_squared[i] = (float)xReal.get(0,0); 
                est_x_squared[i] = (float)kf.getStateEstimate().get(0,0);
                real_x[i] = (float)xReal.get(1,0); 
                est_x[i] = (float)kf.getStateEstimate().get(1,0);
                meas_x[i] = (float)z.get(0,0);
            }
            
            System.out.println(">> ITERATION: " + i);
            System.out.println("measured x" + z.get(0,0));
            System.out.println("estimated squared x: " + kf.getStateEstimate().get(0,0));
            System.out.println("estimated base x: " + kf.getStateEstimate().get(1,0));
            System.out.println("real squared x: " + xReal.get(0,0));
            System.out.println("real base x: " + xReal.get(1,0));
            System.out.println("---------------------------------");
            
            /*
            double error_sum = Math.abs(kf.getStateEstimation().get(0,0)-xReal.get(0,0)) +
               Math.abs(kf.getStateEstimation().get(1,0)-xReal.get(1,0)) +
               Math.abs(kf.getStateEstimation().get(2,0)-xReal.get(2,0)) +
               Math.abs(kf.getStateEstimation().get(3,0)-xReal.get(3,0));
            
            System.out.println("error sum:" + error_sum);
            */
            
        }
        
        System.out.println("iterations satisfing tolerance:" + totalCount);
        System.out.println("max consecutive:" + maxConsecutive);
        
        
        if(registerData){
            Locale.setDefault(Locale.US); // for dot floating point notation
            
            System.out.print("real_x_squared = [");
            for(int i = 0; i < numIterations-1; i++){
                System.out.print(String.format("%.2f", real_x_squared[i]) + ",");
            }
            System.out.println(String.format("%.2f", real_x_squared[numIterations-1]) + "];");

            System.out.print("est_x_squared = [");
            for(int i = 0; i < numIterations-1; i++){
                System.out.print(String.format("%.2f", est_x_squared[i]) + ",");
            }
            System.out.println(String.format("%.2f", est_x_squared[numIterations-1]) + "];");
            
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
