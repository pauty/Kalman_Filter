
package kalmanfilter;

import org.ejml.simple.SimpleMatrix;

/*NOTE: in what follows, lowcase letters (i.e. x) represent vectors
and uppercase letters (i.e. A) represent matrices (for SimpleMatrix variables)
*/

public class KalmanFilter{

    // process description
    private SimpleMatrix A, B, Q;
    
    // measurement description
    private SimpleMatrix H, R;
    
    // Kalman Gain
    private SimpleMatrix K;

    // state estimate and covariance
    private SimpleMatrix x, P;
    

    public void configureProcessModel(SimpleMatrix A, SimpleMatrix B, SimpleMatrix Q) {
        this.A = new SimpleMatrix(A);
        if(B != null)
            this.B = new SimpleMatrix(B);
        this.Q = new SimpleMatrix(Q);
    }
    
    public void configureMeasurementModel(SimpleMatrix H, SimpleMatrix R) {
        this.H = new SimpleMatrix(H);
        this.R = new SimpleMatrix(R);
    }

    public void setStateAndCovariance(SimpleMatrix  x, SimpleMatrix P) {
        this.x = new SimpleMatrix(x);
        this.P = new SimpleMatrix(P);
    }
       
    // u is the input control
    public void predict(SimpleMatrix u) {
        
        //compute new predicted state
        if(B != null && u != null){
            // x = A x + B u
            x = A.mult(x).plus(B.mult(u));
        }
        else{
            //ignore input control
            x = A.mult(x); 
        }
        
        //compute new predicted state covariance
        // P = A P A' + Q
        P = A.mult(P).mult(A.transpose()).plus(Q);
    }
    
    public void predict(){
        this.predict(null);
    }
    
    // z is the measurement
    public void correct(SimpleMatrix z) {
        
        // COMPUTE KALMAN GAIN
        
        // S = H P H' + R
        // S is the Kalman Gain denominator
        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);

        // K = P H' S^(-1)
        K = P.mult(H.transpose().mult(S.invert()));
        //K.print();
      
        // UPDATE STATE
      
        // y = z - H x
        SimpleMatrix y = z.minus(H.mult(x));
        // x = x + K y
        x = x.plus(K.mult(y));
        
        // UPDATE STATE COVARIANCE MATRIX (ERROR IN THE STATE ESTIMATE)

        // P = (I-KH)P = P - KHP
        //P = P.minus(K.mult(H).mult(P));
        P = (SimpleMatrix.identity(P.numRows()).minus(K.mult(H))).mult(P);
        P.print();
    }

    public SimpleMatrix getStateEstimate() {
        return x;
    }

    public SimpleMatrix getStateCovariance() {
        return P;
    }
    
    public SimpleMatrix getKalmanGain() {
        return K;
    }
}
