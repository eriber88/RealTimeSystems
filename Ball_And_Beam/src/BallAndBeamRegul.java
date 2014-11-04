
import SimEnvironment.AnalogSink;
import SimEnvironment.AnalogSource;

// BallAndBeamRegul class to be written by you
public class BallAndBeamRegul extends Thread {
	
	private PI piCont;
	private PID pidCont;
	
	private BallAndBeam bb;
	private ReferenceGenerator refgen;
	
	private AnalogSource analogInAngle;
	private AnalogSource analogInPosition;
	private AnalogSink analogOut;
	private AnalogSink analogRef;
	
	private double uMin = -10.0; //Restrictions on output voltage
	private double uMax = 10.0;
	private double uAng, uPos;
	
	// Constructor
	public BallAndBeamRegul(ReferenceGenerator refgen, BallAndBeam bb, int priority){
		this.bb=bb;
		this.refgen=refgen;
		
		piCont = new PI("PI-regulator");
		pidCont= new PID("PID-regulator");
		
		
		
		analogInPosition = bb.getSource(0);
		analogInAngle = bb.getSource(1);
		analogOut = bb.getSink(0);
		analogRef = bb.getSink(1);
		
		setPriority(priority);
		

		
		
	}
	
	private double limit(double u, double umin, double umax){
		if(u<umin){
			return umin;
		}else if(u>umax){
			return umax;
		}else{
			return u;
		}
	}
		
	
	
	
	public void run(){
		long t = System.currentTimeMillis();
		long h = pidCont.getHMillis();
		long duration;
		
		while(!Thread.interrupted()){
			
			double xRef = refgen.getRef();
			double xPos = analogInPosition.get();
			double xAng = analogInAngle.get();
			
			synchronized (pidCont){
				uPos = pidCont.calculateOutput(xPos,xRef);
				//uPos becomes reference to beam process. (angle reference)
				uPos = limit(uPos, uMin, uMax);
					
				
				synchronized (piCont){
					uAng = piCont.calculateOutput(xAng, uPos);
					uAng = limit(uAng, uMin, uMax);
					analogOut.set(uAng);
											
					piCont.updateState(uPos);
					
				}
				pidCont.updateState(uAng);
			}
			
			System.out.println("Angle output: " + uAng + "\n Position Out: " +uPos );
			
			analogRef.set(xRef);
	
			t = t + piCont.getHMillis();
			duration = t- System.currentTimeMillis();
			if(duration>0){
				try{
					Thread.sleep(duration);
				
				}catch(InterruptedException ex){
					ex.printStackTrace();
				}
			}
		}
		System.out.println("Thread Interrupted: BallAndBeamRegul");				
	}
	
}


/*Regul should run in one out of three different modes: BEAM, BALL, and OFF. 
 * In the mode OFF the control signal (u) = 0 should be sent out.
 *  The regul thread should still be executing. In each loop, the regul thread 
 *  should call putControlDataPoint() and putMeasurementDataPoint() in OpCom.
 *   If the mode is OFF the signals sent to OpCom, (y, yref and u) should all 
 *   have the value 0. 
 *   */
