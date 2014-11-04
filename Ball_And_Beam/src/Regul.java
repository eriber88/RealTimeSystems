
import se.lth.control.DoublePoint;
import se.lth.control.realtime.AnalogIn;
import se.lth.control.realtime.AnalogOut;
import se.lth.control.realtime.IOChannelException;
import se.lth.control.realtime.Semaphore;

public class Regul extends Thread {
	public static final int OFF = 0;
	public static final int BEAM = 1;
	public static final int BALL = 2;
	
	private PI inner = new PI("PI");
	private PID outer = new PID("PID");
	
	private AnalogIn analogInAngle; 
	private AnalogIn analogInPosition; 
	private AnalogOut analogOut;
	
	private ReferenceGenerator referenceGenerator;
	private OpCom opcom;
	
	private int priority;
	private boolean WeShouldRun = true;
	private long starttime;
	private Semaphore mutex; // used for synchronization at shut-down
	
	
	private double uMin = -10;
	private double uMax = 10;

	
	
	
	private ModeMonitor modeMon;
	
	// Inner monitor class
	class ModeMonitor {
		private int mode;
		
		// Synchronized access methods
		public synchronized void setMode(int newMode) {
			mode = newMode;
			inner.reset();
			outer.reset();
		}
		
		public synchronized int getMode() {
			return mode;
		}
	}
	
	public Regul(int pri) {
		priority = pri;
		mutex = new Semaphore(1);
		try {
			analogInAngle = new AnalogIn(0);
			analogInPosition = new AnalogIn(1);
			analogOut = new AnalogOut(0);
		} catch (IOChannelException e) { 
			System.out.print("Error: IOChannelException: ");
			System.out.println(e.getMessage());
		}
		modeMon = new ModeMonitor();
	}
	
	public void setOpCom(OpCom opcom) {
		this.opcom=opcom;
	}
	
	public void setRefGen(ReferenceGenerator referenceGenerator){
		this.referenceGenerator = referenceGenerator;
	
	}
	
	// Called in every sample in order to send plot data to OpCom
	private void sendDataToOpCom(double yref, double y, double u) {
		double x = (double)(System.currentTimeMillis() - starttime) / 1000.0;
		DoublePoint dp = new DoublePoint(x,u);
		PlotData pd = new PlotData(x,yref,y);
		opcom.putControlDataPoint(dp);
		opcom.putMeasurementDataPoint(pd);
	}
	
	public synchronized void setInnerParameters(PIParameters p) {
		inner.setParameters(p);

	}
	
	public synchronized PIParameters getInnerParameters() {
		return inner.getParameters();

	}
	
	public synchronized void setOuterParameters(PIDParameters p) {
		outer.setParameters(p);

	}
	
	public synchronized PIDParameters getOuterParameters(){
		return outer.getParameters();
	
	}
	
	public void setOFFMode(){
		modeMon.setMode(OFF); 

	}
	
	public void setBEAMMode(){
		modeMon.setMode(BEAM); 

	}
	
	public void setBALLMode(){
		modeMon.setMode(BALL); 
	
	}
	
	public int getMode(){
		return modeMon.getMode();
	
	}
	
	// Called from OpCom when shutting down
	public synchronized void shutDown() {
		WeShouldRun = false;
		mutex.take();
		try {
			analogOut.set(0.0);
		} catch (IOChannelException x) {
		}
	}
	
	private double limit(double v, double min, double max) {
		if (v < min) {
			v = min;
		} else if (v > max) {
			v = max;
		}
		return v;
	}
	
	public void run() {
		long duration;
		long t = System.currentTimeMillis();
		starttime = t;
		double xPos =0.0, xAng=0.0, uPos=0.0, uAng=0.0;
		
		setPriority(priority);
		mutex.take();
		
		while (WeShouldRun) {
			
			double xRef = referenceGenerator.getRef();
			try{
				xPos = analogInPosition.get();
				xAng = analogInAngle.get();
			}catch(Exception e){
				e.printStackTrace();
			}

			
			
			
			switch (modeMon.getMode()) {
			case OFF: {

				try{
					analogOut.set(0.0);					
				}catch(Exception e){
					e.printStackTrace();
				}
				inner.reset();
				outer.reset(); 
				sendDataToOpCom(0,0,0);
				
				break;
			}
			case BEAM: {
				
				synchronized (inner){
					uAng = inner.calculateOutput(xAng, xRef);
					uAng = limit(uAng, uMin, uMax);
					
					try{
						analogOut.set(uAng);
					}catch(Exception e){
						e.printStackTrace();
					}
											
					inner.updateState(uAng);
					
				}
				sendDataToOpCom(xRef, xAng, uAng );
				
				// Code for the BEAM mode
				// Written by you.
				// Should include a call to sendDataToOpCom
				break;
			}
			case BALL: {
				synchronized (outer){
					uPos = outer.calculateOutput(xPos,xRef);
					//uPos becomes reference to beam process. (angle reference)
					uPos = limit(uPos, uMin, uMax);
						
					
					synchronized (inner){
						uAng = inner.calculateOutput(xAng, uPos);
						uAng = limit(uAng, uMin, uMax);
						
						try{
							analogOut.set(uAng);
						}catch(Exception e){
							e.printStackTrace();
						}
												
						inner.updateState(uAng);
						
					}
					outer.updateState(uPos);
				}
				sendDataToOpCom(xRef, xPos, uPos );
				// Code for the BALL mode
				// Written by you.
				// Should include a call to sendDataToOpCom 
				break;
			}
			default: {
				System.out.println("Error: Illegal mode.");
				break;
			}
			}
			
			// sleep
			t = t + inner.getHMillis();
			duration = t - System.currentTimeMillis();
			if (duration > 0) {
				try {
					sleep(duration);
				} catch (InterruptedException x) {
				}
			}
		}
		mutex.give();
	}
}
