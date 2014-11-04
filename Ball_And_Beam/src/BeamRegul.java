
import SimEnvironment.*;

// BeamRegul class to be written by you
public class BeamRegul extends Thread {
	
	private PI PIcontroller;
	private ReferenceGenerator referenceGenerator;
	
	
	// IO interface declarations
	private AnalogSource analogIn;
	private AnalogSink analogOut;
	private AnalogSink analogRef;
	
	private double uMin = -10.0;
	private double uMax = 10.0;
	
	
	public BeamRegul(ReferenceGenerator refgen, Beam beam, int priority) {
		// ...
		this.referenceGenerator=refgen;
		PIcontroller = new PI("Beam-PI");
		

		// Code to initialize the IO
		analogIn = beam.getSource(0);
		analogOut = beam.getSink(0);
		analogRef = beam.getSink(1);
		//...		
		
		setPriority(priority);
		
	}
	
	private void tsleep(long t){
		
			t = t + PIcontroller.getHMillis();
			long duration = t- System.currentTimeMillis();
			if(duration>0){
				try{
					Thread.sleep(duration);
			
				}catch(InterruptedException ex){
					ex.printStackTrace();
				}
			}
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
	
	public void run() {
		long t = System.currentTimeMillis();
		while(true){
			double y = analogIn.get();
			double ref = referenceGenerator.getRef();
			
			//Many things to do in Monitor controller (synchronize around class instance)
			synchronized (PIcontroller){
				double u = PIcontroller.calculateOutput(y,ref);
				u = limit(u, uMin, uMax);
				analogOut.set(u);
				PIcontroller.updateState(u);
			}
			analogRef.set(ref);
			tsleep(t);
	
		}

	}
}