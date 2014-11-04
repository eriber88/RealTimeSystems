
// PID class to be written by you
public class PID {
	// Current PID parameters
	private PIDParameters p;
	//private PIDGUI pidGui;
	private double I, error, v, D, ad, bd, y, yref, yold;

	

	
	// Constructor
	public PID(String name){

		
		PIDParameters p = new PIDParameters();
		p.Beta=1.0;
		p.H=0.05;
		p.integratorOn=false;
		p.K=-0.05;
		p.Ti=0.0;
		p.Tr=10.0;
		p.Td=2.0;
		p.N=7.0;
	
		//pidGui = new PIDGUI(this, p, name);
	
		setParameters(p);
	
		this.I=0.0;
		this.v=0.0;
		this.error=0.0;
	}
	
	// Calculates the control signal v.
	// Called from BallAndBeamRegul.
	public synchronized double calculateOutput(double y, double yref){
		error = yref-y;
		D = ad*D - bd*(y-yold);
		this.y=y;
		if(p.integratorOn){
			v= p.K*(p.Beta*yref - y) + I + D;
		}else {
			v= p.K*(p.Beta*yref - y) + D;
		}
		return this.v;
	}
	
	// Updates the controller state.
	// Should use tracking-based anti-windup
	// Called from BallAndBeamRegul.
	public synchronized void updateState(double u){
		if(p.integratorOn){
			I = I + (p.K*p.H/p.Ti) * error + (p.H / p.Tr) * (u - v); 
		}else{
			I=0;
		}
		yold=this.y;
		
	}
	
	// Sets the I-part of the controller to 0.
	  // For example needed when changing controller mode.
	  public synchronized void reset(){
		  this.I=0;  
	  }
	  
	  // Returns the current PIParameters.
	  public synchronized PIDParameters getParameters(){
		  return (PIDParameters) p.clone();
	  }
	
	
	
	// Returns the sampling interval expressed as a long.
	// Explicit type casting needed.
	public synchronized long getHMillis(){
		return (long) (p.H*1000.0);
	}
	
	// Sets the PIDParameters.
	// Called from PIDGUI.
	// Must clone newParameters.
	public synchronized void setParameters(PIDParameters newParameters){
		p = (PIDParameters)newParameters.clone();
		ad = p.Td / (p.Td + p.N*p.H);
        bd = p.K*ad*p.N;
		if(!p.integratorOn){
			I=0.0; //reset integrator part if control is off.
		}
	}
}