package cellModel;

public class CMProteinInteraction {
	
	public static int EXOCYTOSIS = 0, UNBOUND_ENDOCYTOSIS = 1, BOUND_ENDOCYTOSIS = 2;
	
	private CMSimulation sim;
	private CMMembraneProtein signal;
	private CMMembraneProtein target;
	private int signal_id, target_id;
	private float minConc, maxConc;
	private float[] maximumResponses;
	private boolean valid = true;
	
	public CMProteinInteraction(CMSimulation sm, int s, int t, float min, float max){
		sim = sm;
		signal_id = s;
		target_id = t;
		signal = sim.getProtein(s);
		target = sim.getProtein(t);
		minConc = min;
		maxConc = max;
		if (minConc > maxConc){
			valid = false;
			sim.writeToLog("Error creating protein interaction");
			sim.writeToLog("minConc: " + minConc + " maxConc: " + maxConc);
			sim.writeToLog("Response rates will not change.");
		}
		
		maximumResponses = new float[3];
		for (int i = 0; i < 3; i++){
			maximumResponses[i] = 1.0f;
		}
	}
	
	public void setMaxResponse(float r, int rate){
		maximumResponses[rate] = r;
	}
	
	public float changeRate(float conc, float old_value, int rate){
		if (!valid || conc < minConc){
			return old_value;
		}
		if ((conc < maxConc) && (maxConc > minConc)){
			float ratio = (maximumResponses[rate]- 1.0f)/(maxConc - minConc);
			float deltaConc = conc - minConc;
			float response = ratio * deltaConc + 1.0f;
			return old_value * response;
		}
		else{
			return old_value * maximumResponses[rate];
		}
	}
	
	public CMMembraneProtein getSignal(){
		return signal;
	}
	
	public CMMembraneProtein getTarget(){
		return target;
	}
	
	public int getSignalID(){
		return signal_id;
	}
	
	public int getTargetID(){
		return target_id;
	}
}
