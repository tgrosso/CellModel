package cellModel;

public class CMEGFR extends CMMembraneProtein{
	
	float k_off = .24f; //reverse rate of binding of ligand to receptor units /min
	float k_e = .15f; //internalization rate of bound receptor - units /min
	float k_t = .02f; //internalization rate of unbound receptor - units /min
	float k_D = 2.47f; //Dissociation constant - units nM
	//Data taken from human mammary epithelial cells - diameter about 15 micrometer
	long R_t = 200000; 
		//steady state receptor abundance for whole cell units molecules
	float k_on=k_off/k_D; //forward rate of ligand binding
	float Q_r = R_t * k_t; //Synthesization rate - units molecules/min

	public CMEGFR(CMSimulation s, float[] base){
		super(s, (200000f / (float)(4 * Math.PI * 7.5 * 7.5)), (200000f / (float)(4 * Math.PI * 7.5 * 7.5)), base, "EGFR");
	}
	
	@Override
	protected long updateFreeReceptors(float ligandConcentration, long currentBound, long currentFree, float freeEndo, float exo, float deltaTime){
		//ligandConcentration units is nM
		//receptor units are molecules
		//surface area units is micron^2
		//deltaT units is minutes
		//portion is the percentage of the synthesized receptors that should be added
		//using forward Euler method
		
		//System.out.println("***Updating Free EGFR***");
		//System.out.print("Ligand Conc " + ligandConcentration);
		long R = currentFree; //Number of free receptors on surface
		//System.out.print(" Free Receptors: " + R);
		long C = currentBound; //Number of bound receptors on surface
		//System.out.print(" Bound Receptors: " + C);
		float dR = deltaTime * (-k_on * R * ligandConcentration + k_off * C - freeEndo * R + exo);
		//System.out.print(" delta R = " + dR);
		float newFree = R + dR;
		//System.out.println( " newFree = " + newFree);
		
		return (long)(newFree);
	}
	
	@Override
	protected long updateBoundReceptors(float ligandConcentration, long currentBound, long currentFree, float boundEndo, float deltaTime){
		//ligandConcentration units is nM
		//receptor units are molecules
		//deltaT units is minutes
		//using forward Euler method
		//System.out.println("***Updating Bound EGFR***");
		//System.out.print("Ligand Conc " + ligandConcentration);
		long R = currentFree; //Number of free receptors on surface
		//System.out.print(" Free Receptors: " + R);
		long C = currentBound; //Number of bound receptors on surface
		//System.out.print(" Bound Receptors: " + C);
		float dC = deltaTime * (k_on * R * ligandConcentration - k_off * C - boundEndo * C);
		//System.out.print(" delta C = " + dC);
		float newBound = C + dC;
		//System.out.println( " newBound = " + newBound);
		
		return (long)(newBound);
	}
	
	@Override
	public boolean bindsToLaminin(){
		return false;
	}
	
	@Override
	public long getInitialProteins(float portion){
		return (long)(R_t * portion);
	}
	
	@Override
	public int bindReceptors(int numLigands, int numFreeReceptors){
		return (int)(Math.round((Math.max(numLigands, numFreeReceptors) * k_on)));
	}
	
	@Override
	protected float getBoundEndocytosisRate(){
		return k_e;
	}
	
	@Override
	protected float getUnboundEndocytosisRate(){
		return k_t;
	}
	
	@Override
	protected float getExocytosisRate(){
		return Q_r;
	}
}