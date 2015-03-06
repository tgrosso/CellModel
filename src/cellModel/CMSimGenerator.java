/**
 * Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 * Package: cellModel
 * File: CMSimGenerator.java
 * Jul 27, 2013 12:13:51 PM
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * 
 */
package cellModel;

import java.io.File;
import java.util.TimeZone;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.io.IOException;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.util.Date;

import org.lwjgl.LWJGLException;

import com.bulletphysics.demos.opengl.LWJGL;
import com.bulletphysics.demos.opengl.GLDebugDrawer;

/**
 * @author tagsit
 * The CMSimGenerator is the interface between the GUI and the CMSimulator
 * The user-entered parameters can be read from this class.
 */
public class CMSimGenerator {
	public static File dataDir = null;
	
	public File baseFile = null;
	public int numCells = 1;
	public String pdepeDirectory = "";
	public String timeZone = "GMT";
	public int screenWidth = 900;
	public int screenHeight = 600;
	public float sinkConc = 0;
	public float sourceConc = 0;
	public float distFromSource = 0;
	public float channelWidth = 300f;
	public long endTime = 30; //length of run in SECONDS
	public long timeToSteadyState = 11*60*60; //time to steady state in seconds
	public boolean generateImages = false;
	public float secBetweenOutput = .5f;
	public int secBetweenImages = 60;
	public boolean displayImages = true;
	public long seed = 0;
	
	public CMSimGenerator(File base, long sd){
		seed = sd;
		if (base != null){
			baseFile = base;
			baseFile.mkdir();
		}
	}
	
	public CMSimGenerator(File inputFile, File base, long sd){
		seed = sd;
		String name = inputFile.getName();
		int ind = name.lastIndexOf('.');
		if (ind>0){
			name = name.substring(0, ind);
		}
		if (base != null && name.length() > 0){
			baseFile = new File(base, name);
			baseFile.mkdir(); 
		}
		try{
			FileInputStream fis = new FileInputStream(inputFile);
			InputStreamReader isr = new InputStreamReader(fis);
			BufferedReader br = new BufferedReader(isr);
			String line;
			
			PrintWriter pw = new PrintWriter(new File(baseFile, "inputFile.txt"));
			pw.println("\\\\Seed = " + seed);
			
			while ((line = br.readLine()) != null) {
				pw.println(line);
				if (line.length() > 0 && !line.contains("\\\\")){
					int index = line.indexOf('\t');
					if (index < 0){
						System.err.println("Badly formatted input. Variable<tab>value.");
						System.err.println("Input was: " + line);
						continue;
					}
					String value = line.substring(index);
					value = value.substring(1);
					String var = line.substring(0, index).trim();
					setVar(var, value);
				}
			}
			pw.close();
			br.close();
			isr.close();
			fis.close();
			

		}
		catch(IOException e){
			System.err.println("Error reading startup file. ");
			System.err.println(e.toString());
		}
	}
	
	public CMSimGenerator(File inputFile, File base){
		this(inputFile, base, System.currentTimeMillis());
	}
	
	private void setVar(String v, String val){
		if (v.compareTo("pdepeDirectory") == 0){
			pdepeDirectory = val;
			//System.out.println("pdepeDirectory set to " + val);
		}
		else if (v.compareTo("numCells") == 0){
			try{
				numCells = Integer.parseInt(val);
				System.out.println("numCells set to " + numCells);
			}
			catch(NumberFormatException e){
				System.err.println("numCells must be an integer. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("screenWidth") == 0){
			try{
				screenWidth = Integer.parseInt(val);
				System.out.println("screenWidth set to " + screenWidth);
			}
			catch(NumberFormatException e){
				System.err.println("screenWidth must be an integer. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("screenHeight") == 0){
			try{
				screenHeight = Integer.parseInt(val);
				System.out.println("screenHeight set to " + screenHeight);
			}
			catch(NumberFormatException e){
				System.err.println("screenHeight must be an integer. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("sinkConc") == 0){
			try{
				sinkConc = Float.parseFloat(val);
				System.out.println("sinkConc set to " + sinkConc);
			}
			catch(NumberFormatException e){
				System.err.println("sinkConc must be a float. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("sourceConc") == 0){
			try{
				sourceConc = Float.parseFloat(val);
				System.out.println("sourceConc set to " + sourceConc);
			}
			catch(NumberFormatException e){
				System.err.println("sourceConc must be a float. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("distFromSource") == 0){
			try{
				distFromSource = Float.parseFloat(val);
				System.out.println("distFromSource set to " + distFromSource);
			}
			catch(NumberFormatException e){
				System.err.println("distFromSource must be a float. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("channelWidth") == 0){
			try{
				channelWidth = Float.parseFloat(val);
				System.out.println("channelWidth set to " + channelWidth);
			}
			catch(NumberFormatException e){
				System.err.println("channelWidth must be a float. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("secBetweenOutput") == 0){
			try{
				secBetweenOutput = Float.parseFloat(val);
				System.out.println("secBetweenOutput set to " + secBetweenOutput);
			}
			catch(NumberFormatException e){
				System.err.println("secBetweenOutput must be a float. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("generateImages") == 0){
			try{
				generateImages = Boolean.parseBoolean(val);
				System.out.println("generateImages set to " + generateImages);
			}
			catch(NumberFormatException e){
				System.err.println("generateImages must be a boolean. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("endTime") == 0){
			try{
				endTime = Long.parseLong(val);
				System.out.println("endTime set to " + endTime);
			}
			catch(NumberFormatException e){
				System.err.println("endTime must be a long. Found " + val + ". Using default");
			}
		}
		else if (v.compareTo("secBetweenImages") == 0){
			try{
				secBetweenImages = Integer.parseInt(val);
				System.out.println("secBetweenImages set to " + endTime);
			}
			catch(NumberFormatException e){
				System.err.println("secBetweenImages must be an integer. Found " + val + ". Using default");
			}
		}
		else{
			System.err.println("Variable " + v + " not known.");
		}
		
	}
	
	public static void main(String[] args){
		//Usage CMSimGenerator outputDirectory inputfile1 inputFile2 inputFile3...
		long seed = System.currentTimeMillis();
		Date now = new Date();
		SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
	    String dateString = df.format(now);
	    CMSimGenerator[] csgs = null;
	    
	    System.out.println("Number of arguments: " + args.length);
	    if (args.length > 0){
	    	System.out.println("At least one argument");
	    	String outputDir = args[0];
	    	System.out.println("outputDir = " + outputDir);
	    	File testFile = new File(outputDir);
	    	if (testFile.isDirectory()){
	    		try{
	    			File testDoc = new File(testFile, "test.txt");
	    			testDoc.createNewFile();
	    			testDoc.delete();
	    			dataDir = new File(testFile, "CM-" + dateString);
	    		}
	    		catch (IOException e){
	    			System.err.println("Cannot write to " + outputDir + ". Data Saved In Run Directory");
		    		dataDir = new File("CM-"+ dateString);
	    		}
	    	}
	    	else{
	    		System.out.println("Testfile is not directory");
	    	}
	    }
	    
	    if (dataDir == null){
	    	System.err.println("Output Directory Unavailable. Data Saved In Run Directory");
	    	dataDir = new File("CM-"+ dateString);
	    }
		dataDir.mkdir();
	 
		
		if (args.length > 1){
			csgs = new CMSimGenerator[args.length-1];
			for (int i = 1; i < args.length; i++){
				System.out.println(args[i]);
				File f = new File(args[i]);
				csgs[i-1] = new CMSimGenerator(f, dataDir, seed);
			}
		}
		else{
			csgs = new CMSimGenerator[1];
			System.err.println("No input files - Using default values.");
			csgs[0] = new CMSimGenerator(dataDir, seed);
		}
		
		for (int i = 0; i < csgs.length; i++){
			CMSimulation sim = new CMSimulation(LWJGL.getGL(), csgs[i]);
			sim.initPhysics();
			sim.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));

			try{
				CMLWJGL.main(args, csgs[i].screenWidth, csgs[i].screenHeight, "Cell Simulation", sim);
			}
			catch(LWJGLException e){
				System.err.println("Could not run simulation.  Error: " + e.toString());
			}
		}
	}
	
}