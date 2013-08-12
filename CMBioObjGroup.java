/**
 *  Copyright (C) 2013 Terri A. Grosso, Naralys Batista, Nancy Griffeth
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * 
 * Package: cellModel
 * File: CMCell.java
 * Apr 10, 2013 1:44:54 PM
 */
package cellModel;

import com.bulletphysics.util.ObjectArrayList;
import javax.vecmath.Vector3f;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;

public class CMBioObjGroup {
	private String groupName;
	CMSimulation sim;
	ObjectArrayList<CMBioObj> objList;
	
	public CMBioObjGroup(CMSimulation s, String name){
		groupName = name;
		sim = s;
		objList = new ObjectArrayList<CMBioObj>();
	}
	
	public void addObject(CMBioObj c){
		sim.addBioObject(c);
		objList.add(c);
	}
	
	public CMBioObj getObject(int index){
		return objList.getQuick(index);
	}
	
	public boolean removeObject(CMBioObj c){
		int index = objList.indexOf(c);
		if (index < 0){
			return false;
		}
		objList.remove(index);
		sim.removeBioObject(c);
		c.destroy();
		return true;
	}
	
	public void cleanGroup(){
		int index = 0;
		while (index < objList.size()){
			CMBioObj obj = objList.getQuick(index);
			if (obj.isMarked()){
				if (removeObject(obj)){
					continue; //Don't change index if object is removed
				}
			}
			index++;
		}
	}
	
	public String getName(){
		return groupName;
	}
	
	public Vector3f getCenterOfMass(){
		float mass = 0;
		int num = objList.size();
		Vector3f com = new Vector3f();
		Vector3f center = new Vector3f(0,0,0);
		for (int i = 0; i < num; i++){
			CMBioObj o = objList.getQuick(i);
			com.set(0,0,0);
			mass += o.getMass();
			o.getRigidBody().getCenterOfMassPosition(com);		
			com.scale(o.getMass());
			center.add(com);
		}
		center.scale((float)(1.0/mass));
		return center;
	}
	
	public int getNumObjectsInside(String whichGroup, Vector3f minVec, Vector3f maxVec){
		int numInside = 0;
		int num = objList.size();
		Vector3f min = new Vector3f();
		Vector3f max = new Vector3f();
		for (int i = 0; i < num; i++){
			RigidBody rb = objList.getQuick(i).getRigidBody();
			min.set(0,0,0);
			max.set(0,0,0);
			rb.getAabb(min, max);
			if (min.x >= minVec.x && max.x <= maxVec.x
					&& min.y >= minVec.y && max.y <= maxVec.y
					&& min.z >= minVec.z && max.z <= maxVec.z
					&& groupName.equals(whichGroup)){
				//System.out.println(max.toString());
				numInside++;
			}
		}
		return numInside;
	}
	
	public int getNumMembers(){
		return objList.size();
	}
	
	public void transferGroup(CMBioObjGroup newGroup){
		objList.addAll(newGroup.objList);
		newGroup = null;
	}
}
