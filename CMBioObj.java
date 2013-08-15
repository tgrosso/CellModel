/**
 * /**
 *  Copyright (C) 2013 Terri A. Grosso
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * 
 * Terri A. Grosso
 * Package: cellModel
 * File: CMBioObj.java
 * Mar 17, 2013 10:06:21 AM
 */
package cellModel;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.demos.opengl.IGL;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;

import java.util.Random;
import javax.vecmath.Vector3f;


public interface CMBioObj {
	
	public void collided(CMBioObj c, Vector3f point, Vector3f point2, long collID);
	public void addConstraint(CMGenericConstraint c);
	public void removeConstraint(CMGenericConstraint c);
	public CollisionShape getCollisionShape();
	public CMRigidBody getRigidBody();
	public void updateObject();
	public Vector3f getColor3Vector();
	public void setVisible(boolean v);
	public boolean isVisible();
	public int getID();
	public float getMass();
	public String getType();
	public void destroy();
	public void markForRemoval();
	public boolean isMarked();
	public boolean specialRender(IGL gl, Transform t);
}
