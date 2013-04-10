/**
 * Terri A. Grosso
 * Package: cellModel
 * File: CMBioObj.java
 * Mar 17, 2013 10:06:21 AM
 */
package cellModel;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.RigidBody;
import java.util.Random;
import javax.vecmath.Vector3f;


public interface CMBioObj {
	public CollisionShape getCollisionShape();
	public RigidBody getRigidBody();
	public void updateObject(Random r);
	public Vector3f getColor3Vector();
	public boolean isVisible();
}
