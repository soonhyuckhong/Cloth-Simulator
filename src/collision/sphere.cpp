#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass& pm) {
	// TODO (Part 3): Handle collisions with spheres.
	// origin to position. (direction vector) expand to length radius (to get to surface) subtract from position with scale (1-f) to get new position
	
	Vector3D radiusDiff = origin - pm.position;
	if (radiusDiff.norm() <= radius) {
		Vector3D tangentPoint = origin - (radiusDiff.unit() * radius);
		Vector3D correctionVector = tangentPoint - pm.last_position;
		pm.position = pm.last_position + (1 - friction) * correctionVector;
	}
}

void Sphere::render(GLShader& shader) {
	// We decrease the radius here so flat triangles don't behave strangely
	// and intersect with the sphere when rendered
	m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
