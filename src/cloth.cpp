#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
	int num_height_points, float thickness) {
	this->width = width;
	this->height = height;
	this->num_width_points = num_width_points;
	this->num_height_points = num_height_points;
	this->thickness = thickness;

	buildGrid();
	buildClothMesh();
}

Cloth::~Cloth() {
	point_masses.clear();
	springs.clear();

	if (clothMesh) {
		delete clothMesh;
	}
}

void Cloth::buildGrid() {
	// TODO (Part 1): Build a grid of masses and springs.

	// Initializng point masses
	for (int j = 0; j < num_width_points; j++) {
		for (int i = 0; i < num_height_points; i++) {
			bool pin = std::any_of(pinned.begin(), pinned.end(), [i,j](vector<int> xy){return xy[0] == i && xy[1] == j;});
			double h = (height / num_height_points) * i;
			double w = (width / num_width_points) * j;
			if (orientation == HORIZONTAL) {
				
				point_masses.emplace_back(PointMass(Vector3D(h, 1, w), pin));
			}
			else {
				double z = (((double)std::rand() / (RAND_MAX/2.0)) - 1.0) / 1000.0;
				point_masses.emplace_back(PointMass(Vector3D(h, w, z), pin));
			}
		}
	}

	// Initializing springs
	for (int i = 0; i < num_height_points; i++) {
		for (int j = 0; j < num_width_points; j++) {
			int curr = i * num_width_points + j;

			// Structural Constraints
			if (i > 0) { // Skipping first row
				// Above
				int above = (i - 1) * num_width_points + j;
				springs.emplace_back(Spring(&point_masses[curr], &point_masses[above], STRUCTURAL));
			}
			if (j > 0) { // Skipping first column
				// Left
				springs.emplace_back(Spring(&point_masses[curr], &point_masses[curr - 1], STRUCTURAL));
			}

			// Shearing Constraints
			if (i > 0) { // Skipping first row
				if (j > 0) { // Skipping first column
					// Diagonal Upper Left
					int upLeft = (i - 1) * num_width_points + (j - 1);
					springs.emplace_back(Spring(&point_masses[curr], &point_masses[upLeft], SHEARING));
				}
				if (j < num_width_points - 1) { // Skipping last column
					// Diagonal Upper Right
					int upRight = (i - 1) * num_width_points + (j + 1);
					springs.emplace_back(Spring(&point_masses[curr], &point_masses[upRight], SHEARING));
				}
			}

			// Bending Constraints
			if (i > 1) { // Skipping first two rows
				// Two Above
				int twoAbove = (i - 2) * num_width_points + j;
				springs.emplace_back(Spring(&point_masses[curr], &point_masses[twoAbove], BENDING));
			}
			if (j > 1) { // Skipping first two columns
				// Two Left
				int twoLeft = i * num_width_points + (j - 2);
				springs.emplace_back(Spring(&point_masses[curr], &point_masses[twoLeft], BENDING));
			}
		}
	}
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters* cp,
	vector<Vector3D> external_accelerations,
	vector<CollisionObject*>* collision_objects) {
	double mass = width * height * cp->density / num_width_points / num_height_points;
	double delta_t = 1.0f / frames_per_sec / simulation_steps;

	// TODO (Part 2): Compute total force acting on each point mass.
	Vector3D externalAcceleration;
	for (int i = 0; i < external_accelerations.size(); i++) {
		externalAcceleration += external_accelerations[i];
	}
	Vector3D totalForce = mass * externalAcceleration;
	for (int i = 0; i < point_masses.size(); i++) {
		point_masses[i].forces = totalForce;
	}

	std::map<e_spring_type, bool> enabledConstraints;
	enabledConstraints[STRUCTURAL] = cp->enable_structural_constraints;
	enabledConstraints[SHEARING] = cp->enable_shearing_constraints;
	enabledConstraints[BENDING] = cp->enable_bending_constraints;
	
	for (int i = 0; i < springs.size(); i++) {
		if (enabledConstraints[springs[i].spring_type]) {
			double springForce = cp->ks * ((springs[i].pm_a->position - springs[i].pm_b->position).norm() - springs[i].rest_length);
			if (springs[i].spring_type == BENDING) {
				springForce *= 0.2;
			}
			springs[i].pm_a->forces -= springForce * (springs[i].pm_a->position - springs[i].pm_b->position).unit();
			springs[i].pm_b->forces += springForce * (springs[i].pm_a->position - springs[i].pm_b->position).unit();
		}
	}

	// TODO (Part 2): Use Verlet integration to compute new point mass positions
	for (int i = 0; i < point_masses.size(); i++) {
		if (!point_masses[i].pinned) {
			Vector3D totalAcceleration = point_masses[i].forces / mass;
			Vector3D newPosition = point_masses[i].position + (1.0 - (cp->damping / 100.0)) * (point_masses[i].position - point_masses[i].last_position) + (totalAcceleration * delta_t * delta_t);
			point_masses[i].last_position = point_masses[i].position;
			point_masses[i].position = newPosition;
		}
	}

	// TODO (Part 4): Handle self-collisions.
	build_spatial_map();
	for (int i = 0; i < point_masses.size(); i++) {
		self_collide(point_masses[i], simulation_steps);
	}

	// TODO (Part 3): Handle collisions with other primitives.
	for (int i = 0; i < point_masses.size(); i++) {
		for (CollisionObject* object : *collision_objects) {
			object->collide(point_masses[i]);
		}
	}

	// TODO (Part 2): Constrain the changes to be such that the spring does not change
	// in length more than 10% per timestep [Provot 1995].
	for (int i = 0; i < springs.size(); i++) {
		double length = (springs[i].pm_a->position - springs[i].pm_b->position).norm();
		double maxLength = springs[i].rest_length * 1.10;
		if (length > maxLength) {
			double diffLength = length - maxLength;
			Vector3D diffVector = (springs[i].pm_a->position - springs[i].pm_b->position).unit();
			bool aPin = springs[i].pm_a->pinned;
			bool bPin = springs[i].pm_b->pinned;
			if (!aPin && !bPin) {
				springs[i].pm_a->position -= (diffLength / 2.0) * diffVector;
				springs[i].pm_b->position += (diffLength / 2.0) * diffVector;
			}
			else if (!aPin && bPin) {
				springs[i].pm_a->position -= diffLength * diffVector;
			}
			else {
				springs[i].pm_b->position += diffLength * diffVector;
			}
		}
	}
}

void Cloth::build_spatial_map() {
	for (const auto& entry : map) {
		delete(entry.second);
	}
	map.clear();

	// TODO (Part 4): Build a spatial map out of all of the point masses.
	for (int i = 0; i < point_masses.size(); i++) {
		float hash = hash_position(point_masses[i].position);
		if (map.find(hash) == map.end()) {
			map.insert(make_pair(hash, new std::vector<PointMass*>()));
		}
		map[hash]->push_back(&point_masses[i]);
	}
}

void Cloth::self_collide(PointMass& pm, double simulation_steps) {
	// TODO (Part 4): Handle self-collision for a given point mass.
	std::vector<PointMass*>* candidates = map[hash_position(pm.position)];
	Vector3D correctionVector;
	int counter = 0;
	for (int i = 0; i < candidates->size(); i++) {
		PointMass* candidate = candidates->at(i);
		double distance = (pm.position - candidate->position).norm();
		if (distance < 2 * thickness && distance != 0) {
			correctionVector += (2 * thickness - distance) * (candidate->position - pm.position).unit();
			counter++;
		}
	}
	if (counter > 0) {
		pm.position = pm.position - (correctionVector / counter) / simulation_steps;
	}
}

float Cloth::hash_position(Vector3D pos) {
	// TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
	double w = 3 * width / num_width_points;
	double h = 3 * height / num_height_points;
	double t = max(w, h);

	/*int x = pos.x - fmod(pos.x, w);
	int y = pos.y -fmod(pos.y, h);
	int z = pos.z - fmod(pos.z, t);*/

	Vector3D newPos = { 
		pos.x - fmod(pos.x, w) ,
		pos.y - fmod(pos.y,h),
		pos.z - fmod(pos.z,t)
	};

	return newPos.x + newPos.y + newPos.z / t;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
	PointMass* pm = &point_masses[0];
	for (int i = 0; i < point_masses.size(); i++) {
		pm->position = pm->start_position;
		pm->last_position = pm->start_position;
		pm++;
	}
}

void Cloth::buildClothMesh() {
	if (point_masses.size() == 0) return;

	ClothMesh* clothMesh = new ClothMesh();
	vector<Triangle*> triangles;

	// Create vector of triangles
	for (int y = 0; y < num_height_points - 1; y++) {
		for (int x = 0; x < num_width_points - 1; x++) {
			PointMass* pm = &point_masses[y * num_width_points + x];
			// Get neighboring point masses:
			/*                      *
			 * pm_A -------- pm_B   *
			 *             /        *
			 *  |         /   |     *
			 *  |        /    |     *
			 *  |       /     |     *
			 *  |      /      |     *
			 *  |     /       |     *
			 *  |    /        |     *
			 *      /               *
			 * pm_C -------- pm_D   *
			 *                      *
			 */

			float u_min = x;
			u_min /= num_width_points - 1;
			float u_max = x + 1;
			u_max /= num_width_points - 1;
			float v_min = y;
			v_min /= num_height_points - 1;
			float v_max = y + 1;
			v_max /= num_height_points - 1;

			PointMass* pm_A = pm;
			PointMass* pm_B = pm + 1;
			PointMass* pm_C = pm + num_width_points;
			PointMass* pm_D = pm + num_width_points + 1;

			Vector3D uv_A = Vector3D(u_min, v_min, 0);
			Vector3D uv_B = Vector3D(u_max, v_min, 0);
			Vector3D uv_C = Vector3D(u_min, v_max, 0);
			Vector3D uv_D = Vector3D(u_max, v_max, 0);


			// Both triangles defined by vertices in counter-clockwise orientation
			triangles.push_back(new Triangle(pm_A, pm_C, pm_B,
				uv_A, uv_C, uv_B));
			triangles.push_back(new Triangle(pm_B, pm_C, pm_D,
				uv_B, uv_C, uv_D));
		}
	}

	// For each triangle in row-order, create 3 edges and 3 internal halfedges
	for (int i = 0; i < triangles.size(); i++) {
		Triangle* t = triangles[i];

		// Allocate new halfedges on heap
		Halfedge* h1 = new Halfedge();
		Halfedge* h2 = new Halfedge();
		Halfedge* h3 = new Halfedge();

		// Allocate new edges on heap
		Edge* e1 = new Edge();
		Edge* e2 = new Edge();
		Edge* e3 = new Edge();

		// Assign a halfedge pointer to the triangle
		t->halfedge = h1;

		// Assign halfedge pointers to point masses
		t->pm1->halfedge = h1;
		t->pm2->halfedge = h2;
		t->pm3->halfedge = h3;

		// Update all halfedge pointers
		h1->edge = e1;
		h1->next = h2;
		h1->pm = t->pm1;
		h1->triangle = t;

		h2->edge = e2;
		h2->next = h3;
		h2->pm = t->pm2;
		h2->triangle = t;

		h3->edge = e3;
		h3->next = h1;
		h3->pm = t->pm3;
		h3->triangle = t;
	}

	// Go back through the cloth mesh and link triangles together using halfedge
	// twin pointers

	// Convenient variables for math
	int num_height_tris = (num_height_points - 1) * 2;
	int num_width_tris = (num_width_points - 1) * 2;

	bool topLeft = true;
	for (int i = 0; i < triangles.size(); i++) {
		Triangle* t = triangles[i];

		if (topLeft) {
			// Get left triangle, if it exists
			if (i % num_width_tris != 0) { // Not a left-most triangle
				Triangle* temp = triangles[i - 1];
				t->pm1->halfedge->twin = temp->pm3->halfedge;
			}
			else {
				t->pm1->halfedge->twin = nullptr;
			}

			// Get triangle above, if it exists
			if (i >= num_width_tris) { // Not a top-most triangle
				Triangle* temp = triangles[i - num_width_tris + 1];
				t->pm3->halfedge->twin = temp->pm2->halfedge;
			}
			else {
				t->pm3->halfedge->twin = nullptr;
			}

			// Get triangle to bottom right; guaranteed to exist
			Triangle* temp = triangles[i + 1];
			t->pm2->halfedge->twin = temp->pm1->halfedge;
		}
		else {
			// Get right triangle, if it exists
			if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
				Triangle* temp = triangles[i + 1];
				t->pm3->halfedge->twin = temp->pm1->halfedge;
			}
			else {
				t->pm3->halfedge->twin = nullptr;
			}

			// Get triangle below, if it exists
			if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
				Triangle* temp = triangles[i + num_width_tris - 1];
				t->pm2->halfedge->twin = temp->pm3->halfedge;
			}
			else {
				t->pm2->halfedge->twin = nullptr;
			}

			// Get triangle to top left; guaranteed to exist
			Triangle* temp = triangles[i - 1];
			t->pm1->halfedge->twin = temp->pm2->halfedge;
		}

		topLeft = !topLeft;
	}

	clothMesh->triangles = triangles;
	this->clothMesh = clothMesh;
}
