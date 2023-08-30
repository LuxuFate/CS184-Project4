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
    double hSpace = height/num_height_points;
    double wSpace = width/num_width_points;
    for (int i = 0; i < num_height_points; i++) {
        for (int j = 0; j < num_width_points; j++) {
            Vector3D v;
            if (this->orientation == 0) {
                v = Vector3D(j * wSpace, 1, i * hSpace);
            } else {
                v = Vector3D(j * wSpace, i * hSpace,
                                      (((double)(rand()/(double)RAND_MAX))/(double)500)-0.001);
            }
            bool pin = false;
            for (auto iter : this->pinned){
                if (iter[0] == j && iter[1] == i) {
                    pin = true;
                    break;
                }
            }
            point_masses.push_back(PointMass(v, pin));
        }
    }
    for (int y = 0; y < num_height_points; y++) {
        for (int x = 0; x < num_width_points; x++) {
            if (x > 0) {
                Spring Leftspring = Spring(&point_masses[y * num_width_points + x],
                                           &point_masses[y * num_width_points + (x-1)], STRUCTURAL);
                springs.emplace_back(Leftspring);
            }
            if (y+1 < num_height_points) {
                Spring Abovespring = Spring(&point_masses[y * num_width_points + x],
                                            &point_masses[(y+1) * num_width_points + x], STRUCTURAL);
                springs.emplace_back(Abovespring);
            }
            if (x > 0 && y > 0) {
                Spring dLeftpring = Spring(&point_masses[y * num_width_points + x],
                                           &point_masses[(y-1) * num_width_points + (x-1)], SHEARING);
                springs.emplace_back(dLeftpring);
            }
            if (x+1 < num_width_points && y > 0) {
                Spring dRightspring = Spring(&point_masses[y * num_width_points + x],
                                             &point_masses[(y-1) * num_width_points + (x+1)], SHEARING);
                springs.emplace_back(dRightspring);
            }
            if (x-1 > 0) {
                Spring tLeftspring = Spring(&point_masses[y * num_width_points + x],
                                            &point_masses[y * num_width_points + (x-2)], BENDING);
                springs.emplace_back(tLeftspring);
            }
            if (y-1 > 0) {
                Spring tAbovespring = Spring(&point_masses[y * num_width_points + x],
                                             &point_masses[(y-2) * num_width_points + x], BENDING);
                springs.emplace_back(tAbovespring);
            }
        }
    }
    
    
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp -> density / num_width_points / num_height_points;

  // TODO (Part 2): Compute total force acting on each point mass.
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* p = &point_masses[i];
      for (int j = 0; j < external_accelerations.size(); j++) {
          p -> forces += mass * external_accelerations[j]; //F = ma
    }
  }
    
  for (int i = 0; i < springs.size(); i++) {
      Spring* s = &springs[i];
      Vector3D a = s -> pm_a -> position;
      Vector3D b = s -> pm_b -> position;
      double restLength = s -> rest_length;
      double springConstant = cp -> ks;
      if (cp->enable_bending_constraints) {
          s -> pm_a -> forces += (springConstant * 0.2) * ((a - b).norm() - restLength) * (b - a).unit();
          s -> pm_b -> forces += (springConstant * 0.2) * ((a - b).norm() - restLength) * (a - b).unit();
      }
      else if (cp -> enable_structural_constraints || cp -> enable_shearing_constraints) {
          s->pm_a->forces += springConstant * ((a - b).norm() - restLength) * (b - a).unit();
          s->pm_b->forces += springConstant * ((a - b).norm() - restLength) * (a - b).unit();
      }
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  double dt = 1.0 / frames_per_sec / simulation_steps;
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* p = &point_masses[i];
      if (!p -> pinned) {
          Vector3D pos = p -> position;
          double damp = (cp -> damping) / 100;
          Vector3D acc = p -> forces / mass;
          p -> position = pos + (1 - damp) * (pos - p->last_position) + acc * std::pow(dt, 2);
          p -> last_position = pos;
      }
      p -> forces = Vector3D(0);
  }

  // TODO (Part 4): Handle self-collisions.
    build_spatial_map();
    for (PointMass &p : point_masses) {
        self_collide(p, simulation_steps);
    }

  // TODO (Part 3): Handle collisions with other primitives.
    
  for (int i = 0; i < (*collision_objects).size(); i++) {
      CollisionObject* c = (*collision_objects)[i];
        for (int j = 0; j < point_masses.size(); j++) {
            PointMass* p = &point_masses[j];
            c -> collide(*p);
      }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
      Spring* s = &springs[i];
      PointMass* pa = s -> pm_a;
      PointMass* pb = s -> pm_b;
      Vector3D posA = pa -> position;
      Vector3D posB = pb -> position;
      double length = (posA - posB).norm() - (s -> rest_length * 1.1);
      bool adjust = !(length <= 0 || (pa -> pinned && pb -> pinned));
      if (adjust) {
          if (pa -> pinned) {
              pb -> position += length * (posA - posB).unit();
          }
          else if (pb->pinned) {
              pa -> position += length * (posB - posA).unit();
          }
          else {
              pa -> position += (length / 2) * (posB - posA).unit();
              pb -> position += (length / 2) * (posA - posB).unit();
          }
      }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
    for (PointMass &p : point_masses) {
        float hash = hash_position(p.position);
        if (map.count(hash) == 0) {
            map[hash] = new vector<PointMass *>();
        }
        map[hash]->push_back(&p);
    }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    Vector3D totalCorr = Vector3D(0, 0, 0);
    int counter = 0;
    double hash = hash_position(pm.position);
    
    for (PointMass *cpm : *map[hash]) {
        if (!(pm.position == cpm->position)) {
            Vector3D vec = cpm->position - pm.position;
            if (vec.norm() < 2*thickness) {
                Vector3D corDir = pm.position - cpm->position;
                corDir.normalize();
                Vector3D correctionVec = corDir * (2*thickness - vec.norm());
                counter++;
                totalCorr += correctionVec;
            }
        }
    }
    if (counter > 0) {
        pm.position += totalCorr/counter/simulation_steps;
    }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    double w = 3.0 * width / num_width_points;
    double h = 3.0 * height / num_height_points;
    double t = max(w, h);
    
    double x = (pos.x - fmod(pos.x, w))/w;
    double y = (pos.y - fmod(pos.y, h))/h;
    double z = (pos.z - fmod(pos.z, t))/t;
    
    float hash = x*2 + x*3 + z*5;
    return hash;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
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
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
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
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

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
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
