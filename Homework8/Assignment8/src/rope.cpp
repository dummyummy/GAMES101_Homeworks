#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D d_pos((end.x - start.x) / (num_nodes - 1), (end.y - start.y) / (num_nodes - 1));
        for (int i = 0; i < num_nodes; i++)
        {
            masses.push_back(new Mass(start + i * d_pos, node_mass, false));
            if (i)
                springs.push_back(new Spring(masses[i - 1], masses[i], k));
        }

//        Comment-in this part when you implement the constructor
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
       }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D force = -s->k *
                             (s->m2->position - s->m1->position).unit() *
                             ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m1->forces += -force;
            s->m2->forces += force;
        }
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // TODO (Part 2): Add global damping
                double damping_factor = 0.005;
                m->forces += -damping_factor * m->velocity;
                auto accel = m->forces / m->mass + gravity;
                auto old_v = m->velocity;
                m->velocity += accel * delta_t;
                // explicit method: use velocity of last tick to update
                // will diverge!
                // m->position += old_v * delta_t;

                // semi-implicit method: use velocity of next tick to update
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D force = -s->k *
                             (s->m2->position - s->m1->position).unit() *
                             ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m1->forces += -force;
            s->m2->forces += force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                double damping_factor = 0.00001;
                auto accel = m->forces / m->mass + gravity;
                m->position = m->position + 
                              (1 - damping_factor) * (m->position - m->last_position) +
                              accel * delta_t * delta_t;
                m->last_position = temp_position;
            }

            m->forces = Vector2D(0, 0);
        }
    }
}
