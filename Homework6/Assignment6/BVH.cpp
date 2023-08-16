#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        decltype(objects.begin()) middling;
        auto ending = objects.end();

        if (splitMethod == SplitMethod::SAH)
        {
            std::vector<Bounds3> rbbox;
            rbbox.push_back((*(objects.end() - 1))->getBounds());
            for (int i = objects.size() - 2; i >= 0; i--)
            {
                auto nbbox = Union(*(rbbox.end() - 1), objects[i]->getBounds());
                rbbox.push_back(nbbox);
            }
            float sah = std::numeric_limits<float>::max();
            Bounds3 bbox = objects[0]->getBounds();
            int n = objects.size();
            for (int i = 1; i < n - 1; i++)
            {
                float t = bbox.SurfaceArea() * i + rbbox[n - 1 - i].SurfaceArea() * (n - i);
                if (t < sah)
                {
                    sah = t;
                    middling = objects.begin() + i;
                }
                bbox = Union(bbox, objects[i]->getBounds());
            }
        }
        else // NAIVE
            middling = objects.begin() + objects.size() / 2;

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection isect;
    if (node != nullptr && node->object != nullptr)
    {
        isect = node->object->getIntersection(ray);
        return isect;
    }
    if (node != nullptr && node->bounds.IntersectP(ray, ray.direction_inv,
        {(int)(ray.direction.x <= 0), (int)(ray.direction.y <= 0), (int)(ray.direction.z <= 0)}))
    {
        isect = getIntersection(node->left, ray);
        auto isectRight = getIntersection(node->right, ray);
        if (!isect.happened || isect.distance < ray.t_min || isect.distance > ray.t_max)
            isect = isectRight;
        else if (isectRight.happened && isectRight.distance >= ray.t_min &&
                 isectRight.distance <= ray.t_max && isectRight.distance < isect.distance)
            isect = isectRight;
    }
    return isect;
}