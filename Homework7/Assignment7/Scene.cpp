//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum; // interesting method to select an object!
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter, shadow, interLight;
    Vector3f L_light, L_dir;
    inter = intersect(ray);
    if (!inter.happened)
        return Vector3f(0.0f, 0.0f, 0.0f);
    if (inter.m->hasEmission())
        return inter.m->getEmission();
    float pdfLight = 1.0f;
    sampleLight(interLight, pdfLight);
    Vector3f ws = interLight.coords - inter.coords;
    float len = ws.norm();
    ws = normalize(ws);
    shadow = intersect(Ray(inter.coords, ws));
    const float EPS = 0.001f;
    if (!shadow.happened || shadow.distance >= len - EPS)
    {
        L_light = interLight.emit * inter.m->eval(ray.direction, ws, inter.normal) *
                  dotProduct(-ws, interLight.normal) * dotProduct(ws, inter.normal) /
                  (len * len * pdfLight);
    }
    if (get_random_float() < RussianRoulette)
    {
        auto wi = inter.m->sample(ray.direction, inter.normal);
        Ray ri = Ray(inter.coords, wi);
        Intersection interTest = intersect(ri);
        if (!(interTest.m && interTest.m->hasEmission()))
            L_dir = castRay(ri, depth) * inter.m->eval(ray.direction, ri.direction, inter.normal) * 
                    dotProduct(ri.direction, inter.normal) /
                    (RussianRoulette * inter.m->pdf(ray.direction, ri.direction, inter.normal));
    }
    
    return L_light + L_dir;
}