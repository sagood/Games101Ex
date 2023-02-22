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
    float p = get_random_float() * emit_area_sum;
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
    // Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);
    if (!inter.happened) {
        return Vector3f();
    }

    if (inter.m -> hasEmission()) {
        return inter.m -> getEmission();
    }

    Vector3f l_dir;
    Vector3f l_indir;

    Intersection lightInter;
    float pdf = 0.0f;
    sampleLight(lightInter, pdf);

    Vector3f obj2light = lightInter.coords - inter.coords;
    Vector3f obj2lightDir = obj2light.normalized();
    float obj2lightPow = obj2light.x * obj2light.x + obj2light.y * obj2light.y + obj2light.z * obj2light.z;

    Ray obj2lightRay(inter.coords, obj2lightDir);
    Intersection t = intersect(obj2lightRay);
    if (t.distance - obj2light.norm() > -EPSILON) {
        l_dir = lightInter.emit * inter.m->eval(ray.direction, obj2lightDir, inter.normal) * dotProduct(obj2lightDir, inter.normal) *
                dotProduct(-obj2lightDir, lightInter.normal) / obj2lightPow / pdf;
    }

    if (get_random_float() > RussianRoulette) {
        return l_dir;
    }

    Vector3f obj2NextObjDir = inter.m->sample(-ray.direction, inter.normal).normalized();
    Ray obj2NextObjRay(inter.coords, obj2NextObjDir);
    Intersection nextObjInter = intersect(obj2NextObjRay);
    if (nextObjInter.happened && !nextObjInter.m->hasEmission()) {
        float pdf2 = inter.m->pdf(-ray.direction, obj2NextObjDir, inter.normal);
        l_indir = castRay(obj2NextObjRay, depth + 1)
                * inter.m->eval(ray.direction, obj2NextObjDir, inter.normal)
                * dotProduct(obj2NextObjDir, inter.normal) / pdf2 / RussianRoulette;
    }

    return l_dir + l_indir;
}