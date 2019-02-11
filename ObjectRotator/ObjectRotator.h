//
// Created by dlalancette on 19/02/19.
//

#ifndef DETECTION3D_OBJECTROTATOR_H
#define DETECTION3D_OBJECTROTATOR_H

#include <ml.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/quaternion.hpp>


typedef void (*RenderFunction)(glm::mat4 &);

class ObjectRotator
{
public:
    static constexpr float PI = 3.1416f;

    explicit ObjectRotator(RenderFunction function)
        : render(function)
        , interpolationParam(0.0f)
        , desiredAngle(0.0f)
        , currentAngle(0.0f)
    {

    }

    void setRotation(float radians);
    void setRotation(const glm::vec3 &normal, float radians);

    void renderCube();

private:
    RenderFunction render;

    float interpolationParam;

    float currentAngle;
    float desiredAngle;

};

#endif //DETECTION3D_OBJECTROTATOR_H
