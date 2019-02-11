//
// Created by dlalancette on 19/02/19.
//
#include "ObjectRotator.h"

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <glm/gtx/string_cast.hpp>

static const glm::vec3 yAxis = glm::vec3(0.0f, 1.0f, 0.0f);

void ObjectRotator::renderCube()
{
    /// glm Clamp didn't seem to work, so I did it myself.
    interpolationParam = interpolationParam + 0.1f > 1.0f ? 1.0f : interpolationParam + 0.1f;

    ///LERP
    currentAngle = interpolationParam * desiredAngle + (1 - interpolationParam) * currentAngle;

    auto transform = glm::mat4(1.0f);
    transform = glm::rotate(transform, (glm::mediump_float)currentAngle, yAxis);
    render(transform);
}

void ObjectRotator::setRotation(float radians)
{
    currentAngle = radians;
}

void ObjectRotator::setRotation(const glm::vec3 &normal, float radians)
{
    interpolationParam = 0.0f;

    /// The magic number "1.8f" is there to compensate for the lack of accuracy of the normal calculation.
    /// It works quite well to make the cube follow the real world rotation more closely. It can only achieve about 50%
    /// of the angles of the physical object otherwise.
    float angleRad = 1.8f * glm::sign(normal.x) * glm::atan(abs(normal.x) / normal.z);

    desiredAngle = -radians + angleRad;

    float directDifference = abs(desiredAngle - currentAngle);
    float wrapDifference = abs((desiredAngle + 2 * PI) - currentAngle);

    std::cout << "Direct difference: " <<  directDifference << std::endl;
    std::cout << "Wrap difference:   " << wrapDifference << std::endl;

    if (directDifference  > wrapDifference)
    {
        desiredAngle += + PI * 2;
    }
}
