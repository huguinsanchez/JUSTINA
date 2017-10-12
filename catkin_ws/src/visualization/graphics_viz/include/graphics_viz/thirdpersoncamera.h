#ifndef THIRDPERSONCAMERA_H
#define THIRDPERSONCAMERA_H
#include "camera.h"

class ThirdPersonCamera: public Camera
{
public:
    ThirdPersonCamera();
    void mouseMoveCamera(float xoffset, float yoffset, int dt);
    void scrollMoveCamera(float soffset, int dt);
    void updateCamera();
    void setCameraTarget(glm::vec3 cameraTarget){
        this->cameraTarget = cameraTarget;
    }
    void setAngleTarget(float angleTarget){
        this->angleTarget = angleTarget;
    }

private:
    glm::vec3 cameraTarget;
    float angleTarget;
    float distanceFromTarget;
    float angleAroundTarget;
};

#endif // THIRDPERSONCAMERA_H
