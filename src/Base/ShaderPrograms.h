/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SHADER_PROGRAMS_H
#define CNOID_BASE_SHADER_PROGRAMS_H

#include "GLSLProgram.h"
#include <cnoid/SceneCameras>
#include <vector>

namespace cnoid {

class SgLight;

class ShaderProgram : public GLSLProgram
{
public:
    ShaderProgram();
    virtual ~ShaderProgram();
    virtual void initialize() = 0;
    virtual void activate();
    virtual void initializeRendering();
    virtual void deactivate();
    virtual void setColor(const Vector3f& color);
    virtual void enableColorArray(bool on);
};


class NolightingProgram : public ShaderProgram
{
    GLint MVPLocation;
    
public:
    virtual void initialize();
    void setProjectionMatrix(const Matrix4f& PVM);
};


class SolidColorProgram : public NolightingProgram
{
    GLint pointSizeLocation;
    GLint colorLocation;
    GLint colorPerVertexLocation;
    
public:
    virtual void initialize();
    virtual void initializeRendering();
    virtual void setPointSize(float s);
    virtual void setColor(const Vector3f& color);
    virtual void enableColorArray(bool on);
};


class LightingProgram : public ShaderProgram
{
    static const int maxNumLights_ = 10;

protected:

    GLint diffuseColorLocation;
    GLint ambientColorLocation;
    GLint specularColorLocation;
    GLint emissionColorLocation;
    GLint shininessLocation;
    GLint alphaLocation;
    
public:
    virtual void initialize();
    int maxNumLights() const { return maxNumLights_; }
    virtual void setNumLights(int n) = 0;
    virtual bool renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix, bool shadowCasting) = 0;
    virtual void setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV) = 0;

    void setDiffuseColor(const Vector3f& color){
        glUniform3fv(diffuseColorLocation, 1, color.data());
    }
    void setAmbientColor(const Vector3f& color){
        glUniform3fv(ambientColorLocation, 1, color.data());
    }
    void setEmissionColor(const Vector3f& color){
        glUniform3fv(emissionColorLocation, 1, color.data());
    }
    void setSpecularColor(const Vector3f& color){
        glUniform3fv(specularColorLocation, 1, color.data());
    }
    void setShininess(float s){
        glUniform1f(shininessLocation, s);
    }
    void setAlpha(float a){
        glUniform1f(alphaLocation, a);
    }
};


class ShadowMapProgram;

class PhongShadowProgram : public LightingProgram
{
    bool useUniformBlockToPassTransformationMatrices;
    GLSLUniformBlockBuffer transformBlockBuffer;
    GLint modelViewMatrixIndex;
    GLint normalMatrixIndex;
    GLint MVPIndex;

    GLint modelViewMatrixLocation;
    GLint normalMatrixLocation;
    GLint MVPLocation;

    GLint numLightsLocation;

    struct LightInfo {
        GLint positionLocation;
        GLint intensityLocation;
        GLint ambientIntensityLocation;
        GLint constantAttenuationLocation;
        GLint linearAttenuationLocation;
        GLint quadraticAttenuationLocation;
        GLint cutoffAngleLocation;
        GLint beamWidthLocation;
        GLint cutoffExponentLocation;
        GLint directionLocation;
    };
    std::vector<LightInfo> lightInfos;

    bool isShadowAntiAliasingEnabled_;
    int numShadows_;
    int shadowMapWidth_;
    int shadowMapHeight_;
    SgPerspectiveCameraPtr persShadowCamera;  
    SgOrthographicCameraPtr orthoShadowCamera;
    ShadowMapProgram* shadowMapProgram_;
    
    GLint isShadowEnabledLocation;
    GLint numShadowsLocation;
    GLint isShadowAntiAliasingEnabledLocation;

    static const int maxNumShadows_ = 2;
    int currentShadowIndex;

    struct ShadowInfo {
        int lightIndex;
        GLint shadowMatrixLocation;
        GLint lightIndexLocation;
        GLint shadowMapLocation;
        GLuint depthTexture;
        GLuint frameBuffer;
        Matrix4 BPV;
    };
    std::vector<ShadowInfo> shadowInfos;

    Matrix4 shadowBias;

    GLint maxFogDistLocation;
    GLint minFogDistLocation;
    GLint fogColorLocation;
    GLint isFogEnabledLocation;

public:
    PhongShadowProgram();
    ~PhongShadowProgram();

    virtual void initialize();
    virtual void activate();
    virtual void initializeRendering();
    virtual void setNumLights(int n);
    virtual bool renderLight(int index, const SgLight* light, const Affine3& T, const Affine3& viewMatrix, bool shadowCasting);
    virtual void setTransformMatrices(const Affine3& viewMatrix, const Affine3& modelMatrix, const Matrix4& PV);

    void activateShadowMapGenerationPass(int shadowIndex);
    void activateMainRenderingPass();

    int maxNumShadows() const { return maxNumShadows_; }
    void setNumShadows(int n);
    void setShadowAntiAliasingEnabled(bool on) { isShadowAntiAliasingEnabled_ = on; }
    bool isShadowAntiAliasingEnabled() const { return isShadowAntiAliasingEnabled_; }
    int shadowMapWidth() const { return shadowMapWidth_; }
    int shadowMapHeight() const { return shadowMapHeight_; }
    SgCamera* getShadowMapCamera(SgLight* light, Affine3& io_T);
    void setShadowMapViewProjection(const Matrix4& PV);
    ShadowMapProgram& shadowMapProgram() { return *shadowMapProgram_; }
    void setFogEnabled(bool on) {
        glUniform1i(isFogEnabledLocation, on);
    }
    void setFogColor(const Vector3f& color) {
        glUniform3fv(fogColorLocation, 1, color.data());
    }
    void setFogRange(float minDist, float maxDist){
        glUniform1f(minFogDistLocation, minDist);
        glUniform1f(maxFogDistLocation, maxDist);
    }

private:
    void initializeShadowInfo(int index);

    friend class ShadowMapProgram;
};


class ShadowMapProgram : public NolightingProgram
{
    PhongShadowProgram* mainProgram;
    
public:
    ShadowMapProgram(PhongShadowProgram* mainProgram);
    virtual void initialize();
    virtual void activate();
    virtual void initializeRendering();
    virtual void deactivate();
};

}

#endif
