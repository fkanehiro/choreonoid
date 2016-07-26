/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneLights.h"
#include "SceneVisitor.h"

using namespace std;
using namespace cnoid;


SgLight::SgLight()
{
    on_ = true;
    color_.setOnes();
    intensity_ = 1.0f;
    ambientIntensity_ = 0.0f;
}


SgLight::SgLight(const SgLight& org)
    : SgPreprocessed(org)
{
    on_ = org.on_;
    color_ = org.color_;
    intensity_ = org.intensity_;
    ambientIntensity_ = org.ambientIntensity_;
}


SgObject* SgLight::clone(SgCloneMap& cloneMap) const
{
    return new SgLight(*this);
}


void SgLight::accept(SceneVisitor& visitor)
{
    visitor.visitLight(this);
}


SgDirectionalLight::SgDirectionalLight()
{
    direction_ << 0.0, 0.0, -1.0;
}


SgDirectionalLight::SgDirectionalLight(const SgDirectionalLight& org)
    : SgLight(org)
{
    direction_ = org.direction_;
}


SgObject* SgDirectionalLight::clone(SgCloneMap& cloneMap) const
{
    return new SgDirectionalLight(*this);
}


void SgDirectionalLight::accept(SceneVisitor& visitor)
{
    visitor.visitLight(this);
}


SgPointLight::SgPointLight()
{
    constantAttenuation_ = 1.0f;
    linearAttenuation_ = 0.0f;
    quadraticAttenuation_ = 0.0f;
}


SgPointLight::SgPointLight(const SgPointLight& org)
    : SgLight(org)
{
    constantAttenuation_ = org.constantAttenuation_;
    linearAttenuation_ = org.linearAttenuation_;
    quadraticAttenuation_ = org.quadraticAttenuation_;
}


SgObject* SgPointLight::clone(SgCloneMap& cloneMap) const
{
    return new SgPointLight(*this);
}


void SgPointLight::accept(SceneVisitor& visitor)
{
    visitor.visitLight(this);
}


SgSpotLight::SgSpotLight()
{
    direction_ << 0.0, 0.0, -1.0;
    beamWidth_ = 1.570796f;
    cutOffAngle_ = 0.785398f;
    cutOffExponent_ = 1.0f;
}


SgSpotLight::SgSpotLight(const SgSpotLight& org)
    : SgPointLight(org)
{
    direction_ = org.direction_;
    beamWidth_ = org.beamWidth_;
    cutOffAngle_ = org.cutOffAngle_;
    cutOffExponent_ = org.cutOffExponent_;
}


SgObject* SgSpotLight::clone(SgCloneMap& cloneMap) const
{
    return new SgSpotLight(*this);
}


void SgSpotLight::accept(SceneVisitor& visitor)
{
    visitor.visitLight(this);
}
