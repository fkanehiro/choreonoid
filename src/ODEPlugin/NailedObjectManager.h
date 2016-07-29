/*!
  @file
*/

#ifndef CNOID_ODEPLUGIN_NAILEDOBJECTMANAGER_H
#define CNOID_ODEPLUGIN_NAILEDOBJECTMANAGER_H

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <cnoid/Body>
#include <cnoid/Link>
#include <string>
#include <map>
#include "exportdecl.h"

#include <ode/ode.h>

namespace cnoid {

class CNOID_EXPORT NailedObject : public Referenced
{
public:
    NailedObject(dWorldID worldID, dBodyID objID) {
        nailCount = 0;
        maxFasteningForce = 0;
        objId_ = objID;

	jointID = dJointCreateFixed(worldID, 0);
	dJointAttach(jointID, 0, objId_);
	dJointSetFixed(jointID);
	dJointSetFeedback(jointID, new dJointFeedback());
    }
    ~NailedObject();

    void addNail(double fasteningForce) {
        nailCount++;
        maxFasteningForce += fasteningForce;
    }

    bool isLimited(double currentTime);

    const double getMaxFasteningForce() { return maxFasteningForce; }

    int getNailCount() {
        return nailCount;
    }

    void setNailDirection(const Vector3 n) { n_ = n; }

    dBodyID getBodyID() { return objId_; }

private:
    double maxFasteningForce;
    int nailCount;
    dBodyID objId_;
    dJointID jointID;
    Vector3 n_;
};

typedef ref_ptr<NailedObject> NailedObjectPtr;

typedef std::map<dBodyID, NailedObjectPtr> NailedObjectMap;

class CNOID_EXPORT NailedObjectManager
{
public:
    static NailedObjectManager* getInstance();

private:
    NailedObjectManager();
    ~NailedObjectManager();

public:
    void addObject(NailedObjectPtr obj);

    bool find(dBodyID bodyID);

    NailedObjectPtr get(dBodyID bodyID);

    void clear();

    NailedObjectMap& map() { return objectMap; }

private:
    NailedObjectMap objectMap;
};

}

#endif
