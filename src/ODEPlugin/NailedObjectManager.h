/*!
  Nailed object manager.
  @file NailedObjectManager.h
  @author
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

/**
   @brief Nailed object.
 */
class CNOID_EXPORT NailedObject : public Referenced
{
public:
    /**
       @brief Copy constructor.
       @parame[in] worldID Set the dWorldId.
       @parame[in] objID Set the nailed object's dBodyID.
     */
    NailedObject(dWorldID worldID, dBodyID objID) {
        nailCount = 0;
        maxFasteningForce = 0;
        objId_ = objID;

	jointID = dJointCreateFixed(worldID, 0);
	dJointAttach(jointID, 0, objId_);
	dJointSetFixed(jointID);
	dJointSetFeedback(jointID, new dJointFeedback());
    }

    /**
       @brief Destructor.
     */
    ~NailedObject();

    /**
       @brief Hit the nail with additional. (Strengthing max fastening force)
       The nail count will increase when calling this method.
       @param[in] fasteningForce This value adds max fastening force.
     */
    void addNail(double fasteningForce) {
        nailCount++;
        maxFasteningForce += fasteningForce;
    }

    /**
       @brief Check whether limit of max fastening force exceeded.
       @param[in] currentTime Current simulation time.
       @retval true Limit is exceeded.
       @retval false Limit is not exceeded.
     */
    bool isLimited(double currentTime);

    /**
       @brief Getting max fastening force value.
       @return Max fastening force value.
     */
    const double getMaxFasteningForce() { return maxFasteningForce; }

    /**
       @brief Getting nail count.
       The nail count will increase when calling addNail method.
       @return The nail count.
     */
    int getNailCount() {
        return nailCount;
    }

    /**
       @brief Set nail's direction.
       @param[in] n Set nail's direction by Vector3.
     */
    void setNailDirection(const Vector3 n) { n_ = n; }

    /**
       @brief Getting nailed object's BodyID.
       @return Naied object's BodyID.
     */
    dBodyID getBodyID() { return objId_; }

private:
    double maxFasteningForce;
    int nailCount;
    dBodyID objId_;
    dJointID jointID;
    Vector3 n_;
};

typedef ref_ptr<NailedObject> NailedObjectPtr;

/// Map of nailed object manager.
typedef std::map<dBodyID, NailedObjectPtr> NailedObjectMap;

/**
   @brief Nailed object manager.
 */
class CNOID_EXPORT NailedObjectManager
{
public:
    /**
       @brief Retruning nailed object manager instance.
       @return Nailed object manager instance.
     */
    static NailedObjectManager* getInstance();

private:
    NailedObjectManager();
    ~NailedObjectManager();

public:
    /**
       @brief Add nailed object.
       @param[in] obj Pointer of nailed object.
     */
    void addObject(NailedObjectPtr obj);

    /**
       @brief Find nailed object.
       @param[in] Target's bodyID.
       @retval true Nailed object found.
       @retval false Nailed object not found.
     */
    bool find(dBodyID bodyID);

    /**
       @brief Getting nailed object.
       @param[in] bodyID Target's bodyID.
       @return If return not zero is exists nailed object. If return zero is not exists nailed object.
     */
    NailedObjectPtr get(dBodyID bodyID);

    /**
       @brief Remove all nailed object.
     */
    void clear();

    /**
       @brief Returning map reference of nailed object manager.
       @return Map reference of nailed object manager.
     */
    NailedObjectMap& map() { return objectMap; }

private:
    NailedObjectMap objectMap;
};

}

#endif
