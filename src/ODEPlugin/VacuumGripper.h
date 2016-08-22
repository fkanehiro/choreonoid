/*!
  Vacuum gripper device.
  @file VacuumGripper.h
  @author
*/

#ifndef CNOID_ODEPLUGIN_VACUUMGRIPPER_H
#define CNOID_ODEPLUGIN_VACUUMGRIPPER_H

#include <cnoid/EigenTypes>
#include <cnoid/Body>
#include <cnoid/Link>
#include <string>
#include "exportdecl.h"

#include <ode/ode.h>

namespace cnoid {

/**
   @brief Vacuum gripper device.
 */
class CNOID_EXPORT VacuumGripper : public Device
{
public:
    /**
       @brief Constructor.
     */
    VacuumGripper();

    /**
       @brief Copy constructor.
       @param[in] org Reference of VacuumGripper.
       @param[in] copyStateOnly Set true is copy state only. Set false is copy all. (default false) 
     */
    VacuumGripper(const VacuumGripper& org, bool copyStateOnly = false);

public:
    virtual const char* typeName();

    virtual void copyStateFrom(const DeviceState& other);
    virtual void copyStateFrom(const VacuumGripper& other);
    virtual DeviceState* cloneState() const;

    virtual int stateSize() const;

    virtual Device* clone() const;

    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    /**
       @brief Getting state of vacuum gripper.
       @retval true Vacuum gripper is running.
       @retval false Vacuum gripper is stopping.
     */
    bool on() const { return on_; }

    /**
       @brief Change state of vacuum gripper.
       @param[in] on If set true is vacuum gripper start running, set false is vacuum gripper stop running.
     */
    void on(bool on);

    /**
       @brief Check whether is gripping.
       @retval true It has gripped something.
       @retval false It has gripped nothing.
     */
    bool isGripping() const { return jointID != 0; }

    /**
       @brief Check whether is griiping the target object.
       @param[in] body Set the target object's dBodyID.
       @retval true It has gripped target object.
       @retval false It has not gripped target object.
     */
    bool isGripping(dBodyID body) const;

    /**
       @brief To grip the target object.
       @param[in] worldID Set the dworldId
       @param[in] gripped Set the target object's dBodyID.
     */
    void grip(dWorldID worldId, dBodyID gripped);

    /**
       @brief Release the gripped object.
     */
    void release();

    /**
       @brief Check whether or not parallel the gripper surface and the object.
       @param[in] numContacts Number of contact objects.
       @param[in] contacts Instance of contact objects.
       @param[in] dotThreshold Set dot product threshold. This value use for checking parallel between objects.
       @param[in] distanceThreshold Threshold distance. This value use for checking distance between objects.
       @return A number, which is contanct to this gripper.
     */
    int checkContact(int numContacts, dContact* contacts, double dotThreshold, double distanceThreshold);

    /**
       @brief Check whether limit exceeded.
       @param[in] currentTime Current simulation time.
       @retval true Limit execeeded.
       @retval false Limit not execeeded.
     */
    bool limitCheck(double currentTime);

private:
    bool on_;

public:
    /// Setting of gripping position. (Coordinates relative from the vacuum gripper's link position)
    Vector3 position;
    /// Setting of normal of grip surface.
    Vector3 normal;
    /// Setting of maximum pull force. [N]
    double maxPullForce;
    /// Setting of maximum shear force. [N]
    double maxShearForce;
    /// Setting of maximum peel torque. [Nm]
    double maxPeelTorque;

    /// Vacuum gripper's dBodyID.
    dBodyID gripper;
    /// If the not equal zero, exists joint between objects. This means that it is during the adsorption.
    dJointID jointID;
};

typedef ref_ptr<VacuumGripper> VacuumGripperPtr;
}

#endif
