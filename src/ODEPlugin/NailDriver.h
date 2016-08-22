/*!
  Nail driver device.
  @file NailDriver.h
  @author
*/

#ifndef CNOID_ODEPLUGIN_NAILDRIVER_H
#define CNOID_ODEPLUGIN_NAILDRIVER_H

#include <cnoid/EigenTypes>
#include <cnoid/Body>
#include <cnoid/Link>
#include <string>
#include "exportdecl.h"

#include <ode/ode.h>

namespace cnoid {
class NailedObject;

/**
   @brief Nail driver device.
 */
class CNOID_EXPORT NailDriver : public Device
{
public:
    /**
       @brief Constructor.
     */
    NailDriver();

    /**
       @brief Copy constructor.
     */
    NailDriver(const NailDriver& org, bool copyStateOnly = false);

public:
    virtual const char* typeName();

    virtual void copyStateFrom(const DeviceState& other);
    virtual void copyStateFrom(const NailDriver& other);
    virtual DeviceState* cloneState() const;

    virtual int stateSize() const;

    virtual Device* clone() const;

    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    /**
       @brief Get state of this device.
       @retval true This device enabled.
       @retval false This device disabled.
     */
    bool on() const { return on_; }

    /**
       @brief Change state of this device.
       @param[in] on If set true, enable this device. If set false, disable this device.
     */
    void on(bool on);

    /**
       @brief Check whether or not parallel the muzzle and the object.
       @param[in] numContacts Number of conatacts.
       @param[in] dContact Contacts object pointer of dContact.
       @param[in] dotThreshold Set dot product threshold. This value use for checking parallel between objects.
       @param[in] distanceThreshold Set distance threshold. This value use for checking distance between objects.
       @return A number, which is contanct to this nail driver.
     */
    int checkContact(int numContacts, dContact* contacts, double dotThreshold, double distanceThreshold);

    /**
       @brief To inform that the contact has occured.
     */
    void contact() { near_callback_called = true; }

    /**
       @brief Checking whether the away from object.
       If more than the 'distantCheckCount' times, it is a distant from object.
       @param[in] distantCheckCount Set the distant check count.
     */
    void distantCheck(int distantCheckCount);

    /**
       @brief The nail driver to check whether firing.
       @retval true Can fire.
       @retval false Can not fire.
     */
    bool ready() const { return on_ && ready_; }

    /**
       @brief Be able to firing the nail driver.
     */
    void setReady();

    /**
       @brief Firing nail to the target object.
       @param[in] nobj Target object's pointer of NaildedObject.
     */
    void fire(NailedObject* nobj);

    /**
       @brief Set the simulation time of the latest contact to object.
       @param[in] current Set the current simulation time.
       The current simulation time, getting by cnoid::SimulatorItem.currentTime().
     */
    void setLatestContact(const double current) { latestContact = current; }

    /**
       @brief Getting the simulation time of the latest contact to object.
       @return The simulation time of the latest contact to object.
     */
    double getLatestContact() { return latestContact; }

    /**
       @brief Reset the simulation time of the latest contact to object.
     */
    void resetLatestContact() { latestContact = 0.0; }

private:
    bool on_;
    bool contact_;
    bool ready_;
    int not_called_count;
    bool near_callback_called;

public:
    /// Setting of firing position. (Coordinates relative from the vacuum gripper's link position)
    Vector3 position;
    /// Setting of normal of firing surface.
    Vector3 normal;
    /// The force of direction of a nail fastening two objects.
    double maxFasteningForce;
    /// The simulation time of the latest contact to object.
    double latestContact;
};

typedef ref_ptr<NailDriver> NailDriverPtr;
}

#endif
