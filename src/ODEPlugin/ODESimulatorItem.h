/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_ODEPLUGIN_ODE_SIMULATOR_ITEM_H
#define CNOID_ODEPLUGIN_ODE_SIMULATOR_ITEM_H

#include <cnoid/SimulatorItem>
#include <cnoid/EigenTypes>
#include "exportdecl.h"
#ifdef GAZEBO_ODE
#define ODESimulatorItem GazeboODESimulatorItem
#endif

namespace cnoid {

class ODESimulatorItemImpl;
        
class CNOID_EXPORT ODESimulatorItem : public SimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    ODESimulatorItem();
    ODESimulatorItem(const ODESimulatorItem& org);
    virtual ~ODESimulatorItem();

    virtual void setAllLinkPositionOutputMode(bool on);

    enum StepMode { STEP_ITERATIVE, STEP_BIG_MATRIX, NUM_STEP_MODES };

    void setStepMode(int value);
    void setGravity(const Vector3& gravity);
    void setFriction(double friction);
    void setJointLimitMode(bool on);
    void set2Dmode(bool on);
    void setGlobalERP(double erp);
    void setGlobalCFM(double value);
    void setNumIterations(int n);
    void setOverRelaxation(double value);
    void setCorrectingVelocityLimitMode(bool on);
    void setMaxCorrectingVelocity(double vel);
    void setSurfaceLayerDepth(double value);
    void useWorldCollisionDetector(bool on);

#ifdef VACUUM_GRIPPER_ODE    /* VACUUM_GRIPPER_ODE */
    /**
       @brief Change state all of the vacuum gripper.
       @param[in] on If set true, enable the vacuum grippers. If set false, disable the vacuum grippers.
     */
    void useVacuumGripper(bool on);

    /**
       @brief Set vacuum gripper limit check start time.
       @param[in] limitCheckStartTime To set the time until the start of limit check of the restraint forces.
     */
    void setVacuumGripperLimitCheckStartTime(double limitCheckStartTime);

    /**
       @brief Set vacuum gripper dot product threshold.
       @param[in] threshold If the dot product of the grip surface and the object is less than this value,
       then gripping the object.
     */
    void setVacuumGripperDot(double threshold);

    /**
       @brief Set distance check threshold for between the vacuum gripper to the target object.
       @param[in] threshold If the distance of the grip surface and the object is less than this value,
       then gripping the object.
     */
    void setVacuumGripperDistance(double threshold);
#endif                       /* VACUUM_GRIPPER_ODE */

#ifdef NAIL_DRIVER_ODE    /* NAIL_DRIVER_ODE */
    /**
       @brief Change state all of the nail driver.
       @param[in] on If set true, enable the nail drivers. If set false, disable the nail drivers.
     */
    void useNailDriver(bool on);

    /**
       @brief Set nail driver limit check start time.
       @param[in] limitCheckStartTime To set the time until the start of limit check of the restraint forces.
     */
    void setNailDriverLimitCheckStartTime(double limitCheckStartTime);

    /**
       @brief Set distance check count for between the nail driver to the target object.
       Until more than the specified check count value will be treated as being in contact with each other.
       @param[in] distantCheckCount Set the check count value.
     */
    void setNailDriverDistantCheckCount(int distantCheckCount);

    /**
       @brief Set nail driver dot product threshold.
       @param[in] threshold If the dot product of the grip surface and the object is less than this value,
       then gripping the object.
     */
    void setNailDriverDot(double threshold);

    /**
       @brief Set distance check threshold for between the nail driver to the target object.
       @param[in] threshold If the distance of the grip surface and the object is less than this value,
       then gripping the object.
     */
    void setNailDriverDistance(double threshold);
#endif                    /* NAIL_DRIVER_ODE */

protected:
        
    virtual SimulationBody* createSimulationBody(Body* orgBody);
    virtual bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    virtual void initializeSimulationThread();
    virtual bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    virtual void finalizeSimulation();

    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    ODESimulatorItemImpl* impl;
    friend class ODESimulatorItemImpl;
};

typedef ref_ptr<ODESimulatorItem> ODESimulatorItemPtr;
}

#endif
