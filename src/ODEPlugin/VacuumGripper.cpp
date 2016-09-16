/*!
  Vacuum gripper device.
  @file VacuumGripper.cpp
  @author
*/

#include <cnoid/VacuumGripper>

#include <cnoid/Device>
#include <cnoid/EigenUtil>

#include <cnoid/ValueTree>

#include "gettext.h"

#include <cnoid/MessageView>
#include <boost/format.hpp>

#include <iostream>

using namespace cnoid;
using namespace std;

const char* VacuumGripper::typeName()
{
    return "VacuumGripper";
}

DeviceState* VacuumGripper::cloneState() const
{
    return new VacuumGripper(*this, true);
}

int VacuumGripper::stateSize() const
{
    return 1;
}

Device* VacuumGripper::clone() const
{
    return new VacuumGripper(*this);
}

VacuumGripper::VacuumGripper()
{
    on_ = false;
    jointID = 0;
    position << 0, 0, 0;
    normal << 0, 0, 0;
    maxPullForce = std::numeric_limits<double>::max();
    maxShearForce = std::numeric_limits<double>::max();
    maxPeelTorque = std::numeric_limits<double>::max();
}

void VacuumGripper::copyStateFrom(const VacuumGripper& other)
{
    on_ = other.on_;
}

void VacuumGripper::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(VacuumGripper)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const VacuumGripper&>(other));
}

VacuumGripper::VacuumGripper(const VacuumGripper& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
    jointID = org.jointID;
    position = org.position;
    normal = org.normal;
    maxPullForce = org.maxPullForce;
    maxShearForce = org.maxShearForce;
    maxPeelTorque = org.maxPeelTorque;
}

const double* VacuumGripper::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}

double* VacuumGripper::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}

void VacuumGripper::on(bool on) {
    string msg = str(
        boost::format(_("%s: %s %s => %s")) % typeName() % link()->name() % (on_ ? "ON" : "OFF") % (on ? "ON" : "OFF")
        );
    MessageView::instance()->putln(msg);
#ifdef VACUUM_GRIPPER_DEBUG    /* VACUUM_GRIPPER_DEBUG */
    cout << msg << endl;
#endif                         /* VACUUM_GRIPPER_DEBUG */
    on_ = on;
}

void VacuumGripper::grip(dWorldID worldID, dBodyID gripped)
{
    jointID = dJointCreateFixed(worldID, 0);
    dJointAttach(jointID, gripped, gripper);
    dJointSetFixed(jointID);
    dJointSetFeedback(jointID, new dJointFeedback());
#ifdef VACUUM_GRIPPER_STATUS
    MessageView::instance()->putln("VacuumGripper: *** joint created **");
    cout << "VacuumGripper: *** joint created **" << endl;
#endif // VACUUM_GRIPPER_STATUS
}

bool VacuumGripper::isGripping(dBodyID object) const
{
    return dAreConnected(gripper, object);
}

void VacuumGripper::release()
{
    if (!isGripping()) return;

    dJointSetFeedback(jointID, 0);
    dJointDestroy(jointID);
    jointID = 0;
#ifdef VACUUM_GRIPPER_STATUS
    MessageView::instance()->putln("VacuumGripper: *** joint destroy ***");
    cout << "VacuumGripper: *** joint destroy **" << endl;
#endif // VACUUM_GRIPPER_STATUS
}

int VacuumGripper::checkContact(int numContacts, dContact* contacts, double dotThreshold, double distanceThreshold)
{
    Vector3 vacuumPos = link()->p() + link()->R() * position;

    int n = 0;
    for(int i=0; i < numContacts; ++i){
	Vector3 pos(contacts[i].geom.pos);
	Vector3 v(contacts[i].geom.normal);

	float isParallel = (link()->R() * normal).dot(v);

	// Distance gripper (P: vacuumPos) and contact (A:pos)
	Vector3 pa;
	pa[0] = pos[0] - vacuumPos[0];
	pa[1] = pos[1] - vacuumPos[1];
	pa[2] = pos[2] - vacuumPos[2];

	float distance = abs(vacuumPos.dot(pa));
        if (isParallel < dotThreshold && distance < distanceThreshold) {
	    n++;
	}
    }
    return n;
}

bool VacuumGripper::limitCheck(double currentTime)
{
    dJointFeedback* fb = dJointGetFeedback(jointID);

    Vector3 f(fb->f2);
    Vector3 tau(fb->t2);

    const Vector3 n = link()->R() * normal;
    const Vector3 p = link()->R() * position + link()->p();
    const Vector3 ttt = tau - p.cross(f);

    double pullForce = n.dot(f);

    double fx = f[0] * f[0];
    double fy = f[1] * f[1];
    double shearForce = sqrt(fx + fy);

    double peelTorque = fabs(n.dot(ttt));

#ifdef VACUUM_GRIPPER_DEBUG
    cout << currentTime
         << " : vacuum : " << pullForce
         << " " << shearForce
         << " " << peelTorque
         << endl;
#endif // VACUUM_GRIPPER_DEBUG

    if (pullForce + (dReal)maxPullForce < 0) {
        string msg = str(boost::format("PullForce limit exceeded: %f > %f") % fabs(pullForce) % maxPullForce);
        MessageView::instance()->putln(msg);
#ifdef VACUUM_GRIPPER_DEBUG    /* VACUUM_GRIPPER_DEBUG */
        cout << msg << endl;
#endif                         /* VACUUM_GRIPPER_DEBUG */
        return true;
    }

    if (shearForce > (dReal)maxShearForce) {
        string msg = str(boost::format("ShearForce limit exceeded: %f > %f") % shearForce % maxShearForce);
        MessageView::instance()->putln(msg);
#ifdef VACUUM_GRIPPER_DEBUG    /* VACUUM_GRIPPER_DEBUG */
        cout << msg << endl;
#endif                         /* VACUUM_GRIPPER_DEBUG */
        return true;
    }

    if (peelTorque > (dReal)maxPeelTorque) {
        string msg = str(boost::format("PeelTorque limit exceeded: %f > %f") % peelTorque % maxPeelTorque);
        MessageView::instance()->putln(msg);
#ifdef VACUUM_GRIPPER_DEBUG    /* VACUUM_GRIPPER_DEBUG */
        cout << msg << endl;
#endif                         /* VACUUM_GRIPPER_DEBUG */
        return true;
    }

    return false;
}
