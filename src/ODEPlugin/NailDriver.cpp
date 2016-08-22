/*!
  Nail driver device.
  @file NailDriver.cpp
  @author
*/

#include <cnoid/NailDriver>
#include "NailedObjectManager.h"

#include <cnoid/Device>
#include <cnoid/EigenUtil>

#include <cnoid/ValueTree>

#include "gettext.h"

#include <cnoid/MessageView>
#include <boost/format.hpp>

#include <iostream>

using namespace cnoid;
using namespace std;

const char* NailDriver::typeName()
{
    return "NailDriver";
}

DeviceState* NailDriver::cloneState() const
{
    return new NailDriver(*this, true);
}

int NailDriver::stateSize() const
{
    return 1;
}

Device* NailDriver::clone() const
{
    return new NailDriver(*this);
}

NailDriver::NailDriver()
{
    on_ = false;
    contact_ = false;
    ready_ = false;
    position << 0, 0, 0;
    normal << 0, 0, 0;
    maxFasteningForce = std::numeric_limits<double>::max();

    not_called_count = 0;
    near_callback_called = false;

    resetLatestContact();
}

void NailDriver::copyStateFrom(const NailDriver& other)
{
    on_ = other.on_;
    not_called_count = other.not_called_count;
    near_callback_called = other.near_callback_called;
}

void NailDriver::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(NailDriver)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const NailDriver&>(other));
}

NailDriver::NailDriver(const NailDriver& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
    position = org.position;
    normal = org.normal;
    maxFasteningForce = org.maxFasteningForce;
}

const double* NailDriver::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}

double* NailDriver::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}

void NailDriver::on(bool on) {
    string msg = str(boost::format(_("%s: %s %s => %s")) % typeName() % link()->name() % (on_ ? "ON" : "OFF") % (on ? "ON" : "OFF"));
    MessageView::instance()->putln(msg);
    cout << msg << endl;

    if (on_ == false && on == true) {
        // By switching from off to on,
        // it becomes possible to injection of a nail.
        setReady();
    }

    on_ = on;
}

void NailDriver::setReady()
{
    string msg = str(boost::format(_("%s: %s Ready")) % typeName() % link()->name());
    MessageView::instance()->putln(msg);
    cout << msg << endl;

    ready_ = true;
}

void NailDriver::fire(NailedObject* nobj)
{
    string msg;
    msg = str(boost::format(_("%s: %s Fire")) % typeName() % link()->name());
    MessageView::instance()->putln(msg);
    cout << msg << endl;

    if (nobj->getNailCount() == 0) {
#ifdef NAILDRIVER_STATUS
        MessageView::instance()->putln("NailDriver: *** joint created **");
        cout << "NailDriver: *** joint created **" << endl;
#endif // NAILDRIVER_STATUS

        const Vector3 n = link()->R() * normal;
        nobj->setNailDirection(n);
    }

    nobj->addNail(maxFasteningForce);
#ifdef NAILDRIVER_STATUS
    msg = str(boost::format("NailDriver: nail count = %d") % nobj->getNailCount());
    MessageView::instance()->putln(msg);
    cout << msg << endl;
#endif // NAILDRIVER_STATUS
    ready_ = false;
}

void NailDriver::distantCheck(int distantCheckCount)
{
    if (!near_callback_called) {
        // Check number of times nearCallback() was not called continuously.
        // If more than distantCheckCount times, it is processing as a
        // distant from object.
        if (not_called_count < distantCheckCount) {
            not_called_count++;
        } else {
            if (on() && !ready()) {
                setReady();
            }
        }
    } else {
#ifdef NAILDRIVER_DEBUG
        if (not_called_count >= distantCheckCount) {
            string msg = str(boost::format("NailDriver: %d step") % not_called_count);
            MessageView::instance()->putln(msg);
            cout << msg << endl;
        }
#endif // NAILDRIVER_DEBUG
        // Since the nearCallback() was called, reset the counter.
        not_called_count = 0;
        // And reset the flag.
        near_callback_called = false;
    }
}

int NailDriver::checkContact(int numContacts, dContact* contacts, double dotThreshold, double distanceThreshold)
{
    Link* link_ = link();
    Vector3 muzzle = link_->p() + link_->R() * position;

    int n = 0;
    for (int i=0; i < numContacts; ++i) {
	Vector3 pos(contacts[i].geom.pos);
	Vector3 v(contacts[i].geom.normal);

	float isParallel = (link_->R() * normal).dot(v);

	// Distance gripper (P: muzzle) and contact (A:pos)
	Vector3 pa;
	pa[0] = pos[0] - muzzle[0];
	pa[1] = pos[1] - muzzle[1];
	pa[2] = pos[2] - muzzle[2];

	float distance = fabs(muzzle.dot(pa));
        if (isParallel < dotThreshold && distance < distanceThreshold) {
	    n++;
	}
    }
    return n;
}
