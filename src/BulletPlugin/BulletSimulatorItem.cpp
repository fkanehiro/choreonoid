/*!
  @file
  @author Shizuko Hattori
*/

#include "BulletSimulatorItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/Body>
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/ColdetModel>
#include <cnoid/FloatingNumberString>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyCollisionDetectorUtil>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <btBulletDynamicsCommon.h>
#include <HACD/hacdHACD.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

#define DEBUG_OUT 0

namespace {

const btScalar DEFAULT_GRAVITY_ACCELERATION = 9.80665;
const btScalar DEFAULT_ERP = 0.2;
const int  DEFAULT_NUMITERATION = 10;
const btScalar DEFAULT_RESTITUTION = 0.0;
const btScalar DEFAULT_FRICTION = 0.7;
const btScalar DEFAULT_ERP2 = 0.0;
const btScalar DEFAULT_SPLITIMPULSEPENETRATIONTHRESHOLD = -0.0001;
const btScalar DEFAULT_COLLISION_MARGIN = 0.0001;
const bool meshOnly = false;             // not use primitive Shape
const bool mixedPrimitiveMesh = true;   // mixed of Primitive and Mesh on one link

class BulletBody;

class BulletLink : public Referenced
{
public:
    BulletLink* parent;
    Link* link;
    btTransform shift;
    btTransform invShift;
    vector<btScalar> vertices;
    vector<int> triangles;

    btTriangleIndexVertexArray* pMeshData;
    btTriangleMesh* trimesh;
    btCollisionShape* collisionShape;

    btDefaultMotionState* motionState;
    btRigidBody* body;
    btTypedConstraint* joint;
    btDynamicsWorld* dynamicsWorld;
    double q_offset;
    BulletSimulatorItemImpl* simImpl;
    bool isStatic;
        
    BulletLink(BulletSimulatorItemImpl* simImpl, BulletBody* bulletBody, BulletLink* parent,
               const Vector3& parentOrigin, Link* link, short group, bool isSelfCollisionDetectionEnabled);
    ~BulletLink();
    void createLinkBody(BulletSimulatorItemImpl* simImpl, BulletLink* parent, const Vector3& origin,
                        short group, bool isSelfCollisionDetectionEnabled);
    void addMesh(MeshExtractor* extractor, bool meshOnly);
    void createGeometry();
    void getKinematicStateFromBullet();
    void setKinematicStateToBullet();
    void setTorqueToBullet();
    void setVelocityToBullet();
};
typedef ref_ptr<BulletLink> BulletLinkPtr;

class BulletBody : public SimulationBody
{
public:
    vector<BulletLinkPtr> bulletLinks;
    vector<btTypedConstraint*> extraJoints;
    btDynamicsWorld* dynamicsWorld;
    vector<btJointFeedback*> forceSensorFeedbacks;
    BasicSensorSimulationHelper sensorHelper;
    int geometryId;

    BodyPtr body;
    BulletBody(const Body& orgBody);
    ~BulletBody();
    void createBody(BulletSimulatorItemImpl* simImpl, short group);
    void getKinematicStateFromBullet();
    void setKinematicStateToBullet();
    void setTorqueToBullet();
    void setExtraJoints();
    void updateForceSensors();
    void setVelocityToBullet();
};
}


namespace cnoid {
  
class BulletSimulatorItemImpl
{
public:
    BulletSimulatorItem* self;
        
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* broadphase;
    btConstraintSolver* solver;
    btDynamicsWorld* dynamicsWorld;

    Vector3 gravity;
    double timeStep;
    double erp;
    int numIterations;
    double restitution;
    double friction;
    double erp2;
    double splitImpulsePenetrationThreshold;
    bool useWorldCollision;
    bool useHACD;                           //  Hierarchical Approximate Convex Decomposition
    double collisionMargin;
    CollisionDetectorPtr collisionDetector;
    vector<BulletLink*> geometryIdToLink;
    bool velocityMode;

    BulletSimulatorItemImpl(BulletSimulatorItem* self);
    BulletSimulatorItemImpl(BulletSimulatorItem* self, const BulletSimulatorItemImpl& org);
    ~BulletSimulatorItemImpl();
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
    void initialize();
    void clear();
    void addBody(BulletBody* bulletBody, short group);
    void setSolverParameter();
};
}


static bool CustomMaterialCombinerCallback(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0,int partId0,int index0,const btCollisionObjectWrapper* colObj1,int partId1,int index1)
{
    if(cp.m_distance1 < -0.001)
        return true;
    BulletLink* bulletLink0 = (BulletLink*)colObj0->getCollisionObject()->getUserPointer();
    Link* link0 = 0;
    if(bulletLink0)
        link0 = bulletLink0->link;
    BulletLink* bulletLink1 = (BulletLink*)colObj1->getCollisionObject()->getUserPointer();
    Link* link1 = 0;
    if(bulletLink1)
        link1 = bulletLink1->link;

    Link* crawlerlink;
    double sign = 1;
    double friction;
    if(link0 && ((link0->jointType() == Link::CRAWLER_JOINT) || (link0->jointType() == Link::PSEUDO_CONTINUOUS_TRACK))){
        crawlerlink = link0;
        friction = bulletLink0->simImpl->friction;
    } else if(link1 && ((link1->jointType() == Link::CRAWLER_JOINT) || (link1->jointType() == Link::PSEUDO_CONTINUOUS_TRACK))) {
        crawlerlink = link1;
        sign = -1;
        friction = bulletLink1->simImpl->friction;
    }else
        return true;

    Vector3 axis = crawlerlink->R() * crawlerlink->a();
    btVector3 normal = cp.m_normalWorldOnB;
    Vector3 n(normal.x(), normal.y(), normal.z());  
    Vector3 dir = axis.cross(n);

    if(dir.norm()<1e-5)
        return true;
    
    dir *= sign;
    dir.normalize();
    Vector3 frictionDir1 = friction * dir;
    Vector3 frictionDir2 = 0.1 * axis;
    
    cp.m_lateralFrictionInitialized = true;
    cp.m_lateralFrictionDir1.setValue(frictionDir1.x(), frictionDir1.y(), frictionDir1.z());
    cp.m_lateralFrictionDir2.setValue(frictionDir2.x(), frictionDir2.y(), frictionDir2.z());
    if(crawlerlink->jointType() == Link::PSEUDO_CONTINUOUS_TRACK)
        cp.m_contactMotion1 = crawlerlink->dq();
    else
        cp.m_contactMotion1 = crawlerlink->u();

    return true;
}


///////////////////////////////////Link////////////////////////////////////////////////////
BulletLink::BulletLink(BulletSimulatorItemImpl* _simImpl, BulletBody* bulletBody, BulletLink* parent,
                       const Vector3& parentOrigin, Link* link, short group, bool isSelfCollisionDetectionEnabled)
{
    bulletBody->bulletLinks.push_back(this);
    dynamicsWorld = _simImpl->dynamicsWorld;
    this->simImpl = _simImpl;

    this->link = link;
    vertices.clear();
    triangles.clear();
    pMeshData = 0;
    trimesh = 0;
    collisionShape = 0;
    motionState = 0;
    body = 0;
    joint=0;
    q_offset = 0;
    isStatic = false;
    
    Vector3 o = parentOrigin + link->b();
    
    createLinkBody(simImpl, parent, o, group, isSelfCollisionDetectionEnabled);
    body->setActivationState(DISABLE_DEACTIVATION);

    for(Link* child = link->child(); child; child = child->sibling()){
        new BulletLink(simImpl, bulletBody, this, o, child, group, isSelfCollisionDetectionEnabled);
    }
}

void BulletLink::createGeometry()
{
    if(link->shape()){
        MeshExtractor* extractor = new MeshExtractor;
        if(extractor->extract(link->shape(), boost::bind(&BulletLink::addMesh, this, extractor, meshOnly))){
            if(!simImpl->useHACD){
                if(!mixedPrimitiveMesh){
                    if(!vertices.empty()){
                        btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(collisionShape);
                        if(compoundShape){
                            int num = compoundShape->getNumChildShapes();
                            for(int i=0; i<num; i++){
                                delete compoundShape->getChildShape(i);
                            }
                            delete compoundShape;
                            collisionShape = 0;
                            vertices.clear();
                            triangles.clear();
                            extractor->extract(link->shape(), boost::bind(&BulletLink::addMesh, this, boost::ref(extractor), true));
                        }
                    }
                }
                if(!vertices.empty()){
                    pMeshData = new btTriangleIndexVertexArray(triangles.size()/3, &triangles[0], sizeof(int)*3, 
                                                               vertices.size()/3, &vertices[0], sizeof(btScalar)*3);
                    btGImpactMeshShape* meshShape = new btGImpactMeshShape(pMeshData);
                    meshShape->setLocalScaling(btVector3(1.f,1.f,1.f));
                    meshShape->setMargin(simImpl->collisionMargin);
                    meshShape->updateBound();
                    btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(collisionShape);
                    if(compoundShape){
                        btTransform T;
                        T.setIdentity();
                        compoundShape->addChildShape(T, meshShape);
                    }else
                        collisionShape = meshShape;
                }
            }else{
                if(!vertices.empty()){
                    trimesh = new btTriangleMesh();
                    for (size_t i=0; i<triangles.size()/3; i++){
                        int index0 = triangles[i*3];
                        int index1 = triangles[i*3+1];
                        int index2 = triangles[i*3+2];
                        btVector3 vertex0(vertices[index0*3], vertices[index0*3+1], vertices[index0*3+2]);
                        btVector3 vertex1(vertices[index1*3], vertices[index1*3+1], vertices[index1*3+2]);
                        btVector3 vertex2(vertices[index2*3], vertices[index2*3+1], vertices[index2*3+2]);
                        trimesh->addTriangle(vertex0,vertex1,vertex2);
                    }
                    btBvhTriangleMeshShape* concaveShape = new btBvhTriangleMeshShape(trimesh, true);
                    concaveShape->setMargin(simImpl->collisionMargin);
                    btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(collisionShape);
                    if(compoundShape){
                        btTransform T;
                        T.setIdentity();
                        compoundShape->addChildShape(T, concaveShape);
                    }else
                        collisionShape = concaveShape;
                }
            }
        }
        delete extractor;
    }
}


void BulletLink::addMesh(MeshExtractor* extractor, bool meshOnly)
{
    SgMesh* mesh = extractor->currentMesh();
    const Affine3& T = extractor->currentTransform();

    bool meshAdded = false;
    
    if(!meshOnly){
        if(mesh->primitiveType() != SgMesh::MESH){
            bool doAddPrimitive = false;
            Vector3 scale;
            optional<Vector3> translation;
            if(!extractor->isCurrentScaled()){
                scale.setOnes();
                doAddPrimitive = true;
            } else {
                Affine3 S = extractor->currentTransformWithoutScaling().inverse() *
                    extractor->currentTransform();

                if(S.linear().isDiagonal()){
                    if(!S.translation().isZero()){
                        translation = S.translation();
                    }
                    scale = S.linear().diagonal();
                    if(mesh->primitiveType() == SgMesh::BOX){
                        doAddPrimitive = true;
                    } else if(mesh->primitiveType() == SgMesh::SPHERE){
                        // check if the sphere is uniformly scaled for all the axes
                        if(scale.x() == scale.y() && scale.x() == scale.z()){
                            doAddPrimitive = true;
                        }
                    } else if(mesh->primitiveType() == SgMesh::CYLINDER){
                        // check if the bottom circle face is uniformly scaled
                        if(scale.x() == scale.z()){
                            doAddPrimitive = true;
                        }
                    } else if(mesh->primitiveType() == SgMesh::CONE){
                        if(scale.x() == scale.z()){
                            doAddPrimitive = true;
                        }
                    }
                }
            }
            if(doAddPrimitive){
                bool created = false;

                btCollisionShape* primitiveShape;
                switch(mesh->primitiveType()){
                case SgMesh::BOX : {
                    const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                    primitiveShape = new btBoxShape(btVector3(s.x() * scale.x()/2.0, s.y() * scale.y()/2.0, s.z() * scale.z()/2.0));
                    created = true;
                    break; }
                case SgMesh::SPHERE : {
                    SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                    primitiveShape = new btSphereShape(sphere.radius * scale.x());
                    created = true;
                    break; }
                case SgMesh::CYLINDER : {
                    SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                    primitiveShape = new btCylinderShape(btVector3(cylinder.radius * scale.x(), cylinder.height * scale.y()/2.0, cylinder.radius * scale.x()));
                    created = true;
                    break; }
                case SgMesh::CONE : {
                    SgMesh::Cone cone = mesh->primitive<SgMesh::Cone>();
                    primitiveShape = new btConeShape(cone.radius * scale.x(), cone.height * scale.y());
                    created = true;
                    break; }
                default :
                    break;
                }
                if(created){
                    primitiveShape->setMargin(simImpl->collisionMargin);
                    btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(collisionShape);
                    if(!compoundShape){
                        collisionShape = new btCompoundShape();
                        collisionShape->setLocalScaling(btVector3(1.f,1.f,1.f));
                        compoundShape = dynamic_cast<btCompoundShape*>(collisionShape);
                    }
                    Affine3 T_ = extractor->currentTransformWithoutScaling();
                    if(translation){
                        T_ *= Translation3(*translation);
                    }
                    btVector3 p(T_(0,3), T_(1,3), T_(2,3));
                    btMatrix3x3 R(T_(0,0), T_(0,1), T_(0,2),
                                  T_(1,0), T_(1,1), T_(1,2),
                                  T_(2,0), T_(2,1), T_(2,2));
                    btTransform btT(R, p);
                    compoundShape->addChildShape(invShift * btT, primitiveShape);
                    meshAdded = true;
                }
            }
        }
    }
    if(!meshAdded){
        if(!simImpl->useHACD || isStatic){
            const int vertexIndexTop = vertices.size() / 3;

            const SgVertexArray& vertices_ = *mesh->vertices();
            const int numVertices = vertices_.size();
            for(int i=0; i < numVertices; ++i){
                const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
                btVector3 v0 = invShift * btVector3(v.x(), v.y(), v.z());
                vertices.push_back(v0.x());
                vertices.push_back(v0.y());
                vertices.push_back(v0.z());
            }

            const int numTriangles = mesh->numTriangles();
            for(int i=0; i < numTriangles; ++i){
                SgMesh::TriangleRef tri = mesh->triangle(i);
                triangles.push_back(vertexIndexTop + tri[0]);
                triangles.push_back(vertexIndexTop + tri[1]);
                triangles.push_back(vertexIndexTop + tri[2]);
            }
        }else{
            btConvexHullShape* convexHullShape = dynamic_cast<btConvexHullShape*>(collisionShape);
            if(convexHullShape){
                btCompoundShape* compoundShape = new btCompoundShape();
                compoundShape->setLocalScaling(btVector3(1.f,1.f,1.f));

                btTransform T;
                T.setIdentity();
                compoundShape->addChildShape(T, convexHullShape);
                collisionShape = compoundShape;
            }

            std::vector< HACD::Vec3<HACD::Real> > points;
            std::vector< HACD::Vec3<long> > triangles;

            const SgVertexArray& vertices_ = *mesh->vertices();
            const int numVertices = vertices_.size();
            for(int i=0; i < numVertices; ++i){
                const Vector3 v = T * vertices_[i].cast<Position::Scalar>();
                btVector3 v0 = invShift * btVector3(v.x(), v.y(), v.z());
                HACD::Vec3<HACD::Real> vertex(v0.x(), v0.y(), v0.z());
                points.push_back(vertex);
            }

            const int numTriangles = mesh->numTriangles();
            for(int i=0; i < numTriangles; ++i){
                SgMesh::TriangleRef tri = mesh->triangle(i);
                HACD::Vec3<long> triangle(tri[0], tri[1], tri[2]);
                triangles.push_back(triangle);
            }

            HACD::HACD hacd;
            hacd.SetPoints(&points[0]);
            hacd.SetNPoints(points.size());
            hacd.SetTriangles(&triangles[0]);
            hacd.SetNTriangles(triangles.size());
            hacd.SetCompacityWeight(0.1);
            hacd.SetVolumeWeight(0.0);

            size_t nClusters = 1;
            double concavity = 100;
            bool invert = false;
            bool addExtraDistPoints = false;
            bool addNeighboursDistPoints = false;
            bool addFacesPoints = false;       

            hacd.SetNClusters(nClusters);                     // minimum number of clusters
            hacd.SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
            hacd.SetConcavity(concavity);                     // maximum concavity
            hacd.SetAddExtraDistPoints(addExtraDistPoints);   
            hacd.SetAddNeighboursDistPoints(addNeighboursDistPoints);   
            hacd.SetAddFacesPoints(addFacesPoints); 
            hacd.Compute();

            btTransform T;
            T.setIdentity();

            nClusters = hacd.GetNClusters();
            if(nClusters>1){
                btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(collisionShape);
                if(!compoundShape){
                    collisionShape = new btCompoundShape();
                    collisionShape->setLocalScaling(btVector3(1.f,1.f,1.f));
                }
            }

            for(size_t i=0; i<nClusters; i++){
                size_t nPoints = hacd.GetNPointsCH(i);
                size_t nTriangles = hacd.GetNTrianglesCH(i);

                HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
                HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
                hacd.GetCH(i, pointsCH, trianglesCH);
                
                btAlignedObjectArray<btVector3> newVertices_;
                for(size_t j=0; j<nTriangles; j++){
                    long index0 = trianglesCH[j].X();
                    long index1 = trianglesCH[j].Y();
                    long index2 = trianglesCH[j].Z();
                    btVector3 vertex0(pointsCH[index0].X(), pointsCH[index0].Y(), pointsCH[index0].Z());
                    btVector3 vertex1(pointsCH[index1].X(), pointsCH[index1].Y(), pointsCH[index1].Z());
                    btVector3 vertex2(pointsCH[index2].X(), pointsCH[index2].Y(), pointsCH[index2].Z());
                    newVertices_.push_back(vertex0);
                    newVertices_.push_back(vertex1);
                    newVertices_.push_back(vertex2);
                }
                delete [] pointsCH;
                delete [] trianglesCH;

                /*
                  float collisionMargin = 0.01f;
                  btAlignedObjectArray<btVector3> planeEquations;
                  btGeometryUtil::getPlaneEquationsFromVertices(newVertices_, planeEquations);

                  btAlignedObjectArray<btVector3> shiftedPlaneEquations;
                  for (int j=0; j<planeEquations.size(); j++){
                  btVector3 plane = planeEquations[j];
                  plane[3] += collisionMargin;
                  shiftedPlaneEquations.push_back(plane);
                  }
                  btAlignedObjectArray<btVector3> shiftedVertices;
                  btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,shiftedVertices);
                
                  btConvexHullShape* convexHullShape_ = new btConvexHullShape(&(shiftedVertices[0].getX()),shiftedVertices.size());
                */
                btConvexHullShape* convexHullShape_ = new btConvexHullShape(&(newVertices_[0].getX()), newVertices_.size());
                convexHullShape_->setMargin(simImpl->collisionMargin);

                btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(collisionShape);
                if(compoundShape)
                    compoundShape->addChildShape(T, convexHullShape_);
                else
                    collisionShape = convexHullShape_;
            }
        }
    }
}

void BulletLink::createLinkBody(BulletSimulatorItemImpl* simImpl, BulletLink* parent_, const Vector3& origin, 
                                short group, bool isSelfCollisionDetectionEnabled)
{
    parent = parent_;
    const Vector3& c = link->c();
    btTransform transform;
    transform.setBasis(btMatrix3x3::getIdentity());
    transform.setOrigin(btVector3(origin(0), origin(1), origin(2)));

    btScalar mass(link->mass());
    if(link->isRoot() && link->isFixedJoint()){
        isStatic = true;
        mass = 0.0;
    }
    const Matrix3& I = link->I();

    btVector3 localInertia(0,0,0);
    shift.setIdentity();
    shift.setOrigin(btVector3(c.x(), c.y(), c.z()));
    if (mass != 0.0){
        if(!I.isDiagonal()){
            btMatrix3x3 tensor(I(0,0), I(0,1), I(0,2),
                               I(1,0), I(1,1), I(1,2),
                               I(2,0), I(2,1), I(2,2));
            tensor.diagonalize(shift.getBasis(), btScalar(0.00001), 20);
            localInertia.setValue(tensor[0][0], tensor[1][1], tensor[2][2]);

        }else{
            localInertia.setValue(I(0,0), I(1,1), I(2,2));
        }
    }
    invShift = shift.inverse();

#if DEBUG_OUT
    std::cout <<  shift.getBasis()[0][0] << " " << shift.getBasis()[0][1]<< " " << shift.getBasis()[0][2] << " " << shift.getOrigin()[0] << std::endl;
    std::cout <<  shift.getBasis()[1][0] << " " << shift.getBasis()[1][1]<< " " << shift.getBasis()[1][2] << " " << shift.getOrigin()[1] <<std::endl;
    std::cout <<  shift.getBasis()[2][0] << " " << shift.getBasis()[2][1]<< " " << shift.getBasis()[2][2] << " " << shift.getOrigin()[2] <<std::endl;
#endif

    if(!simImpl->useWorldCollision)
        createGeometry();
    motionState = new btDefaultMotionState(transform * shift);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,motionState,collisionShape,localInertia);
    body = new btRigidBody(rbInfo);
    //body->setDamping(0.1,0.1);
    body->setRestitution(simImpl->restitution);
    body->setFriction(simImpl->friction);
    body->setContactProcessingThreshold(BT_LARGE_FLOAT);
    body->forceActivationState(DISABLE_DEACTIVATION);
    
    short mask = 0xFFFF;
    if(!isSelfCollisionDetectionEnabled)
        mask ^= group;
    simImpl->dynamicsWorld->addRigidBody(body, group, mask); 

    const Vector3& a = link->a();
    const Vector3& b = link->b();
    const Vector3& d = link->d();

    switch(link->jointType()){
        
    case Link::ROTATIONAL_JOINT: {
        btVector3 axisA(a(0), a(1), a(2));
        btVector3 axisB(a(0), a(1), a(2)); 
        btVector3 pivotA(b(0), b(1), b(2));
        btVector3 pivotB(0,0,0);
        joint = new btHingeConstraint(*body, *(parent->body), invShift*pivotB, parent->invShift*pivotA,
                                      invShift.getBasis()*axisB, parent->invShift.getBasis()*axisA );

        if(link->q_upper() < numeric_limits<double>::max() && link->q_lower() > -numeric_limits<double>::max()){
            ((btHingeConstraint*)joint)->setLimit(link->q_lower(), link->q_upper());
        } else {
            //Lowerlimit ＞ Upperlimit → axis is free
            ((btHingeConstraint*)joint)->setLimit(1.0, -1.0); 
        }
        simImpl->dynamicsWorld->addConstraint(joint, true);
        q_offset = ((btHingeConstraint*)joint)->getHingeAngle();

#if DEBUG_OUT
        std::cout << "offset= " << q_offset << std::endl;
#endif
        break;
    }
        
    case Link::SLIDE_JOINT: {
        Vector3 u(0,0,1);
        Vector3 ty = d.cross(u);
        btMatrix3x3 btR;
        Matrix3 R0;
        if(ty.norm() == 0){
            btR.setIdentity();
            R0.setIdentity();
        } else {
            ty.normalized();
            Vector3 tx = ty.cross(d).normalized();
            R0.col(0) = tx;
            R0.col(1) = ty;
            R0.col(2) = d;
            btR.setValue(R0(0,0), R0(0,1), R0(0,2),
                         R0(1,0), R0(1,1), R0(1,2),
                         R0(2,0), R0(2,1), R0(2,2));
        }
        btTransform frameA,frameB;
        frameB.setBasis(btR);
        frameB.setOrigin(btVector3(0,0,0));
        frameA.setBasis(btR);
        frameA.setOrigin(btVector3(b(0), b(1), b(2)));
        joint = new btGeneric6DofConstraint(*(parent->body), *body, parent->invShift*frameA, invShift*frameB, false);

        if(link->q_upper() < numeric_limits<double>::max() &&
           link->q_lower() > -numeric_limits<double>::max() &&
           link->q_lower() < link->q_upper()){
            ((btGeneric6DofConstraint*)joint)->setLinearLowerLimit(btVector3(0.0, 0.0, link->q_lower()));
            ((btGeneric6DofConstraint*)joint)->setLinearUpperLimit(btVector3(0.0, 0.0, link->q_upper()));
            ((btGeneric6DofConstraint*)joint)->setAngularLowerLimit(btVector3(0.0, 0.0, 0.0));
            ((btGeneric6DofConstraint*)joint)->setAngularUpperLimit(btVector3(0.0, 0.0, 0.0));
        } else {
            //Lowerlimit ＞ Upperlimit → axis is free
            ((btGeneric6DofConstraint*)joint)->setLinearLowerLimit(btVector3(0.0, 0.0, 1.0)); 
            ((btGeneric6DofConstraint*)joint)->setLinearUpperLimit(btVector3(0.0, 0.0, -1.0));
            ((btGeneric6DofConstraint*)joint)->setAngularLowerLimit(btVector3(0.0, 0.0, 0.0));
            ((btGeneric6DofConstraint*)joint)->setAngularUpperLimit(btVector3(0.0, 0.0, 0.0));
        }
        ((btGeneric6DofConstraint*)joint)->calculateTransforms();
        simImpl->dynamicsWorld->addConstraint(joint, true);
        q_offset = ((btGeneric6DofConstraint*)joint)->getRelativePivotPosition(2);

        break;
    }

    case Link::FREE_JOINT:
        break;
        
    case Link::FIXED_JOINT:
    default:
        if(!link->isRoot()){
            btTransform frameA,frameB;
            frameA.setIdentity();
            frameA.setOrigin(btVector3(b(0), b(1), b(2)));
            frameB.setIdentity();
            joint = new btGeneric6DofConstraint(*(parent->body), *body, parent->invShift*frameA, invShift*frameB, false);
            ((btGeneric6DofConstraint*)joint)->calculateTransforms();
            simImpl->dynamicsWorld->addConstraint(joint, true);
            ((btGeneric6DofConstraint*)joint)->setLinearLowerLimit(btVector3(0.0, 0.0, 0.0));
            ((btGeneric6DofConstraint*)joint)->setLinearUpperLimit(btVector3(0.0, 0.0, 0.0));
            ((btGeneric6DofConstraint*)joint)->setAngularLowerLimit(btVector3(0.0, 0.0, 0.0));
            ((btGeneric6DofConstraint*)joint)->setAngularUpperLimit(btVector3(0.0, 0.0, 0.0));
            if(link->jointType() == Link::CRAWLER_JOINT || link->jointType() == Link::PSEUDO_CONTINUOUS_TRACK){
                body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
                body->setUserPointer(this);
                if(!gContactAddedCallback)
                    gContactAddedCallback = CustomMaterialCombinerCallback;
            }
        }
        break;
    }
}


BulletLink::~BulletLink()
{
    if(pMeshData)
        delete pMeshData;
    if(trimesh)
        delete trimesh;

    btCompoundShape* compoundShape = dynamic_cast<btCompoundShape*>(collisionShape);
    if(compoundShape){
        int num = compoundShape->getNumChildShapes();
        for(int i=0; i<num; i++){
            delete compoundShape->getChildShape(i);
        }
        delete compoundShape;
    }else{
        if(collisionShape)
            delete collisionShape;
    }
    if(motionState)
        delete motionState;
    if(body){
        delete body;
    }
}

void BulletLink::setKinematicStateToBullet()
{
    const Matrix3& R = link->R();
    btTransform transform;
    btMatrix3x3 btR(R(0,0), R(0,1), R(0,2),
                    R(1,0), R(1,1), R(1,2),
                    R(2,0), R(2,1), R(2,2));
    transform.setBasis(btR);
    const Vector3& p = link->p();
    transform.setOrigin(btVector3(p(0), p(1), p(2)));
    body->setWorldTransform(transform*shift);

    const Vector3 lc = R * link->c();
    const Vector3 v = link->v() + link->w().cross(lc);
    body->setLinearVelocity(btVector3(v(0), v(1), v(2)));
    const Vector3& w = link->w();
    body->setAngularVelocity(btVector3(w(0), w(1), w(2)));
    body->activate(true);
}

void BulletLink::getKinematicStateFromBullet()
{
    if(link->isRoot() && link->isFixedJoint()){
        return;
    }
    btTransform trans;
    motionState->getWorldTransform(trans);
    btTransform strans =  trans * invShift;
    Matrix3 R;
    btMatrix3x3& btR = strans.getBasis();
    link->R() << btR[0][0], btR[0][1], btR[0][2],
        btR[1][0], btR[1][1], btR[1][2],
        btR[2][0], btR[2][1], btR[2][2];
    btVector3& p = strans.getOrigin();
    link->p() << p[0], p[1], p[2];

#if DEBUG_OUT
    std::cout << link->q() << std::endl;
    std::cout << R(0,0) << " " << R(0,1) << " " << R(0,2) << std::endl;
    std::cout << R(1,0) << " " << R(1,1) << " " << R(1,2) << std::endl;
    std::cout << R(2,0) << " " << R(2,1) << " " << R(2,2) << std::endl;
    std::cout << link->p().x() << "," << link->p().y() << "," << link->p().z() << std::endl;
    std::cout << std::endl;
#endif

    if(link->isRotationalJoint()){
        link->q() = ((btHingeConstraint*)joint)->getHingeAngle() - q_offset;
    } else if(link->isSlideJoint()){
        link->q() = ((btGeneric6DofConstraint*)joint)->getRelativePivotPosition(2) - q_offset;      
    }
}

void BulletLink::setTorqueToBullet()
{
    if(link->isRotationalJoint()){
        const Vector3 u = link->u() * link->a();
        const Vector3 uu = link->R() * u;
        btVector3 torque(uu(0), uu(1), uu(2));
        body->applyTorque(torque);
        parent->body->applyTorque(-torque);

    } else if(link->isSlideJoint()){
        const Vector3 u = link->u() * link->d();
        const Vector3 uu = link->R() * u;
        btVector3 torque(uu(0), uu(1), uu(2));
        btVector3 pos(0,0,0);
        body->applyForce(torque, invShift*pos);
        const Vector3& b = link->b();
        btVector3 pos1(b(0), b(1), b(2));
        parent->body->applyForce(-torque, parent->invShift*pos1);
    }
}

void BulletLink::setVelocityToBullet()
{
    if(link->isRotationalJoint()){
    	double v = link->dq();
    	((btHingeConstraint*)joint)->enableAngularMotor(true, v, numeric_limits<double>::max());

    } else if(link->isSlideJoint()){
    	double v = link->dq();
    	((btGeneric6DofConstraint*)joint)->getTranslationalLimitMotor()->m_enableMotor[2] = true;
		((btGeneric6DofConstraint*)joint)->getTranslationalLimitMotor()->m_targetVelocity[2] = v;
		((btGeneric6DofConstraint*)joint)->getTranslationalLimitMotor()->m_maxMotorForce[2] =  numeric_limits<double>::max();
    }
}


////////////////////////////////Body////////////////////////////////////////
BulletBody::BulletBody(const Body& orgBody)
    : SimulationBody(new Body(orgBody))
{
    body = SimulationBody::body();
}


BulletBody::~BulletBody()
{
    for(size_t i=0; i < bulletLinks.size(); ++i){
        if(bulletLinks[i]->joint){
            delete bulletLinks[i]->joint;
        }
    }
    for(size_t i=0; i < extraJoints.size(); ++i){
        if(extraJoints[i]){
            delete extraJoints[i];
        }
    }
    for(size_t i=0; i < forceSensorFeedbacks.size(); ++i){
        if(forceSensorFeedbacks[i]){
            delete forceSensorFeedbacks[i];
        }
    }
}

void BulletBody::createBody(BulletSimulatorItemImpl* simImpl, short group)
{
    dynamicsWorld = simImpl->dynamicsWorld;

    BulletLink* rootLink = new BulletLink(simImpl, this, 0, Vector3::Zero(), body->rootLink(), 
                                          group, bodyItem()->isSelfCollisionDetectionEnabled());

    if(simImpl->useWorldCollision){
        geometryId = addBodyToCollisionDetector(*body, *simImpl->collisionDetector, 
                                                bodyItem()->isSelfCollisionDetectionEnabled());
    }

    setKinematicStateToBullet();

    if(simImpl->useWorldCollision){
        int size = simImpl->geometryIdToLink.size();
        int numLinks = bulletLinks.size();
        simImpl->geometryIdToLink.resize(geometryId+numLinks);
        for(int i=0; i < numLinks; i++){
            BulletLink* bulletLink = bulletLinks[i].get();
            int index = bulletLink->link->index();
            simImpl->geometryIdToLink[geometryId+index] = bulletLink;
            simImpl->collisionDetector->updatePosition(geometryId+index, bulletLink->link->T());
        }
    }

    setExtraJoints();
    setTorqueToBullet();

    sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);

    // set joint feedbacks for force sensors
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    forceSensorFeedbacks.resize(forceSensors.size());
    for(size_t i=0; i < forceSensors.size(); ++i){
        forceSensorFeedbacks[i] = new btJointFeedback;
        bulletLinks[forceSensors[i]->link()->index()]->joint->setJointFeedback(forceSensorFeedbacks[i]);
    }
}


void BulletBody::setExtraJoints()
{
    extraJoints.clear();
    int n = body->numExtraJoints();

    for(int j=0; j < n; ++j){
        Body::ExtraJoint& extraJoint = body->extraJoint(j);
        
        BulletLinkPtr bulletLinkPair[2];
        for(int i=0; i < 2; ++i){
            BulletLinkPtr bulletLink;
            Link* link = extraJoint.link[i];
            if(link->index() < bulletLinks.size()){
                bulletLink = bulletLinks[link->index()];
                if(bulletLink->link == link){
                    bulletLinkPair[i] = bulletLink;
                }
            }
            if(!bulletLink){
                break;
            }
        }

        if(bulletLinkPair[1]){
            Link* link0 = bulletLinkPair[0]->link;
            Link* link1 = bulletLinkPair[1]->link;
            Vector3 p0 = link0->Rs() * extraJoint.point[0];  // link0 local position
            Vector3 a = link0->Rs() * extraJoint.axis;        // link0 local axis
            Vector3 p1 = link1->Rs() * extraJoint.point[1];  // link1 local position

            if(extraJoint.type == Body::EJ_PISTON){
                Vector3 u(0,0,1);
                Vector3 ty = a.cross(u);
                btMatrix3x3 btR;
                Matrix3 R0;
                if(ty.norm() == 0){
                    btR.setIdentity();
                    R0.setIdentity();
                }else{
                    ty.normalized();
                    Vector3 tx = ty.cross(a).normalized();
                    R0.col(0) = tx;
                    R0.col(1) = ty;
                    R0.col(2) = a;
                    btR.setValue(R0(0,0), R0(0,1), R0(0,2),
                                 R0(1,0), R0(1,1), R0(1,2),
                                 R0(2,0), R0(2,1), R0(2,2));
                }
                btTransform frameA,frameB;
                frameB.setBasis(btR);
                frameB.setOrigin(btVector3(p0(0), p0(1), p0(2)));
                frameA.setBasis(btR);
                frameA.setOrigin(btVector3(p1(0), p1(1), p1(2)));

                btGeneric6DofConstraint* joint = new btGeneric6DofConstraint(*(bulletLinkPair[1]->body), *(bulletLinkPair[0]->body),
                                                                             bulletLinkPair[1]->invShift*frameA, bulletLinkPair[0]->invShift*frameB, false);
                joint->setLinearLowerLimit(btVector3(0,0,1));  //Lowerlimit ＞ Upperlimit → axis is free
                joint->setLinearUpperLimit(btVector3(0,0,-1));
                joint->setAngularLowerLimit(btVector3(0,0,1));  
                joint->setAngularUpperLimit(btVector3(0,0,-1));    
                joint->calculateTransforms();
                dynamicsWorld->addConstraint(joint, true);
                extraJoints.push_back(joint);
            }else if(extraJoint.type == Body::EJ_BALL){
                btVector3 pivotInA(p0(0), p0(1), p0(2));
                btVector3 pivotInB(p1(0), p1(1), p1(2));
                btPoint2PointConstraint* joint = new btPoint2PointConstraint(*(bulletLinkPair[0]->body), *(bulletLinkPair[1]->body),
                                                                             bulletLinkPair[0]->invShift*pivotInA, bulletLinkPair[1]->invShift*pivotInB);
                dynamicsWorld->addConstraint(joint, true);
                extraJoints.push_back(joint);
            }
        }
    }
}

void BulletBody::setKinematicStateToBullet()
{
    for(size_t i=0; i < bulletLinks.size(); ++i){
        bulletLinks[i]->setKinematicStateToBullet();
    }
}

void BulletBody::getKinematicStateFromBullet()
{
    for(size_t i=0; i < bulletLinks.size(); ++i){
        bulletLinks[i]->getKinematicStateFromBullet();
    }
}

void BulletBody::setTorqueToBullet()
{
    for(size_t i=1; i < bulletLinks.size(); ++i){
        bulletLinks[i]->setTorqueToBullet();
    }
}

void BulletBody::updateForceSensors()
{
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(size_t i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        const Link* link = sensor->link();
        btTypedConstraint* joint = bulletLinks[link->index()]->joint;
        btJointFeedback* fb = joint->getJointFeedback();
        Vector3 f, tau;
        if(link->isRotationalJoint()){
            f   << fb->m_appliedForceBodyB.x(), fb->m_appliedForceBodyB.y(), fb->m_appliedForceBodyB.z();
            tau << fb->m_appliedTorqueBodyB.x(),  fb->m_appliedTorqueBodyB.y(),  fb->m_appliedTorqueBodyB.z();
        }else{
            f   << fb->m_appliedForceBodyA.x(), fb->m_appliedForceBodyA.y(), fb->m_appliedForceBodyA.z();
            tau << fb->m_appliedTorqueBodyA.x(),  fb->m_appliedTorqueBodyA.y(),  fb->m_appliedTorqueBodyA.z();
        }
       
        const Matrix3 R = link->R() * sensor->R_local();
        //const Vector3 p = link->p() + link->R() * sensor->p_local();
        const Vector3 p = link->R() * sensor->p_local();
        sensor->f()   = R.transpose() * f;
        sensor->tau() = R.transpose() * (tau - p.cross(f));
        sensor->notifyStateChange();
    }
}


void BulletBody::setVelocityToBullet()
{
    // Skip the root link
    for(size_t i=1; i < bulletLinks.size(); ++i){
        bulletLinks[i]->setVelocityToBullet();
    }
}


////////////////////////////////////SimItem imple////////////////////////////////////////
void BulletSimulatorItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BulletSimulatorItem>(N_("BulletSimulatorItem"));
    ext->itemManager().addCreationPanel<BulletSimulatorItem>();
}


BulletSimulatorItem::BulletSimulatorItem()
{
    impl = new BulletSimulatorItemImpl(this);
}


BulletSimulatorItemImpl::BulletSimulatorItemImpl(BulletSimulatorItem* self)
    : self(self)

{
    initialize();
    gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
    erp = DEFAULT_ERP;
    numIterations = DEFAULT_NUMITERATION;
    restitution = DEFAULT_RESTITUTION;
    friction = DEFAULT_FRICTION;
    erp2 = DEFAULT_ERP2;
    splitImpulsePenetrationThreshold = DEFAULT_SPLITIMPULSEPENETRATIONTHRESHOLD;
    useHACD = true;
    collisionMargin = DEFAULT_COLLISION_MARGIN;

    useWorldCollision = false;
    velocityMode = false;
}


BulletSimulatorItem::BulletSimulatorItem(const BulletSimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new BulletSimulatorItemImpl(this, *org.impl);
}


BulletSimulatorItemImpl::BulletSimulatorItemImpl(BulletSimulatorItem* self, const BulletSimulatorItemImpl& org)
    : self(self)
{
    initialize();
    gravity = org.gravity;
    erp = org.erp;
    numIterations = org.numIterations;
    restitution = org.restitution;
    friction = org.friction;
    erp2 = org.erp2;
    splitImpulsePenetrationThreshold = org.splitImpulsePenetrationThreshold;
    useHACD = org.useHACD;
    collisionMargin = org.collisionMargin;

    useWorldCollision = org.useWorldCollision;
    velocityMode = org.velocityMode;
}

void BulletSimulatorItemImpl::initialize()
{
    collisionConfiguration =0;
    dispatcher = 0;
    broadphase = 0;
    solver =0;
    dynamicsWorld = 0;
    self->SimulatorItem::setAllLinkPositionOutputMode(true);
}

BulletSimulatorItem::~BulletSimulatorItem()
{
    delete impl;
}


BulletSimulatorItemImpl::~BulletSimulatorItemImpl()
{
    clear();
}


void BulletSimulatorItem::setAllLinkPositionOutputMode(bool on)
{
    // The mode is not changed.
    // This simulator only supports the all link position output
    // because joint positions may be slightly changed
}


Item* BulletSimulatorItem::doDuplicate() const
{
    return new BulletSimulatorItem(*this);
}


SimulationBody* BulletSimulatorItem::createSimulationBody(Body* orgBody)
{
    return new BulletBody(*orgBody);
}


bool BulletSimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool BulletSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    clear();

    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    broadphase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    if(useWorldCollision){
        collisionDetector = self->collisionDetector();
        collisionDetector->clearGeometries();
    }

    btVector3 g(gravity.x(), gravity.y(), gravity.z());
    dynamicsWorld->setGravity(g);
    timeStep = self->worldTimeStep();
    setSolverParameter();

    short group = 1;
    for(size_t i=0; i < simBodies.size(); ++i){
        addBody(static_cast<BulletBody*>(simBodies[i]), group<<i);
    }

    if(useWorldCollision)
        collisionDetector->makeReady();
    else
        btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

    return true;
}

void BulletSimulatorItemImpl::clear()
{
    if(dynamicsWorld)
        delete dynamicsWorld;
    if(solver)
        delete solver;
    if(dispatcher)
        delete dispatcher;
    if(collisionConfiguration)
        delete collisionConfiguration;
    if(broadphase)
        delete broadphase;
}

void BulletSimulatorItemImpl::addBody(BulletBody* bulletBody, short group)
{
    Body* body = bulletBody->body;

    Link* rootLink = body->rootLink();
    rootLink->v().setZero();
    rootLink->dv().setZero();
    rootLink->w().setZero();
    rootLink->dw().setZero();

    for(int i=0; i < body->numJoints(); ++i){
        Link* joint = body->joint(i);
        joint->u() = 0.0;
        joint->dq() = 0.0;
        joint->ddq() = 0.0;
    }
    
    body->clearExternalForces();
    body->calcForwardKinematics(true, true);

    bulletBody->createBody(this, group);
}

void BulletSimulatorItem::initializeSimulationThread()
{

}


bool BulletSimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool BulletSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        BulletBody* bulletBody = static_cast<BulletBody*>(activeSimBodies[i]);
        bulletBody->body->setVirtualJointForces();
        if(velocityMode)
        	bulletBody->setVelocityToBullet();
        else
        	bulletBody->setTorqueToBullet();
    }

    dynamicsWorld->stepSimulation(timeStep,2,timeStep/2.);

#if DEBUG_OUT
    int numManifolds = dispatcher->getNumManifolds();
    for (int i=0;i<numManifolds;i++)
        {
            btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
            int numContacts = contactManifold->getNumContacts();
            for (int j=0;j<numContacts;j++)
                {
                    btManifoldPoint& pt = contactManifold->getContactPoint(j);
                    btVector3 ptA = pt.getPositionWorldOnA();
                    btVector3 ptB = pt.getPositionWorldOnB();
                    btVector3 normal = pt.m_normalWorldOnB; 
                    std::cout << i << " " << j <<" pos= " <<ptA[0]<<" "<<ptA[1]<<" "<<ptA[2]<<"norm= " <<
                        normal[0] << " " << normal[1] << " " << normal[2] << " Impulse= " << pt.getAppliedImpulse() <<std::endl;
                    std::cout << pt.m_lateralFrictionDir1.x() << " " << pt.m_lateralFrictionDir1.y() << " " << pt.m_lateralFrictionDir1.z() << " " << pt.m_appliedImpulseLateral1 << std::endl;
                    std::cout << pt.m_lateralFrictionDir2.x() << " " << pt.m_lateralFrictionDir2.y() << " " << pt.m_lateralFrictionDir2.z() << " " << pt.m_appliedImpulseLateral2 << std::endl;
                    std::cout << pt.m_contactMotion1 << std::endl;
                }
        }
#endif

    for(size_t i=0; i < activeSimBodies.size(); ++i){
        BulletBody* bulletBody = static_cast<BulletBody*>(activeSimBodies[i]);
        if(!bulletBody->sensorHelper.forceSensors().empty()){
            bulletBody->updateForceSensors();
        }
        bulletBody->getKinematicStateFromBullet();
        if(bulletBody->sensorHelper.hasGyroOrAccelerationSensors()){
            bulletBody->sensorHelper.updateGyroAndAccelerationSensors();
        }
    }
    return true;
}


void BulletSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void BulletSimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.min(0.0).max(1.0);
    putProperty(_("Error Reduction Parameter"), erp, changeProperty(erp));
    putProperty(_("Restitution"), restitution, changeProperty(restitution));
    putProperty(_("Friction"), friction, changeProperty(friction));
    putProperty(_("ERP2"), erp2, changeProperty(erp2));
    putProperty.min(1);
    putProperty(_("Num of Iterations"), numIterations, changeProperty(numIterations));
    putProperty.decimals(4).min(-1.0).max(1.0);
    putProperty(_("SplitImpulsePenetrationThreshold"), splitImpulsePenetrationThreshold, changeProperty(splitImpulsePenetrationThreshold));
    putProperty(_("use HACD"), useHACD, changeProperty(useHACD));
    putProperty(_("Collision Margin"), collisionMargin, changeProperty(collisionMargin));
    putProperty(_("Velocity Control Mode"), velocityMode, changeProperty(velocityMode));
}


bool BulletSimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void BulletSimulatorItemImpl::store(Archive& archive)
{
    archive.write("ErrorReductionParameter", erp);
    archive.write("NumIterations", numIterations);
    archive.write("Restitution", restitution);
    archive.write("Friction", friction);
    archive.write("ERP2", erp2);
    archive.write("SplitImpulsePenetrationThreshold", splitImpulsePenetrationThreshold);
    archive.write("useHACD", useHACD);
    archive.write("CollisionMargin", collisionMargin);
    archive.write("velocityMode", velocityMode);
}


bool BulletSimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void BulletSimulatorItemImpl::restore(const Archive& archive)
{
    archive.read("ErrorReductionParameter", erp);
    archive.read("NumIterations", numIterations);
    archive.read("Restitution", restitution);
    archive.read("Friction", friction);
    archive.read("ERP2", erp2);
    archive.read("SplitImpulsePenetrationThreshold", splitImpulsePenetrationThreshold);
    archive.read("useHACD", useHACD);
    archive.read("CollisionMargin", collisionMargin);
    archive.read("velocityMode", velocityMode);
}

void BulletSimulatorItemImpl::setSolverParameter()
{
    if(dynamicsWorld != NULL){
        btContactSolverInfo& slvInfo = dynamicsWorld->getSolverInfo();
        slvInfo.m_erp = erp;
        slvInfo.m_numIterations = numIterations;
        slvInfo.m_erp2 = erp2;
        slvInfo.m_splitImpulsePenetrationThreshold = splitImpulsePenetrationThreshold;
        
        slvInfo.m_solverMode |= SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION+SOLVER_USE_2_FRICTION_DIRECTIONS;
        slvInfo.m_solverMode |= SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;
    }
}
