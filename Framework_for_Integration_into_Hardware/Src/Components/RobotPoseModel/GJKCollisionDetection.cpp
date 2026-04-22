#include <iostream>

#include "GJKCollisionDetection.h"

//======================================================================
// Public functions
//======================================================================
//----------------------------------------------------------------------
// Constructors and Destructor
//----------------------------------------------------------------------
GJKCollisionDetection::GJKCollisionDetection()
{
    _pObjectShape = nullptr;
    _vpCollisionCheckShapes.clear();
}

GJKCollisionDetection::GJKCollisionDetection(const GJKCollisionDetection& gjk)
{
    _pObjectShape = gjk._pObjectShape;
    _vpCollisionCheckShapes = gjk._vpCollisionCheckShapes;
}

GJKCollisionDetection::~GJKCollisionDetection()
{
}

//----------------------------------------------------------------------
// Set shape data
//----------------------------------------------------------------------
void GJKCollisionDetection::setObjectShape(vtkPointSet* shape)
{
    _pObjectShape = shape;
}

void GJKCollisionDetection::addCollisionCheckShape(vtkPointSet* shape)
{
    if (shape != nullptr)
        _vpCollisionCheckShapes.push_back(shape);
}

//----------------------------------------------------------------------
// Check for collision between main object and any of the collision
// check objects
//----------------------------------------------------------------------
bool GJKCollisionDetection::isColliding()
{
    bool collision = false;

    for (vtkPointSet* pCollisionCheckShape : _vpCollisionCheckShapes) {
        CollisionResult_t result = checkForIntersection(pCollisionCheckShape);
        if (result == FOUND_INTERSECTION) {
            collision = true;
            break;
        }
    }

    return collision;
}

//======================================================================
// Private functions
//======================================================================
//----------------------------------------------------------------------
// Check for a collision between the main object and another object
//----------------------------------------------------------------------
GJKCollisionDetection::CollisionResult_t GJKCollisionDetection::checkForIntersection(vtkPointSet* pCollisionCheckShape)
{
    if (_pObjectShape == nullptr || _pObjectShape->GetNumberOfPoints() <= 0) {
        std::cerr << "Warning in GJKCollisionDetection::checkForIntersection: no main object defined." << std::endl;
        return NO_INTERSECTION;
    }
    if (pCollisionCheckShape == nullptr || pCollisionCheckShape->GetNumberOfPoints() <= 0) {
        std::cerr << "Warning in GJKCollisionDetection::checkForIntersection: no comparison object defined." << std::endl;
        return NO_INTERSECTION;
    }

    CollisionResult_t result = SEARCHING;
    _simplexVertices.clear();
    while (result == SEARCHING)
        result = evolveSimplex(pCollisionCheckShape);

    return result;
}

//----------------------------------------------------------------------
// Update the simplex.
// The simplex searches the Minkowski difference between the shapes for
// the origin. If the diference contains the origin, then the shapes are
// intersecting.
//----------------------------------------------------------------------
GJKCollisionDetection::CollisionResult_t GJKCollisionDetection::evolveSimplex(vtkPointSet* ptsB)
{
    vtkPointSet* const ptsA = _pObjectShape;

    if (_simplexVertices.size() == 0) {
        // No simplex yet: initialise parameters and set first direction between object centres
        double pt[3];
        double x = 0.0, y = 0.0, z = 0.0;
        int nA = ptsA->GetNumberOfPoints();
        for (int p = 0; p < nA; p++) {
            ptsA->GetPoint(p, pt);
            x += pt[0];
            y += pt[1];
            z += pt[2];
        }
        _centreA.set(x / double(nA), y / double(nA), z / double(nA));

        x = 0.0;  y = 0.0;  z = 0.0;
        int nB = ptsB->GetNumberOfPoints();
        for (int p = 0; p < nB; p++) {
            ptsB->GetPoint(p, pt);
            x += pt[0];
            y += pt[1];
            z += pt[2];
        }
        _centreB.set(x / double(nB), y / double(nB), z / double(nB));

        _simplexEvolveDirection = _centreB - _centreA;
        _simplexEvolveDirection.normalise();
    }

    else if (_simplexVertices.size() == 1) {
        // First point is set. Next point is in the opposite direction, for maximum size
        _simplexEvolveDirection *= -1;
        _simplexEvolveDirection.normalise();
    }

    else if (_simplexVertices.size() == 2) {
        // Two points are set. Next point makes a triangle
        Point3D AB = _simplexVertices[1] - _simplexVertices[0];  // Line between first two points
        Point3D A0 = _simplexVertices[0] * -1.0;  // Line to origin from first point

        // Vector triple product for direction perpendicular to AB in direction of the origin
        // (AB X A0) X AB = -(AB.A0)AB + (AB.AB)A0
        _simplexEvolveDirection = AB * -(AB.dot(A0)) + A0 * (AB.dot(AB));
        _simplexEvolveDirection.normalise();
    }

    else if (_simplexVertices.size() == 3) {
        // Already a triangle. Add the final point
        Point3D AC = _simplexVertices[2] - _simplexVertices[0];
        Point3D AB = _simplexVertices[1] - _simplexVertices[0];
        _simplexEvolveDirection = AC.cross(AB);
        _simplexEvolveDirection.normalise();

        // New direction should be towards the origin
        Point3D A0 = _simplexVertices[0] * -1.0;
        if (_simplexEvolveDirection.dot(A0) < 0)
            _simplexEvolveDirection *= -1.0;
    }

    else if (_simplexVertices.size() == 4) {
        //for (int i = 0; i < 4; i++)
        //    std::cerr << i << " " << _simplexVertices[i].X() << " " << _simplexVertices[i].Y() << " " << _simplexVertices[i].Z() << std::endl;
        // Already a tetrahedron. Check whether it contains the origin
        Point3D AD = _simplexVertices[3] - _simplexVertices[0];
        Point3D BD = _simplexVertices[3] - _simplexVertices[1];
        Point3D CD = _simplexVertices[3] - _simplexVertices[2];
        Point3D D0 = _simplexVertices[3] * -1.0;

        // Triangle normals
        Point3D ABDNormal = AD.cross(BD);
        Point3D BCDNormal = BD.cross(CD);
        Point3D CADNormal = CD.cross(AD);

        // Check which side of each triangle has the origin, and evovle the simplex
        // We already know the origin is inside ABC, by definition of D.
        std::vector<Point3D>::iterator it = _simplexVertices.begin();
        if (ABDNormal.dot(D0) > 0) {
            // Origin is outside ABD, so replace vertex C
            _simplexVertices.erase(it+2);
            _simplexEvolveDirection = ABDNormal;
            _simplexEvolveDirection.normalise();
        }
        else if (BCDNormal.dot(D0) > 0) {
            // Origin is outside BCD, so replace vertex A
            _simplexVertices.erase(it);
            _simplexEvolveDirection = BCDNormal;
            _simplexEvolveDirection.normalise();
        }
        else if (CADNormal.dot(D0) > 0) {
            // Origin is outside CAD, so replace vertex B
            // In this case, swap C and D to keep tetrahedron point order
            // the same (hence normal directions pointing outwards)
            Point3D tmp(_simplexVertices[2]);
            _simplexVertices[2].set(_simplexVertices[3]);
            _simplexVertices[3].set(tmp);
            _simplexVertices.erase(it+1);
            _simplexEvolveDirection = CADNormal;
            _simplexEvolveDirection.normalise();
        }
        else {
            // Origin is inside the tetrahedron, indicating an intersection
            return FOUND_INTERSECTION;
        }
    }

    else {
        std::cerr << "Error in GJKCollisionDetection::EvolveSimplex: unexpected number of vertices (" << _simplexVertices.size() << ") in simplex." << std::endl;
        return UNKNOWN_STATUS;
    }

    bool result = addSupportPoint(ptsA, ptsB, _simplexEvolveDirection);
    if (result)
        return SEARCHING;
    else
        return NO_INTERSECTION;
}

//----------------------------------------------------------------------
// Find the support point (i.e. the furthest point on the edge of the
// shape) in a given direction, on the Minkowski difference shape.
// The support point is always one of the vertices.
//----------------------------------------------------------------------
bool GJKCollisionDetection::addSupportPoint(vtkPointSet* ptsA, vtkPointSet* ptsB, const Point3D& direction)
{
    Point3D newVertex = calculateMinkowskiDifferenceSupportPoint(ptsA, ptsB, direction);
    _simplexVertices.push_back(newVertex);

    return direction.dot(newVertex) >= 0;
}

//----------------------------------------------------------------------
// Calculate support(ptsA, direction) - support(ptsB, -direction)
//----------------------------------------------------------------------
Point3D GJKCollisionDetection::calculateMinkowskiDifferenceSupportPoint(vtkPointSet* ptsA, vtkPointSet* ptsB, const Point3D& direction)
{
    Point3D oppositeDirection = direction * -1.0;
    Point3D supportA, supportB;

    calculateSupport(ptsA, direction, supportA);
    calculateSupport(ptsB, oppositeDirection, supportB);

    return supportA - supportB;
}

void GJKCollisionDetection::calculateSupport(vtkPointSet* pts, const Point3D& direction, Point3D& supportPoint)
{
    double pt[3];
    double maxDistance = 0.0;

    for (int p = 0; p < pts->GetNumberOfPoints(); p++) {
        pts->GetPoint(p, pt);
        double distance = pt[0] * direction.X() + pt[1] * direction.Y() + pt[2] * direction.Z();  // Dot product
        if (p == 0 || distance > maxDistance) {
            maxDistance = distance;
            supportPoint.set(pt[0], pt[1], pt[2]);
        }
    }
}
