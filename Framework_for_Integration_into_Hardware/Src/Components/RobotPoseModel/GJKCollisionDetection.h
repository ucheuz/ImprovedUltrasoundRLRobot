#ifndef GJK_COLLISION_DETECTION_H
#define GJK_COLLISION_DETECTION_H

#include <vector>

#include "Point3D.h"  // From TransMatrix library

#include <vtkPointSet.h>

//======================================================================
// This class allows checking for intersections between convex shapes,
// i.e. collision detection.
// It is based very closely on the code developed at
// https://blog.hamaluik.ca/posts/building-a-collision-engine-part-3-3d-gjk-collision-detection/
//======================================================================
class GJKCollisionDetection
{
public:
    GJKCollisionDetection();
    GJKCollisionDetection(const GJKCollisionDetection& gjk);
    ~GJKCollisionDetection();

    void setObjectShape(vtkPointSet* shape);
    void addCollisionCheckShape(vtkPointSet* shape);

    bool isColliding();

private:
    enum CollisionResult_t {NO_INTERSECTION=0, SEARCHING, FOUND_INTERSECTION, UNKNOWN_STATUS};

    vtkPointSet* _pObjectShape;
    std::vector<vtkPointSet*> _vpCollisionCheckShapes;

    // Variables needed by intersection check algorithm
    std::vector<Point3D> _simplexVertices;
    Point3D _centreA;
    Point3D _centreB;
    Point3D _simplexEvolveDirection;

    CollisionResult_t checkForIntersection(vtkPointSet* pCollisionCheckShape);
    CollisionResult_t evolveSimplex(vtkPointSet* ptsB);
    bool addSupportPoint(vtkPointSet* ptsA, vtkPointSet* ptsB, const Point3D& direction);
    Point3D calculateMinkowskiDifferenceSupportPoint(vtkPointSet* ptsA, vtkPointSet* ptsB, const Point3D& direction);
    void calculateSupport(vtkPointSet* pts, const Point3D& direction, Point3D& supportPoint);

};

#endif  // GJK_COLLISION_DETECTION_H
