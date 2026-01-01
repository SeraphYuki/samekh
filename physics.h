#ifndef PHYSICS_DEF
#define PHYSICS_DEF
#include "math.h"
#include "mesh.h"

#define MAX_BODIES 128
#define MAX_TREES 32
#define MAX_CONSTRAINTS 128


typedef struct PhysicsConstraint_t PhysicsConstraint_t;

struct PhysicsConstraint_t {



	Bone *body1;
	Bone *body2;

	Vec3 c1[2];
	Vec3 c2[2];

	Vec3 J1[6][2];
	Vec3 J2[6][2];
	void (*Evaluate)( PhysicsConstraint_t *);
	void *data;

	Vec3 offset;
	Quat offsetRotation;
	struct {
		Quat rot;
		Vec3 anchor;
		Vec3 axis;
		Vec3 bodyAxis;
		float angle;
		float cosHalfAngle;
		float sinHalfAngle;
	} cone;

	
} ;

struct FigureTree_t{
	Bone *bodies[MAX_BODIES];
	int nBodies;	
};

typedef struct {
	Bone *bone; 
	Vec3 J1[6][2];
	Vec3 J2[6][2];
	float lm;
	int responses[MAX_CONSTRAINTS];
	int nResponses;
	PhysicsConstraint_t *constraint;
} ConstraintSolution_t;

typedef struct {
	PhysicsConstraint_t constraints[MAX_CONSTRAINTS];
	FigureTree_t trees[MAX_TREES];
	Skeleton *skel;
	int nTrees;
	int nBodies;
	int nConstraints;
} PhysicsFigure_t;

void ConeConstraint_Create(PhysicsConstraint_t *constraint, Bone *body1,
								 Bone *body2, Vec3 axis, float coneAngle);

void FixedConstraint_Create(PhysicsConstraint_t *constraint, 
		Bone *body1, Bone *body2);

void Physics_ApplyForces(PhysicsFigure_t *figure);

#endif