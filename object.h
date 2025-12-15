#ifndef OBJECT_DEF
#define OBJECT_DEF

#include "bounding_box.h"
#include "math.h"
#include "game.h"
#include "mesh.h"
#include <stdio.h>

typedef struct _OctreeLeaf OctreeLeaf;

typedef struct {
	Object *obj1;
	Object *obj2;
	BoundingBox *bb1;
	BoundingBox *bb2;
	Vec3 axis;
	float overlap;
} Collision;

struct Object {

	void (*Free)(Object *this);
	void (*AddUser)(Object *this);
	void (*RemoveUser)(Object *this);
	void (*OnCollision)(Object *this, Object *obj2, BoundingBox *bb1, BoundingBox *bb2, Vec3 axis, float overlap);
	void (*Update)(Object *this);
	void (*Draw)(Object *this);
	void (*ObjUpdate)(Object *this);
	Object *(*Copy)(Object *this);

	Model *model;

	float matrix[16];
	float matrixInvTranspose[16];
	float rotationMatrix[16];
	float transMatrix[16];
	float scaleMatrix[16];

	u8 occluder;
	u8 transparent;
	Skeleton *skeleton;

	int shader;

	BoundingBox bb;
	BoundingBox skelBb;
	char frozen;
	
	char offScreenUpdated;

	OctreeLeaf *inOctant;

	void *data;

	char storeLastCollisions;
	Collision *lastCollisions;
	int nLastCollisions;

	int numUsers; // always remember to inc
};

Object *Object_Create();
Object *Object_Copy(Object *obj);
void Object_UpdateSkeleton(Object *obj, Skeleton *skel);

int Object_SkeletonCollision(BoundingBox *bb1, BoundingBox *bb2, Vec3 *axis, float *overlap);
void Object_Free(Object *obj);
void Object_Freeze(Object *obj);
void Object_SetModel(Object *obj, Model *model);

#endif