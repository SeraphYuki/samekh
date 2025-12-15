#ifndef BOUNDING_BOX_DEF
#define BOUNDING_BOX_DEF

#include "math.h"

typedef struct _BoundingBox {
	struct _BoundingBox *parent;
	struct _BoundingBox *children;
	short numChildren;
	Cube cube;
	Cube wsCube;
	float matrix[16];
	Vec3 points[8];
	Vec3 axes[3];
	Vec3 pos;
	Vec3 rot;
	Vec3 scale;
	char **types;
	int nTypes;
} BoundingBox;

typedef struct Object Object;

int BoundingBox_IsSAT(BoundingBox *bb);

int BoundingBox_CheckCollision(BoundingBox *bb, BoundingBox *bb2);

void BoundingBox_AddType(BoundingBox *bb, char *name);

void BoundingBox_Rotate(BoundingBox *bb, Vec3 rot);
void BoundingBox_Scale(BoundingBox *bb, Vec3 scale);
void BoundingBox_SetPos(BoundingBox *bb, Vec3 pos);
void BoundingBox_UpdatePoints(BoundingBox *bb);
void BoundingBox_UpdateWorldSpaceCube(BoundingBox *bb);

BoundingBox BoundingBox_Create(Cube cube, Vec3 pos);

int BoundingBox_AddChild(BoundingBox *bb, BoundingBox *child);

float SAT_Collision(Vec3 *pointsA, Vec3 *pointsB, Vec3 *axesA, Vec3 *axesB, float *overlap, Vec3 *axis);
void BoundingBox_FreeData(BoundingBox *bb);

void BoundingBox_Copy(BoundingBox *into, BoundingBox *bb);

int BoundingBox_SATCollision(BoundingBox *bb1, BoundingBox *bb2, float *overlap, Vec3 *axis);
int BoundingBox_ResolveCollision(Object *obj1, BoundingBox *bb, Object *obj2, BoundingBox *bb2);

Vec3 BoundingBox_GetPosition(BoundingBox *bb);

float BoundingBox_CheckCollisionRay(BoundingBox *bb, Ray ray, BoundingBox **closest);

BoundingBox *BoundingBox_GetTop(BoundingBox *bb);

#endif
