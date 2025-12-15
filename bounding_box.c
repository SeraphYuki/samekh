#include "bounding_box.h"
#include "window.h"
#include "object.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"

static float GetOverlap(float minA, float maxA, float minB, float maxB){

    if(minA > maxB || minB > maxA)
        return 0; 

    double smallest = HUGE_VAL;
    double max0min1 = maxA - minB;
    double max1min0 = maxB - minA;
    double min0max1 = minA - maxB;

    if(minA < minB){
        if(maxA < maxB)
            smallest = maxA - minB;
        else
            if(maxA - minB < maxB - minA) 
            	smallest = max0min1; 
            else 
	            smallest = -max1min0;
    } else {
        if(maxA > maxB)
            smallest = min0max1;            
        else
            if(max0min1 < max1min0) 
            	smallest = max0min1; 
            else 
	            smallest = -max1min0;
    }

    return smallest;
 }

static int CheckCollision(Vec3 *axes, Vec3 *pointsA, int nPointsA, Vec3 *pointsB, int nPointsB, 
	float *minOverlap, Vec3 *minAxis){

	*minOverlap = HUGE_VAL;
	int k;
	for(k = 0; k < 15; k++){

		float minA = HUGE_VAL, maxA = -HUGE_VAL, minB = HUGE_VAL, maxB = -HUGE_VAL;

		if(axes[k].x == 0 && axes[k].y == 0 && axes[k].z == 0 ) return 0;

		axes[k] = Math_Vec3Normalize(axes[k]);

		int j;
		for(j = 0; j < nPointsB; j++){
			float dot = Math_Vec3Dot(axes[k], pointsB[j]);
			if(dot < minB)
				minB = dot;
			if(dot > maxB)
				maxB = dot;
		}
		for(j = 0; j < nPointsA; j++){
			float dot = Math_Vec3Dot(axes[k], pointsA[j]);
			if(dot < minA)
				minA = dot;
			if(dot > maxA)
				maxA = dot;
		}

		float overlap = GetOverlap(minA, maxA,minB,maxB);
		if(fabs(overlap) < *minOverlap){
			*minOverlap = fabs(overlap);
			if(overlap > 0)
				*minAxis = axes[k];
			else
				*minAxis = (Vec3){-axes[k].x,-axes[k].y,-axes[k].z};
		}

		if(overlap == 0) return 0;
	}

	return 1;
}

float SAT_Collision(Vec3 *pointsA, Vec3 *pointsB, Vec3 *axesA, Vec3 *axesB, float *overlap, Vec3 *axis){

		Vec3 axes[] = {
			axesA[0],
			axesA[1],
			axesA[2],
			axesB[0],
			axesB[1],
			axesB[2],
			Math_Vec3Cross(axesA[0], axesB[0]),
			Math_Vec3Cross(axesA[0], axesB[1]),
			Math_Vec3Cross(axesA[0], axesB[2]),

			Math_Vec3Cross(axesA[1], axesB[0]),
			Math_Vec3Cross(axesA[1], axesB[1]),
			Math_Vec3Cross(axesA[1], axesB[2]),

			Math_Vec3Cross(axesA[2], axesB[0]),
			Math_Vec3Cross(axesA[2], axesB[1]),
			Math_Vec3Cross(axesA[2], axesB[2])
		};

		if(CheckCollision(axes, pointsA,8,pointsB, 8, overlap, axis))
			return 1;


		return 0;
}

int BoundingBox_SATCollision(BoundingBox *bb1, BoundingBox *bb2, float *overlap, Vec3 *axis){


	return SAT_Collision(bb1->points, bb2->points, bb1->axes, bb2->axes, overlap, axis);
}

void BoundingBox_Copy(BoundingBox *into, BoundingBox *bb){

	memcpy(into, bb, sizeof(BoundingBox));

	into->children = NULL;

	into->children = (BoundingBox *)malloc(bb->numChildren*sizeof(BoundingBox));
	into->types = (char **)malloc(bb->nTypes*sizeof(char *));

	int k;
	for(k = 0; k < bb->nTypes; k++){
		
		into->types[k] = malloc(strlen(bb->types[k]) + 1);
		strcpy(into->types[k], bb->types[k]);
		into->types[k][strlen(bb->types[k])] = 0;
	}

	for(k = 0; k < bb->numChildren; k++){
		BoundingBox_Copy(&into->children[k], &bb->children[k]);
		into->children[k].parent = into;
	}
}

void BoundingBox_AddType(BoundingBox *bb, char *name){
	
	if(!strlen(name)) return;

	bb->types = (char **)realloc(bb->types, sizeof(char *) * ++bb->nTypes);
	bb->types[bb->nTypes-1] = malloc(strlen(name) + 1);
	memset(bb->types[bb->nTypes-1], 0, strlen(name) + 1);
	strcpy(bb->types[bb->nTypes-1], name);
}

void BoundingBox_FreeData(BoundingBox *bb){
	int k;

	if(bb->types){

		for(k = 0; k < bb->nTypes; k++)
			free(bb->types[k]);
		
		free(bb->types);
	}

	if(bb->children){
		
		for(k = 0; k < bb->numChildren; k++)
			BoundingBox_FreeData(&bb->children[k]);
		
		free(bb->children);
	}

	memset(bb, 0, sizeof(BoundingBox));

	if(bb->parent){
		
		for(k = 0; k < bb->parent->numChildren; k++){
			if(&bb->parent->children[k] == bb){
				
				int j;
				for(j = k; j < bb->parent->numChildren-1; j++)
					bb->parent->children[j] = bb->parent->children[j+1];
			
				bb->parent->children = (BoundingBox *)realloc(bb->parent->children, 
					sizeof(BoundingBox) * --bb->parent->numChildren);
			
				break;
			}

		}
	}
}

Vec3 BoundingBox_GetPosition(BoundingBox *bb){

	return Math_CubeXYZ(bb->wsCube);
}

BoundingBox *BoundingBox_GetTop(BoundingBox *bb){

	if(bb->parent)
		return BoundingBox_GetTop(bb->parent);

	return bb;
}

BoundingBox BoundingBox_Create(Cube cube, Vec3 pos){

	BoundingBox bb;
	memset(&bb, 0, sizeof(BoundingBox));

	bb.cube = cube;
	bb.pos = pos;
	BoundingBox_UpdatePoints(&bb);
	return bb;
}

void BoundingBox_Rotate(BoundingBox *bb, Vec3 rot){
	bb->rot = rot;
	BoundingBox_UpdatePoints(bb);
}

void BoundingBox_Scale(BoundingBox *bb, Vec3 scale){
	bb->scale = scale;
	BoundingBox_UpdatePoints(bb);
}

void BoundingBox_SetPos(BoundingBox *bb, Vec3 pos){
	bb->pos = pos;
	BoundingBox_UpdatePoints(bb);
}

int BoundingBox_AddChild(BoundingBox *bb, BoundingBox *child) {

	int index = bb->numChildren;

	if(bb->children == NULL)
		bb->children = (BoundingBox *)malloc((++bb->numChildren)*sizeof(BoundingBox));
	else
		bb->children = (BoundingBox *)realloc(bb->children, (++bb->numChildren)*sizeof(BoundingBox));

	BoundingBox_Copy(&bb->children[index], child);
	bb->children[index].parent = bb;

	return index;
}

int BoundingBox_CheckCollision(BoundingBox *bb, BoundingBox *bb2){


	int ret = 0;
	if(!Math_CheckCollisionCube(bb->wsCube, bb2->wsCube)) return ret;

	int k;
    for(k = 0; k < bb->numChildren; k++){
    	ret += BoundingBox_CheckCollision(&bb->children[k], bb2);
	}
    for(k = 0; k < bb2->numChildren; k++){
    	ret += BoundingBox_CheckCollision(bb, &bb2->children[k]);
	}

	return ret+1;
}

int BoundingBox_ResolveCollision(Object *obj1, BoundingBox *bb, Object *obj2, BoundingBox *bb2){


	// children not iterative. need checking for each pair

	int ret = 0;
	if(!Math_CheckCollisionCube(bb->wsCube, bb2->wsCube))
		return ret;

	int k;
    for(k = 0; k < bb->numChildren; k++){
    	ret += BoundingBox_ResolveCollision(obj1, &bb->children[k], obj2, bb2);
	}
    for(k = 0; k < bb2->numChildren; k++){
    	ret += BoundingBox_ResolveCollision(obj1, bb, obj2, &bb2->children[k]);
	}


	Vec3 axis;
	float overlap = 0;
	if(BoundingBox_IsSAT(bb) || BoundingBox_IsSAT(bb2)){
		if(!BoundingBox_SATCollision(bb, bb2, &overlap, &axis)){
			return ret;	
		}
	}

	if(obj1 && (obj1->storeLastCollisions || obj1->OnCollision)){
		
		if(obj1->OnCollision) obj1->OnCollision(obj1, obj2, bb, bb2, axis, overlap);
		
		if(obj1->storeLastCollisions){
			obj1->lastCollisions = (Collision *)realloc(obj1->lastCollisions, sizeof(Collision) * ++obj1->nLastCollisions);
			obj1->lastCollisions[obj1->nLastCollisions-1] = (Collision){obj1, obj2, bb, bb2,axis,overlap};
		}
	}

	if(obj2 && (obj2->storeLastCollisions || obj2->OnCollision)){
		if(obj2->OnCollision) obj2->OnCollision(obj2, obj1, bb2, bb, axis, overlap);
		if(obj2->storeLastCollisions){
			obj2->lastCollisions = (Collision *)realloc(obj2->lastCollisions, sizeof(Collision) * ++obj2->nLastCollisions);
			obj2->lastCollisions[obj2->nLastCollisions-1] = (Collision){obj2, obj1, bb2, bb,axis,overlap};
		}
	}

    return ret+1;
}
void BoundingBox_UpdateWorldSpaceCube(BoundingBox *bb){
	bb->wsCube.x = bb->wsCube.y = bb->wsCube.z = HUGE_VAL;
	bb->wsCube.w = bb->wsCube.h = bb->wsCube.d = -HUGE_VAL;

	int k;
	for(k = 0; k < 8; k++){
		Vec3 pos = bb->points[k];
        if(pos.x < bb->wsCube.x)
            bb->wsCube.x = pos.x;
        if(pos.x > bb->wsCube.w)
            bb->wsCube.w = pos.x;
        if(pos.y < bb->wsCube.y)
            bb->wsCube.y = pos.y;
        if(pos.y > bb->wsCube.h)
            bb->wsCube.h = pos.y;
        if(pos.z < bb->wsCube.z)
            bb->wsCube.z = pos.z;
        if(pos.z >  bb->wsCube.d)
            bb->wsCube.d = pos.z;
    }
 
	bb->wsCube.w -= bb->wsCube.x;
    bb->wsCube.h -= bb->wsCube.y;
    bb->wsCube.d -= bb->wsCube.z;
}

void BoundingBox_UpdatePoints(BoundingBox *bb){

    bb->points[0] = (Vec3){bb->cube.x, bb->cube.y+bb->cube.h, bb->cube.z+bb->cube.d};
    bb->points[1] = (Vec3){bb->cube.x, bb->cube.y, bb->cube.z+bb->cube.d};
    bb->points[2] = (Vec3){bb->cube.x, bb->cube.y+bb->cube.h, bb->cube.z};
    bb->points[3] = (Vec3){bb->cube.x, bb->cube.y, bb->cube.z};
    bb->points[4] = (Vec3){bb->cube.x+bb->cube.w, bb->cube.y+bb->cube.h, bb->cube.z+bb->cube.d};
    bb->points[5] = (Vec3){bb->cube.x+bb->cube.w, bb->cube.y, bb->cube.z+bb->cube.d};
    bb->points[6] = (Vec3){bb->cube.x+bb->cube.w, bb->cube.y+bb->cube.h, bb->cube.z};
    bb->points[7] = (Vec3){bb->cube.x+bb->cube.w, bb->cube.y, bb->cube.z};

    float rmatrix[16];
    Math_RotateMatrix(rmatrix,bb->rot);

    bb->axes[0] = (Vec3){1,0,0};
    bb->axes[1] = (Vec3){0,1,0};
    bb->axes[2] = (Vec3){0,0,1};
    bb->axes[0] = Math_Vec3Normalize(Math_MatrixMult(bb->axes[0], rmatrix));
    bb->axes[1] = Math_Vec3Normalize(Math_MatrixMult(bb->axes[1], rmatrix));
    bb->axes[2] = Math_Vec3Normalize(Math_MatrixMult(bb->axes[2], rmatrix));

	Math_ScalingMatrixXYZ(bb->matrix, bb->scale);
    Math_MatrixMatrixMult(bb->matrix, rmatrix, bb->matrix);	

    float matrix[16];
    Math_TranslateMatrix(matrix, bb->pos);
    Math_MatrixMatrixMult(bb->matrix, matrix, bb->matrix);

   if(bb->parent)
       Math_MatrixMatrixMult(bb->matrix, bb->parent->matrix, bb->matrix); 

    bb->points[0] = Math_MatrixMult(bb->points[0], bb->matrix);
    bb->points[1] = Math_MatrixMult(bb->points[1], bb->matrix);
    bb->points[2] = Math_MatrixMult(bb->points[2], bb->matrix);
    bb->points[3] = Math_MatrixMult(bb->points[3], bb->matrix);
    bb->points[4] = Math_MatrixMult(bb->points[4], bb->matrix);
    bb->points[5] = Math_MatrixMult(bb->points[5], bb->matrix);
    bb->points[6] = Math_MatrixMult(bb->points[6], bb->matrix);
    bb->points[7] = Math_MatrixMult(bb->points[7], bb->matrix);

	BoundingBox_UpdateWorldSpaceCube(bb);

	int k;	
	if(bb->children){
		for(k = 0; k < bb->numChildren; k++)
			BoundingBox_UpdatePoints(&bb->children[k]);
		
	}
}

int BoundingBox_IsSAT(BoundingBox *bb){
	if(bb->rot.x == 0 && bb->rot.y == 0 && bb->rot.z == 0){
		return 0;
	}
	return 1;
}

float BoundingBox_CheckCollisionRay(BoundingBox *bb, Ray ray, BoundingBox **closest){

	float ret = HUGE_VAL;

	int k;

	for(k = 0; k < bb->numChildren; k++){

		BoundingBox *tBb;

		float d = BoundingBox_CheckCollisionRay(&bb->children[k], ray, &tBb);
	
		if(d < ret){
			*closest = tBb;
			ret = d;
		}
	}

	return Math_CubeCheckCollisionRay(bb->wsCube, ray);
}