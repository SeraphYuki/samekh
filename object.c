#include <stdlib.h>
#include "object.h"
#include "memory.h"

static void AddUser(Object *obj){
    obj->numUsers++;
}


static void RemoveUser(Object *obj){
    obj->numUsers--;
    if(obj->numUsers == 0)
    	Object_Free(obj);
 }

static void ObjUpdate(Object *obj){
 	BoundingBox_UpdatePoints(&obj->bb);
 }


Object *Object_Create(){
	Object *obj =  malloc(sizeof(Object));
	memset(obj, 0 , sizeof(Object));
	obj->RemoveUser = RemoveUser;
	obj->AddUser = AddUser;
	obj->ObjUpdate = ObjUpdate;
	return obj;
 }

Object *Object_Copy(Object *obj){
	Object *obj2 = malloc(sizeof(Object));
	memcpy(obj2, obj, sizeof(Object));
	return obj2;
}
int Object_SkeletonCollision(BoundingBox *bb1, BoundingBox *bb2, Vec3 *axis, float *overlap){
	int k;
	for(k = 0; k < bb2->numChildren; k++){

		if(BoundingBox_SATCollision(bb1, &bb2->children[k], overlap, axis)){
			return 1;
		}
	}
	return 0;
}

void Object_UpdateSkeleton(Object *obj, Skeleton *skel){
	obj->skeleton = skel;
	BoundingBox_FreeData(&obj->skelBb);
	Cube cube;
	cube.x = cube.y = cube.z = HUGE_VAL;
	cube.w = cube.h = cube.d = -HUGE_VAL;

	int k;
	for(k = 0; k < skel->nBones; k++){
        BoundingBox child;
        memset(&child, 0, sizeof(BoundingBox));
		int m;
		for(m = 0; m < 8; m++){
			Vec3 pos = skel->bones[k].points[m];
			pos = Math_Vec3MultVec3(skel->bones[k].points[m], obj->bb.scale);
			pos = Math_Vec3AddVec3(pos, obj->bb.pos);
			child.points[m] = pos;

			pos = skel->bones[k].points[m];
	        if(pos.x < cube.x)
	            cube.x = pos.x;
	        if(pos.x > cube.w)
	            cube.w = pos.x;
	        if(pos.y < cube.y)
	            cube.y = pos.y;
	        if(pos.y > cube.h)
	            cube.h = pos.y;
	        if(pos.z < cube.z)
	            cube.z = pos.z;
	        if(pos.z >  cube.d)
	            cube.d = pos.z;
		}
		for(m = 0; m < 3; m++){
			child.axes[m] = skel->bones[k].axes[m];
		}
		BoundingBox_AddChild(&obj->skelBb,  &child);
		BoundingBox_UpdateWorldSpaceCube(&child);
	 
    }
	cube.w -= cube.x;
    cube.h -= cube.y;
    cube.d -= cube.z;
    obj->bb.points[0] = (Vec3){cube.x, cube.y+cube.h, cube.z+cube.d};
    obj->bb.points[1] = (Vec3){cube.x, cube.y, cube.z+cube.d};
    obj->bb.points[2] = (Vec3){cube.x, cube.y+cube.h, cube.z};
    obj->bb.points[3] = (Vec3){cube.x, cube.y, cube.z};
    obj->bb.points[4] = (Vec3){cube.x+cube.w, cube.y+cube.h, cube.z+cube.d};
    obj->bb.points[5] = (Vec3){cube.x+cube.w, cube.y, cube.z+cube.d};
    obj->bb.points[6] = (Vec3){cube.x+cube.w, cube.y+cube.h, cube.z};
    obj->bb.points[7] = (Vec3){cube.x+cube.w, cube.y, cube.z};
	memcpy(&obj->skelBb.points[0].x, &obj->bb.points[0].x, sizeof(Vec3)*8);
	BoundingBox_UpdateWorldSpaceCube(&obj->skelBb);
	// BoundingBox_UpdateWorldSpaceCube(&obj->bb);
	// obj->skelBb.cube = obj->skelBb.wsCube;
	// obj->bb.cube = obj->bb.wsCube;
}
void Object_SetModel(Object *obj, Model *model){
	obj->model= model; 
	memset(&obj->bb, 0, sizeof(BoundingBox));

	if(obj->model->numBB == 1){
        obj->bb.cube = obj->model->bb[0].cube;
		obj->bb.pos = obj->model->bb[0].pos;
		obj->bb.rot = obj->model->bb[0].rot;
		obj->bb.scale = obj->model->bb[0].scale;
		BoundingBox_UpdatePoints(&obj->bb);
		return;
	}

	Cube cube;
	cube.x = cube.y = cube.z = HUGE_VAL;
	cube.w = cube.h = cube.d = -HUGE_VAL;
	obj->bb.pos = (Vec3){0,0,0};
	obj->bb.rot = (Vec3){0,0,0};
	obj->bb.scale = (Vec3){1,1,1};
    
    int k;
    for(k = 0; k < obj->model->numBB; k++){
        BoundingBox child;
        memset(&child, 0, sizeof(BoundingBox));
        child.cube = obj->model->bb[k].cube;

		child.pos = obj->model->bb[k].pos;
		child.rot = obj->model->bb[k].rot;
		child.scale = obj->model->bb[k].scale;
		BoundingBox_AddChild(&obj->bb,  &child);
		BoundingBox_UpdatePoints(&child);
 
		if(child.wsCube.x < cube.x) cube.x = child.wsCube.x;       
		if(child.wsCube.y < cube.y) cube.y = child.wsCube.y;       
		if(child.wsCube.z < cube.z) cube.z = child.wsCube.z;
		if(child.wsCube.w+child.wsCube.z > cube.w) 
			cube.w = child.wsCube.w+child.wsCube.z;
		if(child.wsCube.h+child.wsCube.y > cube.h)
			cube.h = child.wsCube.h+child.wsCube.y;       
		if(child.wsCube.d+child.wsCube.z > cube.d)
			cube.d = child.wsCube.d+child.wsCube.z;
    }

	cube.w -= cube.x;
    cube.h -= cube.y;
    cube.d -= cube.z;
    obj->bb.points[0] = (Vec3){cube.x, cube.y+cube.h, cube.z+cube.d};
    obj->bb.points[1] = (Vec3){cube.x, cube.y, cube.z+cube.d};
    obj->bb.points[2] = (Vec3){cube.x, cube.y+cube.h, cube.z};
    obj->bb.points[3] = (Vec3){cube.x, cube.y, cube.z};
    obj->bb.points[4] = (Vec3){cube.x+cube.w, cube.y+cube.h, cube.z+cube.d};
    obj->bb.points[5] = (Vec3){cube.x+cube.w, cube.y, cube.z+cube.d};
    obj->bb.points[6] = (Vec3){cube.x+cube.w, cube.y+cube.h, cube.z};
    obj->bb.points[7] = (Vec3){cube.x+cube.w, cube.y, cube.z};
	BoundingBox_UpdateWorldSpaceCube(&obj->bb);
	obj->bb.cube = obj->bb.wsCube;

	BoundingBox_UpdatePoints(&obj->bb);
}
void Object_Free(Object *obj){
	free(obj);
}
void Object_Freeze(Object *obj){}
