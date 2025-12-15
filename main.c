#include <GL/glew.h>
#include "window.h"
#include "sound.h"
#include "world.h"
#include "mesh.h"
#include "image_loader.h"
#include "shaders.h"
#include "object.h"
#include "phsyics.h"
// #include "skybox.h"
#define WINDOW_WIDTH 960 
#define WINDOW_HEIGHT 544

static Object *cubeObj, *groundObj, *throwObj;
static Model cubeModel, groundModel, throwModel;
static Animation cubeAnim;
static Skeleton cubeSkel;
static PlayingAnimation cubeAnims[1];

// static Skybox skybox;

static float mouseSensitivity = 0.001;
static float moveSpeed = 0.005;

static Vec2 rotation = {0,0};
static Vec3 position = {2,1,5};

static char movingDirs[5];


float GetDeltaTime(void){
    return Window_GetDeltaTime();
}
static void onThrow(Object *obj, Object *obj2, BoundingBox *bb, BoundingBox *bb2, Vec3 axis, float overlap){
	obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,Math_Vec3MultFloat(axis, -overlap));
	Vec3 satAxis;
	float satOverlap = 0;
	if(Object_SkeletonCollision(&obj->bb, &obj2->skelBb, &satAxis, &satOverlap)){
		obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,Math_Vec3MultFloat(satAxis, -satOverlap));
	}
	obj->ObjUpdate(obj);
}
static void onCube(Object *obj, Object *obj2, BoundingBox *bb, BoundingBox *bb2, Vec3 axis, float overlap){
	obj->bb.pos = Math_Vec3AddVec3(obj->bb.pos,Math_Vec3MultFloat(axis, -overlap));
	obj->ObjUpdate(obj);	
}

static void Update(){

    cubeAnims[0].into += Window_GetDeltaTime() / 10.1f;

    if(cubeAnims[0].into > cubeAnim.length/1.5){
        cubeAnims[0].into = cubeAnim.length/8;
	}

    cubeObj->bb.pos.y -= Window_GetDeltaTime() / 1000.0f;
    if(cubeObj->bb.pos.y < 0){
        cubeObj->bb.pos.y = 0;
	}

	Skeleton_Update(&cubeSkel, cubeAnims, 1);
	Object_UpdateSkeleton(cubeObj, &cubeSkel);
	Vec3 moveVec = {0,0,0};

    if(movingDirs[4]) moveVec.y += 1;
	if(movingDirs[0]) moveVec.z -= 1;
	if(movingDirs[1]) moveVec.z += 1;
	if(movingDirs[2]) moveVec.x -= 1;
	if(movingDirs[3]) moveVec.x += 1;
    
    if(Math_Vec3Magnitude(moveVec)){

        moveVec = Math_Vec3Normalize(moveVec);

        moveVec = Math_Rotate(moveVec, (Vec3){-rotation.y, -rotation.x, 0});

        moveVec = Math_Vec3MultFloat(moveVec, Window_GetDeltaTime() * moveSpeed);

        // position.x += moveVec.x;
        // position.z += moveVec.z;
        cubeObj->bb.pos.x += moveVec.x;
        cubeObj->bb.pos.z += moveVec.z;
        cubeObj->bb.pos.y += moveVec.y;
    }

    cubeObj->ObjUpdate(cubeObj);
	cubeObj->OnCollision = onCube;
    World_UpdateObjectInOctree(cubeObj);
    World_ResolveCollisions(cubeObj, &cubeObj->bb);
    cubeObj->ObjUpdate(cubeObj);

	float thrown = GetDeltaTime() / 500.0f;
    Vec3 forward = Math_Rotate((Vec3){0,0,-1}, (Vec3){-rotation.y, -rotation.x, 0});
	throwObj->bb.pos.x += thrown * forward.x;
	throwObj->bb.pos.y += thrown * forward.y;
	throwObj->bb.pos.z += thrown * forward.z;
	if(Math_Vec3Magnitude(Math_Vec3SubVec3(throwObj->bb.pos,position)) > 5) throwObj->bb.pos = position;
    throwObj->OnCollision = onThrow;
    throwObj->ObjUpdate(throwObj);
    World_ResolveCollisions(throwObj, &throwObj->bb);
}

static void Event(SDL_Event ev){

    // Thoth_Event(thoth, ev);

    // if(ev.window.event == SDL_WINDOWEVENT_RESIZED || 
    //     ev.window.event == SDL_WINDOWEVENT_SIZE_CHANGED){
    //     int w = ev.window.data1;
    //     int h = ev.window.data2;

    //     Thoth_Resize(thoth, w/4, 0, (w/2) + (w/4), h);
    // }

	if(ev.type == SDL_MOUSEMOTION){

		rotation.x += mouseSensitivity * ev.motion.xrel;
		rotation.y += mouseSensitivity * ev.motion.yrel;

        while(rotation.x > PI*2) rotation.x -= PI*2;
        while(rotation.x < 0) rotation.x += PI*2;

        if(rotation.y < -PI/2.5) rotation.y = -PI/2.5;
        if(rotation.y > PI/3) rotation.y = PI/3;

	} else if(ev.type == SDL_KEYDOWN){

        if(ev.key.keysym.sym == SDLK_w)
            movingDirs[0] = 1;
        if(ev.key.keysym.sym == SDLK_q)
            movingDirs[4] = 1;
		else if(ev.key.keysym.sym == SDLK_s)
			movingDirs[1] = 1;
		else if(ev.key.keysym.sym == SDLK_a)
			movingDirs[2] = 1;
        else if(ev.key.keysym.sym == SDLK_d)
            movingDirs[3] = 1;
        else if(ev.key.keysym.sym == SDLK_ESCAPE)
            exit(0);

	} else if(ev.type == SDL_KEYUP){

        if(ev.key.keysym.sym == SDLK_w)
            movingDirs[0] = 0;
        if(ev.key.keysym.sym == SDLK_q)
            movingDirs[4] = 0;
        else if(ev.key.keysym.sym == SDLK_s)
			movingDirs[1] = 0;
		else if(ev.key.keysym.sym == SDLK_a)
			movingDirs[2] = 0;
		else if(ev.key.keysym.sym == SDLK_d)
			movingDirs[3] = 0;

        int k;
        for(k = 0; k < 4; k++) if(movingDirs[k] != 0) break;

	}
}

static void Focus(){

} 


static void DrawRigged(Object *obj){


    Shaders_UseProgram(SKELETAL_ANIMATION_SHADER);
   
    Shaders_SetModelMatrix(obj->bb.matrix);
    Shaders_UpdateModelMatrix();

    
    glUniform4fv(Shaders_GetBonesLocation(), obj->skeleton->nBones * 3, &obj->skeleton->matrices[0].x);


    glActiveTexture(GL_TEXTURE0);
    
    glBindVertexArray(obj->model->vao);

    int curr = 0;


    int k;
    for(k = 0; k < obj->model->nMaterials; k++){

        glBindTexture(GL_TEXTURE_2D, obj->model->materials[k].texture);
	    glUniform4fv(Shaders_GetDiffuseLocation(), 1, (float *)&obj->model->materials[k].diffuse);
	    glUniform4fv(Shaders_GetSpecularLocation(), 1, (float *)&obj->model->materials[k].specular);

        glDrawElements(GL_TRIANGLES, obj->model->nElements[k], GL_UNSIGNED_INT, (void *)(curr * sizeof(GLuint)));
        curr += obj->model->nElements[k];
    }

    glBindVertexArray(0);
}
static void DrawModel(Object *obj){

    Shaders_UseProgram(TEXTURED_SHADER);

    Shaders_SetModelMatrix(obj->bb.matrix);
    Shaders_UpdateModelMatrix();
    
    glActiveTexture(GL_TEXTURE0);

    glBindVertexArray(obj->model->vao);

    int curr = 0;


    int k;
    for(k = 0; k < obj->model->nMaterials; k++){
        glBindTexture(GL_TEXTURE_2D, obj->model->materials[k].texture);
	    glUniform4fv(Shaders_GetDiffuseLocation(), 1, (float *)&obj->model->materials[k].diffuse);
	    glUniform4fv(Shaders_GetSpecularLocation(), 1, (float *)&obj->model->materials[k].specular);
        glDrawElements(GL_TRIANGLES, obj->model->nElements[k], GL_UNSIGNED_INT, (void *)(curr * sizeof(GLuint)));
        curr += obj->model->nElements[k];
	}

    glBindVertexArray(0);
}
static char Draw(){


    float persp[16];

    Vec3 forward = Math_Rotate((Vec3){0,0,-1}, (Vec3){-rotation.y, -rotation.x, 0});
    float view[16];
    Math_LookAt(view, position, Math_Vec3AddVec3(position, forward), (Vec3){0,1,0});
    Shaders_SetViewMatrix(view);

    Math_Perspective(persp, 60.0f*(3.1415/180), (float)1920 / (float)1080, 0.1f, 50.0f);
    Shaders_SetProjectionMatrix(persp);

    Shaders_UseProgram(SKELETAL_ANIMATION_SHADER);
    Shaders_UpdateViewMatrix();
    Shaders_UpdateProjectionMatrix();
    Shaders_UseProgram(TEXTURED_SHADER);
    Shaders_UpdateViewMatrix();
    Shaders_UpdateProjectionMatrix();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glCullFace(GL_BACK);
    float idenity[16];
    Shaders_UseProgram(TEXTURED_SHADER);
    
    Math_Identity(idenity);
    Shaders_SetModelMatrix(idenity);
	int k;
	for(k = 0; k < cubeObj->skelBb.numChildren; k++){
		World_DrawSkeleton(&cubeObj->skelBb.children[k]);
	}
    World_Render(1);
    return 1;
}

static void OnResize(){
//         Thoth_Render(thoth); stencil buffer/framebuffer access todo

}
//


int main(int argc, char **argv){


    Window_Open("Editor", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,WINDOW_WIDTH, WINDOW_HEIGHT, 0);


    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    Memory_Init((0x01 << 20) * 64);

    glClearColor(0,0,0,1);
    Shaders_Init();

    World_InitOctree((Vec3){-100, -100, -100}, 200, 25);

    glClearColor(0,0,0,1);


    float persp[16], view[16], model[16];
    Math_Perspective(persp, 60.0f*(3.1415/180), (float)1920 / (float)1080, 0.1f, 100.0f);
    Math_LookAt(view, (Vec3){0,0,-5}, (Vec3){0,0,0}, (Vec3){0,1,0});
    Math_Identity(model);
    Shaders_UseProgram(SKELETAL_ANIMATION_SHADER);
    Shaders_SetProjectionMatrix(persp);
    Shaders_UpdateProjectionMatrix();
    Shaders_SetModelMatrix(model);
    Shaders_UpdateModelMatrix();
    Shaders_SetViewMatrix(view);
    Shaders_UpdateViewMatrix(view);
    Shaders_UpdateProjectionMatrix();
    memset(&cubeSkel, 0, sizeof(Skeleton));

    cubeObj = Object_Create();
	cubeObj->skeleton = &cubeSkel;
	memcpy(cubeObj->matrix, Math_Identity, sizeof(Math_Identity));
    RiggedModel_Load(&cubeModel, &cubeSkel, "Resources/figure.yuk");
    memset(&cubeAnim, 0, sizeof(Animation));
    Animation_Load(&cubeAnim, "Resources/figure_ArmatureAction.anm");

	Object_SetModel(cubeObj, &cubeModel);
    cubeObj->Draw = DrawRigged;
    cubeObj->AddUser(cubeObj);
    cubeObj->bb.pos.y = 1;
    cubeObj->bb.scale = (Vec3){0.1,0.1,0.1};
    cubeObj->bb.rot = (Vec3){0,0,0};
    World_UpdateObjectInOctree(cubeObj);

	groundObj = Object_Create();
    Model_Load(&groundModel, "Resources/ground.yuk");
    Model_LoadCollisions(&groundModel, "Resources/ground.col");
	Object_SetModel(groundObj, &groundModel);
    groundObj->Draw = DrawModel;
    groundObj->AddUser(groundObj);
    World_UpdateObjectInOctree(groundObj);
	throwObj = Object_Create();
    Model_Load(&throwModel, "Resources/cube.yuk");
	Object_SetModel(throwObj, &throwModel);
    throwObj->Draw = DrawModel;
	throwObj->bb.pos = position;
    throwObj->AddUser(throwObj);
    World_UpdateObjectInOctree(throwObj);

    cubeAnims[0] = (PlayingAnimation){
        .active = 1,
        .weight = 1,
        .into = 0,
        .anim = &cubeAnim,
    };

    // thoth = Thoth_Create(WINDOW_WIDTH, WINDOW_HEIGHT );
    // Thoth_LoadFile(thoth, "main.c");
    // Thoth_Resize(thoth, 50, 50, WINDOW_WIDTH, WINDOW_HEIGHT);

    Window_MainLoop(Update, Event, Draw, Focus, OnResize, 1, 1);


    // Thoth_Destroy(thoth);

	World_Free();
    Shaders_Close();
    ImageLoader_Free();


    return 0;
}