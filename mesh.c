#include <GL/glew.h>
#include <string.h>
#include "deflate.h"
#include "log.h"
#include "mesh.h"
#include "shaders.h"
#include "math.h"

static void GetAnimsKeyframe(Bone *bone, PlayingAnimation *pAnim, Keyframe **next, Keyframe **prev){

    Animation *anim = pAnim->anim;

    int f;
    for(f = 0; f < anim->nKeyframes[bone->index]; f++){

        if(anim->keyframes[bone->index][f].frame <= pAnim->into){

            *prev = &anim->keyframes[bone->index][f];

        } else {

            *next = &anim->keyframes[bone->index][f];
            break;
        }
    }
}

static Quat GetAnimsBoneRot(Bone *bone, PlayingAnimation *pAnim){

    Keyframe *next = NULL;
    Keyframe *prev = NULL;

    GetAnimsKeyframe(bone, pAnim, &next, &prev);

    if(!prev)
        return pAnim->anim->keyframes[bone->index][0].rot;

    if(!next)
        return prev->rot;

    float slerp = (pAnim->into - prev->frame) / (next->frame - prev->frame);

    return Math_Slerp(prev->rot, next->rot, slerp);
}

static Vec3 GetAnimsBonePos(Bone *bone, PlayingAnimation *pAnim){

    Keyframe *next = NULL;
    Keyframe *prev = NULL;

    GetAnimsKeyframe(bone, pAnim, &next, &prev);

    if(!prev)
        return pAnim->anim->keyframes[bone->index][0].pos;

    if(!next)
        return prev->pos;

    float lerp = (pAnim->into - prev->frame) / (next->frame - prev->frame);

    return Math_LerpVec3(prev->pos, next->pos, lerp);
}


// i store the displacement for each bone as a quaternion and then i have a recursive function that goes through each 
// bone calculating the force to apply to the angular velocity which is converted to a quaternion and multiplied by 
// the current displacement and set to the current displacement.

// and at the same time that angular velocity quaternions inverse is multiplied by each child bones displacement and set to it. for example
// bone->children[j]->displacement = QuatMult(bone->children[j]->displacement, QuatInv(quat));

// (quat here is calculated from the bones angular velocity)

// then each of the bones children is iterated over and the recursion occurs there.

// to get the force from the displacement i rotate a vector of (0,1,0) by the displacement and then take the cross product
// of that and (0,1,0) and use that as the displacement vector in the standard linear spring damper equation.
// since the cross product of the rotated up and the up returns the vector orthogonal to both multiplied by the sine of the
// angle between them it seems to work, so yeah that's why you don't normalize it.

// not sure if this is completely accurate though but it seems to be working.

static void UpdateBoneSpring(Bone *bone, Quat displacement){

    float dt = GetDeltaTime();

    if(bone->spring != 0 && dt > 0 ){

        Vec3 curr = Math_QuatRotate(displacement, (Vec3){0,1,0});

        if(Math_Vec3Dot(curr, (Vec3){0,1,0}) == 1)
            return;

        Vec3 sinaxis = Math_Vec3Cross(curr, (Vec3){0,1,0});

        Vec3 force = Math_Vec3MultFloat(sinaxis, -bone->spring);

        force = Math_Vec3AddVec3(force, Math_Vec3MultFloat(bone->angVel, -bone->damping));

        force = Math_Vec3MultFloat(force, dt);

        bone->angVel = Math_Vec3AddVec3(bone->angVel, force);

        Vec3 vec = Math_Vec3Normalize(bone->angVel);

        float angle = -dt * Math_Vec3Magnitude(bone->angVel);

        Quat quat = Math_Quat(vec, angle);

        bone->rotDisplacement = Math_QuatNormalize(Math_QuatMult(bone->rotDisplacement, quat));

        int j;
        for(j = 0; j < bone->nChildren; j++)
            bone->children[j]->rotDisplacement = Math_QuatMult(bone->children[j]->rotDisplacement, Math_QuatInv(quat));
    }
}

static void BoneUpdateSprings(Bone *bone, Vec4 *matrices){

    UpdateBoneSpring(bone, bone->rotDisplacement);

    Vec3 pos = bone->pos;
    Quat rot = Math_QuatMult(bone->rot, bone->rotDisplacement);

    float matrix[16];

    Math_TranslateMatrix(matrix, pos);
    Math_MatrixFromQuat(rot, bone->absMatrix);
    Math_MatrixMatrixMult(bone->absMatrix, matrix, bone->absMatrix);

    if(bone->parent)
        Math_MatrixMatrixMult(bone->absMatrix, bone->parent->absMatrix, bone->absMatrix); 

    Math_MatrixMatrixMult(matrix, bone->absMatrix, bone->invBindMatrix);

    // matrices[(bone->index*3)].x = matrix[0];
    // matrices[(bone->index*3)].y = matrix[1];
    // matrices[(bone->index*3)].z = matrix[2];
    // matrices[(bone->index*3)].w = matrix[3];

    // matrices[(bone->index*3)+1].x = matrix[4];
    // matrices[(bone->index*3)+1].y = matrix[5];
    // matrices[(bone->index*3)+1].z = matrix[6];
    // matrices[(bone->index*3)+1].w = matrix[7];

    // matrices[(bone->index*3)+2].x = matrix[8];
    // matrices[(bone->index*3)+2].y = matrix[9];
    // matrices[(bone->index*3)+2].z = matrix[10];
    // matrices[(bone->index*3)+2].w = matrix[11];

    int j;
    for(j = 0; j < bone->nChildren; j++)
        BoneUpdateSprings(bone->children[j], matrices);
}

static void BoneUpdate(Bone *bone, PlayingAnimation *anims, int nAnims, Vec4 *matrices){

    Quat rot = (Quat){0,0,0,1};
    Vec3 pos = (Vec3){0,0,0};

    int j;
    float matrix[16];

    for(j = 0; j < nAnims; j++){
        if(anims[j].anim->nKeyframes[bone->index]){
            rot = Math_Slerp(rot, GetAnimsBoneRot(bone, &anims[j]), anims[j].weight);
            pos = Math_LerpVec3(pos, GetAnimsBonePos(bone, &anims[j]), anims[j].weight);
        }
    }

    if(bone->spring != 0){

        if(bone->parent && bone->parent->spring == 0)
            UpdateBoneSpring(bone, Math_QuatMult(bone->parent->rotDisplacement, Math_QuatMult(bone->rotDisplacement, rot)));
        else
            UpdateBoneSpring(bone, Math_QuatMult(bone->rotDisplacement, rot));

        rot = Math_QuatMult(bone->rot, bone->rotDisplacement);
    
    } else {

        // accumulate rotation in rotDisplacement

        bone->rotDisplacement = Math_QuatMult(bone->rotDisplacement, rot);
        
        for(j = 0; j < bone->nChildren; j++){
            if(bone->children[j]->spring == 0)
                bone->children[j]->rotDisplacement = bone->rotDisplacement;
        }


        rot = Math_QuatMult(bone->rot, rot);
    }

    pos = Math_Vec3AddVec3(bone->pos, Math_QuatRotate(bone->rot, pos));

    Math_TranslateMatrix(matrix, pos);
    Math_MatrixFromQuat(rot, bone->absMatrix);

    Math_MatrixMatrixMult(bone->absMatrix, matrix, bone->absMatrix);
    if(bone->parent){
        Math_MatrixMatrixMult(bone->absMatrix, bone->parent->absMatrix, bone->absMatrix); 
	}

    Math_MatrixMatrixMult(matrix, bone->absMatrix, bone->invBindMatrix);

    bone->axes[0] = (Vec3){1,0,0};
    bone->axes[1] = (Vec3){0,1,0};
    bone->axes[2] = (Vec3){0,0,1};
    bone->axes[0] = Math_Vec3Normalize(Math_MatrixMult(bone->axes[0], bone->absMatrix));
    bone->axes[1] = Math_Vec3Normalize(Math_MatrixMult(bone->axes[1], bone->absMatrix));
    bone->axes[2] = Math_Vec3Normalize(Math_MatrixMult(bone->axes[2], bone->absMatrix));

    bone->points[0] = (Vec3){bone->cube.x, bone->cube.y, bone->cube.z};
    bone->points[1] = (Vec3){bone->cube.x+bone->cube.w, bone->cube.y, bone->cube.z};
    bone->points[2] = (Vec3){bone->cube.x+bone->cube.w, bone->cube.y+bone->cube.h, bone->cube.z};
    bone->points[3] = (Vec3){bone->cube.x, bone->cube.y+bone->cube.h, bone->cube.z};
    bone->points[4] = (Vec3){bone->cube.x, bone->cube.y, bone->cube.z+bone->cube.d};
    bone->points[5] = (Vec3){bone->cube.x+bone->cube.w, bone->cube.y, bone->cube.z+bone->cube.d};
    bone->points[6] = (Vec3){bone->cube.x+bone->cube.w, bone->cube.y+bone->cube.h, bone->cube.z+bone->cube.d};
    bone->points[7] = (Vec3){bone->cube.x, bone->cube.y+bone->cube.h, bone->cube.z+bone->cube.d};

    bone->points[0] = Math_MatrixMult(bone->points[0], bone->absMatrix);
    bone->points[1] = Math_MatrixMult(bone->points[1], bone->absMatrix);
    bone->points[2] = Math_MatrixMult(bone->points[2], bone->absMatrix);
    bone->points[3] = Math_MatrixMult(bone->points[3], bone->absMatrix);
    bone->points[4] = Math_MatrixMult(bone->points[4], bone->absMatrix);
    bone->points[5] = Math_MatrixMult(bone->points[5], bone->absMatrix);
    bone->points[6] = Math_MatrixMult(bone->points[6], bone->absMatrix);
    bone->points[7] = Math_MatrixMult(bone->points[7], bone->absMatrix);
    
    matrices[(bone->index*3)].x = matrix[0];
    matrices[(bone->index*3)].y = matrix[1];
    matrices[(bone->index*3)].z = matrix[2];
    matrices[(bone->index*3)].w = matrix[3];

    matrices[(bone->index*3)+1].x = matrix[4];
    matrices[(bone->index*3)+1].y = matrix[5];
    matrices[(bone->index*3)+1].z = matrix[6];
    matrices[(bone->index*3)+1].w = matrix[7];

    matrices[(bone->index*3)+2].x = matrix[8];
    matrices[(bone->index*3)+2].y = matrix[9];
    matrices[(bone->index*3)+2].z = matrix[10];
    matrices[(bone->index*3)+2].w = matrix[11];

    for(j = 0; j < bone->nChildren; j++)
        BoneUpdate(bone->children[j], anims, nAnims, matrices);


}

static void NormalizeAnimWeights(PlayingAnimation *anims, int nAnims){

    float mag = 0;

    int k;
    for(k = 0; k < nAnims; k++)
        mag += anims[k].weight * anims[k].weight; 

    mag = sqrt(mag);

    for(k = 0; k < nAnims; k++)
        anims[k].weight /= mag;     
}

void Skeleton_BlendAnims(PlayingAnimation *anims, int nAnims, float dt){

    int k;
    for(k = 0; k < nAnims; k++){

        if(anims[k].active)
            anims[k].weight = MIN(anims[k].weight + (anims[k].weightSpeed * dt), 1);            
        else
            anims[k].weight = MAX(anims[k].weight - (anims[k].weightSpeed * dt), 0);            
    }

}

void Skeleton_Update(Skeleton *skeleton, PlayingAnimation *anims, int nAnims){

    if(nAnims > 1)
        NormalizeAnimWeights(anims, nAnims);

    BoneUpdate(skeleton->root, anims, nAnims, skeleton->matrices);

}

void Skeleton_UpdateSprings(Skeleton *skeleton){

    BoneUpdateSprings(skeleton->root, skeleton->matrices);
}

static void LoadModel(Model *model, FILE *fp, u16 stride){

    fread(&model->nMaterials, 1, sizeof(int), fp);

    if(model->nMaterials >= MAX_MODEL_MATERIALS){

        LOG(LOG_RED, "FATAL ERROR: NUM MATERIALS IN MODEL ARE MORE THAN MAX ALLOWED MATERIALS\n");
    }

    int k;
    for(k = 0; k < model->nMaterials; k++){
        fread(&model->materials[k].texture, 1, sizeof(int), fp); // store tex index for tmp use
        fread(&model->materials[k].normalTexture, 1, sizeof(int), fp); // store tex index for tmp use
        fread(&model->materials[k].diffuse, 1, sizeof(Vec4), fp);
        fread(&model->materials[k].specular, 1, sizeof(Vec4), fp);
    }

    u32 textures[MAX_MODEL_MATERIALS];

    int nTextures = 0;
    fread(&nTextures, 1, sizeof(int), fp);
    for(k = 0; k < nTextures; k++){
    
        int w, h, channels;
        fread(&w, 1, sizeof(int), fp);
        fread(&h, 1, sizeof(int), fp);
        fread(&channels, 1, sizeof(int), fp);

        glGenTextures(1, &textures[k]);
        glBindTexture(GL_TEXTURE_2D, textures[k]);

        int size = w * h * channels;

        u8 *data = (u8 *)Memory_StackAlloc(TEMP_STACK, size);
        // Deflate_Read(fp, data, size);
        fread(data, sizeof(u8), size, fp);

        if(channels == 3) channels = GL_RGB;
        else if(channels == 4) channels = GL_RGBA;
        else channels = GL_RED;

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, channels, GL_UNSIGNED_BYTE, data);
        Memory_StackPop(TEMP_STACK, 1);

        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);//GL_NEAREST_MIPMAP_NEAREST));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);//GL_NEAREST_MIPMAP_NEAREST));
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    for(k = 0; k < model->nMaterials; k++){

        if(model->materials[k].texture > 0 && model->materials[k].texture < MAX_MODEL_MATERIALS){
            ++model->nTextures;
            model->materials[k].texture = textures[model->materials[k].texture-1];
        }

        if(model->materials[k].normalTexture > 0 && model->materials[k].normalTexture < MAX_MODEL_MATERIALS){
            ++model->nNormalTextures;
            model->materials[k].normalTexture = textures[model->materials[k].normalTexture-1];
        }
    }

    int nVerts = 0;
    fread(&nVerts, 1, sizeof(int), fp);

    int size = stride * nVerts;

    u8 *vboData = (u8 *)Memory_StackAlloc(TEMP_STACK, size);

    // Deflate_Read(fp, vboData, size);
    
    fread(vboData, 1, size, fp);

	model->bb[0].cube.x = model->bb[0].cube.y = model->bb[0].cube.z = HUGE_VAL;
	model->bb[0].cube.w = model->bb[0].cube.h = model->bb[0].cube.d = -HUGE_VAL;

    for(k = 0; k < nVerts; k++){
        Vec3 *pos = (Vec3 *)&vboData[(stride * k)];
        if(pos->x < model->bb[0].cube.x)
            model->bb[0].cube.x = pos->x;
        if(pos->x > model->bb[0].cube.w)
            model->bb[0].cube.w = pos->x;
        if(pos->y < model->bb[0].cube.y)
            model->bb[0].cube.y = pos->y;
        if(pos->y > model->bb[0].cube.h)
            model->bb[0].cube.h = pos->y;
        if(pos->z < model->bb[0].cube.z)
            model->bb[0].cube.z = pos->z;
        if(pos->z >  model->bb[0].cube.d)
            model->bb[0].cube.d = pos->y;
    }
	model->numBB = 1;
    model->bb[0].cube.w -= model->bb[0].cube.x;
    model->bb[0].cube.h -= model->bb[0].cube.y;
    model->bb[0].cube.d -= model->bb[0].cube.z;

	model->bb[0].scale = (Vec3){1,1,1};
	model->bb[0].pos = (Vec3){0,0,0};
	model->bb[0].rot = (Vec3){0,0,0};

    glBindBuffer(GL_ARRAY_BUFFER, model->vbo);
    glBufferData(GL_ARRAY_BUFFER, size, vboData, GL_STATIC_DRAW);

    Memory_StackPop(TEMP_STACK, 1);

    int totalElements = 0;

    for(k = 0; k < model->nMaterials; k++){

        fread(&model->nElements[k], 1, sizeof(int), fp);

        totalElements += model->nElements[k];

        // Deflate_Read(fp, model->elements[k], sizeof(u16) * model->nElements[k]);
    }

    u32 *elements = (u32 *)Memory_StackAlloc(TEMP_STACK, sizeof(u32) * totalElements);

    fread(elements, totalElements, sizeof(u32), fp);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model->ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, totalElements * sizeof(u32), elements, GL_STATIC_DRAW);


    Memory_StackPop(TEMP_STACK, 1);

}

void Model_Load(Model *model, const char *path){

    // u16 stride = sizeof(Vec2) + (sizeof(Vec3) * 4);
    // u16 stride = sizeof(Vec2) + (sizeof(Vec3) * 3);
    u16 stride = sizeof(Vec2) + (sizeof(Vec3) * 2) + sizeof(Vec4);

    glGenVertexArrays(1, &model->vao);
    glBindVertexArray(model->vao);

    glGenBuffers(1, &model->ebo);
    glGenBuffers(1, &model->vbo);
    glBindBuffer(GL_ARRAY_BUFFER, model->vbo);

    GLuint positionAttribute = glGetAttribLocation(Shaders_GetProgram(TEXTURED_SHADER), SHADERS_POSITION_ATTRIB);
    GLuint uvAttribute = glGetAttribLocation(Shaders_GetProgram(TEXTURED_SHADER), SHADERS_COORD_ATTRIB);
    GLuint normAttribute = glGetAttribLocation(Shaders_GetProgram(TEXTURED_SHADER), SHADERS_NORM_ATTRIB);
    GLuint tangentAttribute = glGetAttribLocation(Shaders_GetProgram(TEXTURED_SHADER), SHADERS_TAN_ATTRIB);

    glEnableVertexAttribArray(positionAttribute);
    glEnableVertexAttribArray(uvAttribute);
    glEnableVertexAttribArray(normAttribute);
    glEnableVertexAttribArray(tangentAttribute);

    glVertexAttribPointer(positionAttribute, 3, GL_FLOAT, GL_FALSE, stride, 0);

    void *offset = (void *)sizeof(Vec3);

    glVertexAttribPointer(uvAttribute, 2, GL_FLOAT, GL_FALSE, stride, (void*)offset);

    offset += sizeof(Vec2);

    glVertexAttribPointer(normAttribute, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);

    offset += sizeof(Vec3);

    glVertexAttribPointer(tangentAttribute, 4, GL_FLOAT, GL_FALSE, stride, (void*)offset);
    
    // offset += sizeof(Vec4);

 //    glEnableVertexAttribArray(TANGENT_LOC);
 //    glVertexAttribPointer(TANGENT_LOC, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);
    
    // offset += sizeof(Vec3);

    // glEnableVertexAttribArray(BITANGENT_LOC);
    // glVertexAttribPointer(BITANGENT_LOC, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);
    
    // offset += sizeof(Vec3);


    FILE *fp = fopen(path, "rb");

    LoadModel(model, fp, stride);

    fclose(fp);

    glBindVertexArray(0);
}

void Model_DeleteTextures(Model *model){
    
    int k;
    for(k = 0; k < model->nMaterials; k++)
        glDeleteTextures(1, &model->materials[k].texture);
}

void Model_Free(Model *model){

    Model_DeleteTextures(model);

    glDeleteVertexArrays(1, &model->vao);
    glDeleteBuffers(1, &model->vbo);
    glDeleteBuffers(1, &model->ebo);
}

static int LoadAnimation(Animation *anim, FILE *fp){

    int nBones;
    fread(&nBones, 1, sizeof(int), fp);

    memset(anim, 0, sizeof(Animation));

    int nAllocations = 0;

    int k;
    for(k = 0; k < nBones; k++){

        fread(&anim->nKeyframes[k], 1, sizeof(int), fp);

        if(!anim->nKeyframes[k]) continue;
    
        ++nAllocations;

        anim->keyframes[k] = Memory_StackAlloc(MAIN_STACK, sizeof(Keyframe) * anim->nKeyframes[k]);

        memset(anim->keyframes[k], 0, sizeof(Keyframe) * anim->nKeyframes[k]);

        int j;
        for(j = 0; j < anim->nKeyframes[k]; j++){
    
            fread(&anim->keyframes[k][j].frame, 1, sizeof(int), fp);
            anim->keyframes[k][j].boneIndex = k;
            fread(&anim->keyframes[k][j].pos, 1, sizeof(Vec3), fp);
            fread(&anim->keyframes[k][j].rot, 1, sizeof(Quat), fp);


            if(anim->keyframes[k][j].frame > anim->length)
                anim->length = anim->keyframes[k][j].frame;

        }
    }

    return nAllocations;
}

void Animation_Free(Animation anim){

    int nAllocations = 0;

    int k;
    for(k = 0; k < MAX_BONES; k++)
        if(anim.keyframes[k])
            ++nAllocations;

    Memory_StackPop(MAIN_STACK, nAllocations);
}

static void InitBone(Skeleton *skeleton, Bone *bone){

    static float matrix[16];
    Math_TranslateMatrix(matrix, bone->pos);
    Math_MatrixFromQuat(bone->rot, bone->absMatrix);
    Math_MatrixMatrixMult(bone->absMatrix, matrix, bone->absMatrix);

    // bone->modelPos = bone->rest;

    if(bone->parent){
        Math_MatrixMatrixMult(bone->absMatrix, bone->parent->absMatrix, bone->absMatrix); 
        // bone->modelPos = Math_Vec3AddVec3(bone->parent->modelPos, bone->modelPos);
    }

    memcpy(bone->invBindMatrix, bone->absMatrix, sizeof(float) * 16);
    Math_InverseMatrix(bone->invBindMatrix);

    bone->rotDisplacement = (Quat){0,0,0,1};

    // bone->modelPos.x = bone->absMatrix[3];
    // bone->modelPos.y = bone->absMatrix[7];
    // bone->modelPos.z = bone->absMatrix[11];

    skeleton->matrices[(bone->index*3)].x = 1;
    skeleton->matrices[(bone->index*3)].y = 0;
    skeleton->matrices[(bone->index*3)].z = 0;
    skeleton->matrices[(bone->index*3)].w = 0;
    skeleton->matrices[(bone->index*3)+1].x = 0;
    skeleton->matrices[(bone->index*3)+1].y = 1;
    skeleton->matrices[(bone->index*3)+1].z = 0;
    skeleton->matrices[(bone->index*3)+1].w = 0;
    skeleton->matrices[(bone->index*3)+2].x = 0;
    skeleton->matrices[(bone->index*3)+2].y = 0;
    skeleton->matrices[(bone->index*3)+2].z = 1;
    skeleton->matrices[(bone->index*3)+2].w = 0;

    int k;
    for(k = 0; k < bone->nChildren; k++)
        InitBone(skeleton, bone->children[k]);
}

static void LoadSkeleton(Skeleton *skeleton, FILE *fp){

    memset(skeleton, 0, sizeof(Skeleton));

    fread(&skeleton->nBones, 1, sizeof(int), fp);

    int k;
    for(k = 0; k < skeleton->nBones; k++){

        int index;
        int parentIndex;

        fread(&parentIndex, 1, sizeof(int), fp);
        fread(&index, 1, sizeof(int), fp);

        Bone *bone = &skeleton->bones[index];

        bone->index = index;

        fread(&bone->pos, 1, sizeof(Vec3), fp);
        fread(&bone->rot, 1, sizeof(Quat), fp);
        fread(&bone->cube, 1, sizeof(Cube), fp);
        if(parentIndex >= 0){

            bone->parent = &skeleton->bones[parentIndex];


            if(bone->parent->nChildren < BONE_MAX_CHILDREN)
                bone->parent->children[bone->parent->nChildren++] = bone;
        
        } else {

            skeleton->root = bone;
        }
    }

    InitBone(skeleton, skeleton->root);
}

void Animation_Load(Animation *animation, const char *path){

    FILE *fp = fopen(path, "rb");

    LoadAnimation(animation, fp);

    fclose(fp);
}

void Model_LoadCollisions(Model *model, const char *path){

    FILE *fp = fopen(path, "rb");
    if(!fp) return;

    fread(&model->numBB, 1, sizeof(int), fp);
    int k;
    for (k = 0; k < model->numBB; k++){
        fread(&model->bb[k].cube, 1, sizeof(Cube), fp);
		fread(&model->bb[k].pos, 1, sizeof(Vec3), fp);
        Quat rot;
		fread(&rot, 1, sizeof(Quat), fp);
		model->bb[k].rot = Math_QuatToEuler(rot);
        fread(&model->bb[k].scale, 1, sizeof(Vec3), fp);
	}


    fclose(fp);
}

void RiggedModel_Load(Model *model, Skeleton *skeleton, const char *path){

    memset(model, 0, sizeof(Model));

    // u16 stride = sizeof(Vec2) + (sizeof(Vec3) * 4) + (sizeof(Vec4) * 2);
    // u16 stride = sizeof(Vec2) + (sizeof(Vec3) * 3) + (sizeof(Vec4) * 2);
    u16 stride = sizeof(Vec2) + (sizeof(Vec3) * 2) + (sizeof(Vec4) * 3);

    glGenVertexArrays(1, &model->vao);
    glBindVertexArray(model->vao);

    glGenBuffers(1, &model->ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model->ebo);
    
    glGenBuffers(1, &model->vbo);
    glBindBuffer(GL_ARRAY_BUFFER, model->vbo);

    Shaders_UseProgram(SKELETAL_ANIMATION_SHADER);

    GLuint positionAttribute = glGetAttribLocation(Shaders_GetProgram(SKELETAL_ANIMATION_SHADER), SHADERS_POSITION_ATTRIB);
    GLuint uvAttribute = glGetAttribLocation(Shaders_GetProgram(SKELETAL_ANIMATION_SHADER), SHADERS_COORD_ATTRIB);
    GLuint normAttribute = glGetAttribLocation(Shaders_GetProgram(SKELETAL_ANIMATION_SHADER), SHADERS_NORM_ATTRIB);
    GLuint tangentAttribute = glGetAttribLocation(Shaders_GetProgram(SKELETAL_ANIMATION_SHADER), SHADERS_TAN_ATTRIB);
    GLuint weightsAttribute = glGetAttribLocation(Shaders_GetProgram(SKELETAL_ANIMATION_SHADER), SHADERS_WEIGHT_ATTRIB);
    GLuint boneIndicesAttrib = glGetAttribLocation(Shaders_GetProgram(SKELETAL_ANIMATION_SHADER), SHADERS_BONES_ATTRIB);

    glEnableVertexAttribArray(positionAttribute);
    glVertexAttribPointer(positionAttribute, 3, GL_FLOAT, GL_FALSE, stride, 0);

    void *offset = (void *)sizeof(Vec3);

    glEnableVertexAttribArray(uvAttribute);
    glVertexAttribPointer(uvAttribute, 2, GL_FLOAT, GL_FALSE, stride, (void*)offset);
    
    offset += sizeof(Vec2);

    glEnableVertexAttribArray(normAttribute);
    glVertexAttribPointer(normAttribute, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);
    
    offset += sizeof(Vec3);

    glEnableVertexAttribArray(tangentAttribute);
    glVertexAttribPointer(tangentAttribute, 4, GL_FLOAT, GL_FALSE, stride, (void*)offset);
    
    offset += sizeof(Vec4);

    // glEnableVertexAttribArray(TANGENT_LOC);
    // glVertexAttribPointer(TANGENT_LOC, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);
    
    // offset += sizeof(Vec3);

 //    glEnableVertexAttribArray(BITANGENT_LOC);
 //    glVertexAttribPointer(BITANGENT_LOC, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);
    
    // offset += sizeof(Vec3);

    glEnableVertexAttribArray(weightsAttribute);
    glVertexAttribPointer(weightsAttribute, 4, GL_FLOAT, GL_FALSE, stride, (void*)offset);
    
    offset += sizeof(Vec4);

    glEnableVertexAttribArray(boneIndicesAttrib);
    glVertexAttribPointer(boneIndicesAttrib, 4, GL_FLOAT, GL_FALSE, stride, (void*)offset);

    FILE *fp = fopen(path, "rb");

    LoadModel(model, fp, stride);

    LoadSkeleton(skeleton, fp);

    fclose(fp);

    glBindVertexArray(0);
}

void RiggedModel_Free(Model *model){

    Model_Free(model);
}