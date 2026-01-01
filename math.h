#ifndef MATH_DEF
#define MATH_DEF
#include <math.h>
#include <string.h>
#include <float.h>

#define SWAP(x, y, T) do { T SWAP = x; x = y; y = SWAP; } while(0)

#define EPSILON 0.000005
#define PI 3.14159265359

typedef struct {
	  float x;
	  float y;
} Vec2;

typedef struct {
	  float a;
	  float b;
	  float c;
	  float d;
} Plane;

typedef struct {
	  float x;
	  float y;
	  float z;
} Vec3;


typedef struct {
	  float x;
	  float y;
	  float z;
	  float w;
} Vec4;

typedef struct {
	  float x;
	  float y;
	  float z;
	  float w;
} Quat;

typedef struct {
	  Vec2 pos;
	  Vec2 line;
} Ray;

typedef struct {
	  float x;
	  float y;
	  float z;
	  float w;
	  float h;
} Rect;

typedef struct {
	  float x;
	  float y;
	  float z;
	  float w;
	  float h;
	  float d;
} Cube;

typedef struct {
	  float x;
	  float y;
	  float w;
	  float h;
} Rect2D;

typedef struct {
	  float mag;
	  Quat quat;
} Joint;

typedef struct {
	  Joint *joints;
	  int nJoints;
	  Vec3 basePos;
} Arm;

 char Math_Vec2EqualToVec2(Vec2 v1, Vec2 v2);
 Vec2 Math_Vec2MultFloat(Vec2 v1, float s);
 Vec2 Math_Vec2MultVec2(Vec2 v1, Vec2 v2);
 Vec2 Math_Vec2DivideFloat(Vec2 v1, float s);
 Vec2 Math_Vec2DivideVec2(Vec2 v1, Vec2 v2);
 Vec2 Math_Vec2AddFloat(Vec2 v1, float s);
 Vec2 Math_Vec2AddVec2(Vec2 v1, Vec2 v2);
 Vec2 Math_Vec2SubFloat(Vec2 v1, float s);
 Vec2 Math_Vec2SubVec2(Vec2 v1, Vec2 v2);
 float Math_Vec2Magnitude(Vec2 v) ;
 float Math_Vec2Dot(Vec2 v1, Vec2 v2);
 Vec2 Math_RoundVec2(Vec2 v);
 Vec2 Math_Vec2Reflect(Vec2 v, Vec2 l);
 Vec2 Math_Vec2Normalize(Vec2 v);
 char Math_Vec3EqualToVec3(Vec3 v1, Vec3 v2);
 Vec3 Math_Vec3MultFloat(Vec3 v1, float s);
 Vec3 Math_Vec3MultVec3(Vec3 v1, Vec3 v2);
 Vec3 Math_Vec3DivideFloat(Vec3 v1, float s);
 Vec3 Math_Vec3DivideVec3(Vec3 v1, Vec3 v2);
 Vec3 Math_Vec3AddFloat(Vec3 v1, float s);
 Vec3 Math_Vec3AddVec3(Vec3 v1, Vec3 v2);
 Vec3 Math_Vec3SubFloat(Vec3 v1, float s);
 Vec3 Math_Vec3SubVec3(Vec3 v1, Vec3 v2);
 float Math_Vec3Magnitude(Vec3 v) ;
 float Math_Vec3Dot(Vec3 v1, Vec3 v2);
 Vec3 Math_Vec3Cross(Vec3 v1, Vec3 v) ;
 Vec2 Math_Vec3ToVec2(Vec3 v);
 Vec3 Math_Vec2ToVec3(Vec2 v);
 Vec3 Math_RoundVec3(Vec3 v);
 Vec3 Math_Vec3Reflect(Vec3 v, Vec3 l);
 Vec3 Math_Vec3Normalize(Vec3 v);
 Vec3 Math_Vec4ToVec3(Vec4 v);
 Vec2 Math_Vec4ToVec2(Vec4 v);
 char Math_Vec3EqualToVec4(Vec4 v1, Vec4 v2);
 Vec4 Math_Vec4MultFloat(Vec4 v1, float s);
 Vec4 Math_Vec4MultVec4(Vec4 v1, Vec4 v2);
 Vec4 Math_Vec4DivideFloat(Vec4 v1, float s);
 Vec4 Math_Vec4DivideVec4(Vec4 v1, Vec4 v2);
 Vec4 Math_Vec4AddFloat(Vec4 v1, float s);
 Vec4 Math_Vec4AddVec4(Vec4 v1, Vec4 v2);
 Vec4 Math_Vec4SubFloat(Vec4 v1, float s);
 Vec4 Math_Vec4SubVec4(Vec4 v1, Vec4 v2);
 float Math_Vec4Magnitude(Vec4 v) ;
 Vec4 Math_RoundVec4(Vec4 v);
 Vec4 Math_Vec4Normalize(Vec4 v);
 Rect Math_Rect(Vec3 pos, float width, float height);
 Vec3 Math_RectXYZ(Rect r);
 Rect2D Math_RoundRect2D(Rect2D r);
 Rect Math_RoundRect(Rect r);
 char Math_CheckCollisionRect(Rect r1, Rect r);
	char Math_CheckCollisionRect2D(Rect2D r1, Rect2D r);
 Rect Math_Rect2DToRect(Rect2D r, float d);
 char Math_RectIsCompletelyInside(Rect r1, Rect r);
 Cube Math_Cube(Vec3 pos, float width, float height, float depth);
 Vec3 Math_CubeXYZ(Cube r);
 char Math_CheckCollisionCube(Cube r1, Cube r);
 char Math_CubeIsCompletelyInside(Cube r1, Cube r);
 float Math_DistanceToPlane(Plane p, Vec3 point) ;
 double Math_PlaneMagnitude(Plane p);
 void Math_PlaneNormalize(Plane *p);
int Math_CheckFrustumCollision(Cube r, Plane *frustumPlanes);

float Math_GetDistanceFloat(float min1, float max1, float min2, float max2);
Vec2 Math_GetDistanceRect(Rect r1, Rect r2);
Vec3 Math_GetDistanceCube(Cube r1, Cube r2);
int Math_GetDistanceInt(int min1, int max1, int min2, int max2);
float Math_RectCheckCollisionRay(Rect r, Ray ray);
float Math_CubeCheckCollisionRay(Cube r, Ray ray);

void Math_Ik_AddJoint(Arm *arm, float mag, Vec3 rotation);
float Math_Ik_ArmLength(Arm arm);
void Math_Ik_Solve(Arm *arm, Vec3 endPos);
Vec3 Math_LerpVec3(Vec3 a1, Vec3 a2, float t);
float Math_Lerp(float a1, float a2, float t);
float Math_DistanceToLine(Vec2 n, Vec2 o, Vec2 p);
char Math_QuatEqualToQuat(Quat q1, Quat q2);
Quat Math_QuatMult(Quat q1, Quat q2);
Vec3 Math_QuatRotate(Quat q, Vec3 v);
Quat Math_Quat(Vec3 v, float a);
Quat Math_QuatConj(Quat q);
float Math_QuatMag(Quat q);
Quat Math_QuatInv(Quat q);
Quat Math_QuatNormalize(Quat q);
Quat Math_QuatLookAt(Vec3 forward, Vec3 up);
Vec3 Math_RotateMatrixToEuler(float *m);
float Math_Clamp(float m, float min, float max);
void Math_InverseMatrixNxN(float *mat, int n);
float Math_Determinant(float *mat, int n);
void Math_Mat4ToMat3(float *mat4, float *mat3);
void Math_InverseMatrixMat3(float *mat3);
void Math_SkewSymetricMat3(Vec3 v, float *matrix);

Vec3 Math_MatrixMultMat3(Vec3 vec, float *matrix);
Vec3 Math_MatrixMult(Vec3,float*);
Vec4 Math_MatrixMult4(Vec4,float*);

void Math_Matrix6x6Set(float *m, float *m1, float *m2, float *m3, float *m4);
void Math_TransposeMatrix(float *matrix, int n);
Quat Math_MatrixToQuat(float *matrix);
void Math_OuterProduct(Vec3 vec, Vec3 trans, float *matrix);
void Math_RotateMatrix(float *matrix, Vec3 angles);
void Math_Perspective( float *matrix, float fov, float a, float n, float f);
void Math_Perspective2(float *matrix, float l, float r, float t, float b, float n, float f);
void Math_Ortho(float *matrix, float l, float r, float t, float b, float n, float f);
void Math_MatrixMatrixMult(float *res, float *a, float *b);
void Math_LookAt(float *ret, Vec3 eye, Vec3 center, Vec3 up );
void Math_RotateAroundAxis(Vec3 p, float a, float *);
void Math_MatrixFromQuat(Quat q, float*);
Quat Math_Slerp(Quat q, Quat q2, float);
void Math_CopyMatrix(float *m1, float *m2);
void Math_InverseMatrix(float *m);
void Math_ScaleMatrix(float *matrix, int n, float amount);
void Math_ScalingMatrixXYZ(float *matrix, Vec3 amount);
Vec3 Math_Rotate(Vec3 pos, Vec3 angles);
void Math_TranslateMatrix(float *matrix, Vec3 vector);
void Math_ScalingMatrix(float *matrix, float amount);

void Math_MatrixMatrixAdd(float *matrix, float *m0, float *m1);
void Math_MatrixMatrixSub(float *matrix, float *m0, float *m1);
Vec3 Math_QuatToAxisAngle(Quat quat ,float *angle);
Vec3 Math_AxisAngleToEuler(Vec3 axis, float angle) ;
Vec3 Math_QuatToEuler(Quat quat);
Quat Math_EulerToQuat(const Vec3 euler);
void Math_Identity(float *matrix);
// Vec3 Math_RotateMatrixToEuler(float *m);
Vec3 Math_Ik_GetJointPos(Arm *arm, int jointIndex);

#endif