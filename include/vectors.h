#ifndef vectors_h__
#define vectors_h__

typedef struct //Матрица связанного 3х-гранника
{
	float X;
	float Y;
	float Z;
} vec_body;
typedef struct //Матрица географического 3х-гранника
{
	float E;
	float N;
	float U;
} vec_enu;

#endif
