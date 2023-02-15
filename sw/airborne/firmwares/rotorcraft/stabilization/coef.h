/*Copyright (C) Florian Sansou <florian.sansou@enac.fr> 
 This file is directly generated with MATLAB to obtain the coefficient set of the looping
*/

#define CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE  2
#define CTRL_HOVER_WIND_INPUT 11
#define CTRL_HOVER_WIND_NUM_ACT 4

const float kf = 0.000000014047000;
const float mot_max_speed = 15769.000000000000000;

const float ueq[CTRL_HOVER_WIND_NUM_ACT][1] = {{2.4665},{2.4665},{0},{0}};

const float H[CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE][CTRL_HOVER_WIND_INPUT] = {{-0.015404,-0.11184,-0.0032262,-0.0161,-0.31149,-0.99772,0.14132,-0.50102,0.12626,-0.21741,-0.18422}, 
{-0.21989,0.07656,-0.1352,0.51855,-0.092622,-0.27788,-0.036852,-0.16495,0.11405,0.81657,-0.3761}};

const float K[CTRL_HOVER_WIND_NUM_ACT][CTRL_HOVER_WIND_INPUT] = {{0.220642,0.121133,0.205437,0.839664,7.50467,18.0691,-0.441994,14.0305,-0.213024,1.02391,6.50987}, 
{0.0972424,0.0989496,-0.0236427,-0.575826,-3.0524,7.89991,-5.15129,-1.94741,-0.292523,-1.84963,-5.85729}, 
{-0.0942055,-0.0188204,0.200939,-11.4311,-2.9841,0.223723,-2.5935,-0.53974,1.73211,10.5932,0.065539}, 
{0.203738,-0.0804832,-0.125676,-10.3988,-1.2274,-0.520814,-1.39113,2.97358,-1.37951,10.6895,1.65469}};

const float num[3] = {-0.003038217066126,-0.000013475188059,0.003024741878067};

const float den[3] = {1.000000000000000,-1.874295341777843,0.874295341803071};

