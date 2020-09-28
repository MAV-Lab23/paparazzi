#include "std.h"
#include "stdio.h"
#include "filter.h"
#include "state.h"
#include "flightplan_Bang.h"
#include "subsystems/datalink/telemetry.h"
#include "bangbang.h"

#define LOG
struct bangbang_fp_struct dr_bang;
int next_gate_nr;
int timer1=0;
int angle_index=0;
#define d2r M_PI/180.0f
#define NR_ANGLEVAR 16 //16
// #define ANGLEOVERWRITE
// float angle_variations[NR_ANGLEVAR]={5, 5,5, 5, 10, 10,10,10, 15,15,15,15, 20, 20,20,20};
//  float angle_variations[NR_ANGLEVAR]={25,25,25,25, 30,30,30,30 ,35,35,35,35, 40,40,40,40};
 float angle_variations[NR_ANGLEVAR]={5,5,10,10, 15,15,20,20 ,25,25,30,30, 35,35,40,40};
int target_reached = 0;
// int gate_nr;float gate_x;float gate_y;float gate_z;float gate_psi;float gate_speed;int gate_type;int controller_type;int turning;float psi_forced; bool overwrite_psi, float satangle;


//------- Flightplan Structures------------------------------------

// Demo Forward
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,0,-1.5,M_PI,0.2,STARTGATE,BANGBANG,0,0,false,35},
// {1, 2.5,0,-1.5,0,0.2,ENDGATE,BANGBANG,0,0,false,35},
// };

// Demo Angle variations forward
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,0,-1.5,M_PI,0.2,STARTGATE,BANGBANG,0,0,false,30},
// {1, 2.5,0,-1.5,0,0.2,GATE,BANGBANG,0,0,false,30},
// {2, 0.25,0,-1.5,M_PI,0.2,GATE,BANGBANG,0,0,false,30},
// {3, -2,0,-1.5,M_PI,0.2,GATE,BANGBANG,0,0,false,30},
// {4, 2.5,0,-1.5,0,0.2,ENDGATE,BANGBANG,0,0,false,30},
// };

// Demo Angle variations sideways
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -1.5,0,-1.5,-0.5*M_PI,0.2,STARTGATE,BANGBANG,0,-0.5*M_PI,true,30},   
// {1, 2.5,0,-1.5,-0.5*M_PI,0.2,GATE,BANGBANG,0,-0.5*M_PI,true,30},
// {2, 0.25,0,-1.5,-0.5*M_PI,0.2,GATE,BANGBANG,0,-0.5*M_PI,true,30},
// {3, -1.5,0,-1.5,-0.5*M_PI,0.2,GATE,BANGBANG,0,-0.5*M_PI,true,30},
// {4, 2.5,0,-1.5,-0.5*M_PI,0.2,ENDGATE,BANGBANG,0,-0.5*M_PI,true,30},
// };

// angle variations backwards
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,0,-1.5,0,0.2,STARTGATE,BANGBANG,0,0,true,30},
// {1, 2.5,0,-1.5,M_PI,0.2,GATE,BANGBANG,0,M_PI,true,30},
// {2, 0.25,0,-1.5,0,0.2,GATE,BANGBANG,0,0,true,30},
// {3, -2,0,-1.5,0,0.2,GATE,BANGBANG,0,0,true,30},
// {4, 2.5,0,-1.5,M_PI,0.2,ENDGATE,BANGBANG,0,M_PI,true,30},
// };

//  backwards
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,0,-1.5,0,0.2,STARTGATE,BANGBANG,0,0,true,35},
// {1, 2.5,0,-1.5,M_PI,0.2,ENDGATE,BANGBANG,0,M_PI,true,35},
// };

// demo forward height diff
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,0,-1.0,M_PI,0.2,STARTGATE,BANGBANG,0,0,false,35},
// {1, 2.5,0,-2.75,0,0.2,ENDGATE,BANGBANG,0,0,false,35},
// };

// Demo Forward/backwards 
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -1.5,0,-1.5,0.0,0.2,STARTGATE,BANGBANG,0,0,true,25},
// {1, 1.5,0,-1.5,0.0,-0.2,ENDGATE,BANGBANG,0,0,true,25},
// };


// // // Demo Sideways    // set saturation angles to 25 deg or lower for relatively safe
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,1,-1.75,-0.5*M_PI,0.2,STARTGATE,BANGBANG,0,-0.5*M_PI,true,35},
// {1, 2.0,1,-1.75,-0.5*M_PI,0.2,ENDGATE,BANGBANG,0,-0.5*M_PI,true,35},
// };

// Demo forward + sidestep 
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,2,-1.5,-0.5*M_PI,0.1,STARTGATE,BANGBANG,0,-0.5*M_PI,true,30},
// {1, 1.0,-2,-1.5,-0.5*M_PI,0.1,ENDGATE,BANGBANG,0,-0.5*M_PI,true,30},
// };

// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,1,-1.5,-0.5*M_PI,3,STARTGATE,BANGBANG,0,-0.5*M_PI,true,20},
// {0, 0.0,-1.5,-1.5,-0.5*M_PI,3,STARTGATE,BANGBANG,0,-0.5*M_PI,true,20},
// {1, 2.0,1,-1.5,-0.5*M_PI,3,ENDGATE,BANGBANG,0,-0.5*M_PI,true,20},
// };

// 4 waypoints continuous HIGHPID 
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,1.5,-1.5,-1*M_PI,3,STARTGATE,HIGHPID,0,-0.5*M_PI,false,30},
// {0, -2.0,-1.5,-1.5,-0.5*M_PI,3,STARTGATE,HIGHPID,0,-0.5*M_PI,false,30},
// {0, 2.0,-1.5,-1.5,0.0,3,STARTGATE,HIGHPID,0,-0.5*M_PI,false,30},
// {1, 2.0,1.5,-1.5,0.5*M_PI,3,ENDGATE,HIGHPID,0,-0.5*M_PI,false,30},
// };


// 4 waypoints continuous BANGBANG variable yaw
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,1.5,-1.5,-1*M_PI,3,STARTGATE,BANGBANG,0,-0.5*M_PI,false,30},
// {0, -2.0,-1.5,-1.5,-0.5*M_PI,3,STARTGATE,BANGBANG,0,-0.5*M_PI,false,30},
// {0, 2.0,-1.5,-1.5,0.0,3,STARTGATE,BANGBANG,0,-0.5*M_PI,false,30},
// {1, 2.0,1.5,-1.5,0.5*M_PI,3,ENDGATE,BANGBANG,0,-0.5*M_PI,false,30},
// };

// // 4 waypoints continuous BANGBANG fixed yaw
const struct bangbang_fp_struct Banggates[MAX_GATES] = {
{0, -2.0,1.5,-1.5,-0.5*M_PI,5,STARTGATE,BANGBANG,0,-0.5*M_PI,true,30},
{0, -2.0,-1.5,-1.5,-0.5*M_PI,5,GATE,BANGBANG,0,-0.5*M_PI,true,30},
{0, 2.0,-1.5,-1.5,-0.5*M_PI,5,GATE,BANGBANG,0,-0.5*M_PI,true,30},
{1, 2.0,1.5,-1.5,-0.5*M_PI,5,ENDGATE,BANGBANG,0,-0.5*M_PI,true,30},
};

// 4 waypoints continuous BANGBANG fixed yaw
// const struct bangbang_fp_struct Banggates[MAX_GATES] = {
// {0, -2.0,1.5,-1.5,-0.5*M_PI,2,STARTGATE,HIGHPID,0,-0.5*M_PI,true,25},
// {0, -2.0,-1.5,-1.5,-0.5*M_PI,2,GATE,HIGHPID,0,-0.5*M_PI,true,25},
// {0, 2.0,-1.5,-1.5,-0.5*M_PI,2,GATE,HIGHPID,0,-0.5*M_PI,true,25},
// {1, 2.0,1.5,-1.5,-0.5*M_PI,2,ENDGATE,HIGHPID,0,-0.5*M_PI,true,25},
// };


//----------------------------------------------------------------------------------------

static void update_gate_setpoints(void){
    dr_bang.gate_x= Banggates[dr_bang.gate_nr].gate_x;
    dr_bang.gate_y= Banggates[dr_bang.gate_nr].gate_y;
    dr_bang.gate_z= Banggates[dr_bang.gate_nr].gate_z;
    

    dr_bang.gate_speed_sat= Banggates[dr_bang.gate_nr].gate_speed_sat;
    // dr_bang.gate_speed_sec= Banggates[dr_bang.gate_nr].gate_speed_sec;
    dr_bang.gate_type= Banggates[dr_bang.gate_nr].gate_type;
    dr_bang.controller_type = Banggates[dr_bang.gate_nr].controller_type;
    dr_bang.turning=0;
    dr_bang.psi_forced=Banggates[dr_bang.gate_nr].psi_forced;
    dr_bang.gate_psi= Banggates[dr_bang.gate_nr].gate_psi;
    dr_bang.overwrite_psi=Banggates[dr_bang.gate_nr].overwrite_psi;
    
    #ifdef ANGLEOVERWRITE
    dr_bang.sat_angle=angle_variations[angle_index];
    #else
    dr_bang.sat_angle=Banggates[dr_bang.gate_nr].sat_angle;
    #endif
    sat_angle.x=-dr_bang.sat_angle*d2r;
    sat_angle.y=dr_bang.sat_angle*d2r;
    if(dr_bang.gate_type==ENDGATE){
        next_gate_nr=0;
    }
    else{
        next_gate_nr=dr_bang.gate_nr+1;
    }
    
}

void flightplan_reset(){
    dr_bang.gate_nr=0;
    update_gate_setpoints();
    dr_bang.controller_type=PID; // in a reset fly to the first waypoint in PID mode
    timer1=0;
    angle_index=0;
    target_reached=0;
    
}
    float pos_error_x;
    float pos_error_z;
    float pos_error_y;
    float pos_error_x_vel; 

    float error_speed;
    float error_psi; 

void flightplan_run(void){
    
    

    // update_gate_setpoints();
    // printf("target reached: %d, distance to gate: %f\n",target_reached,dist2gate);
    pos_error_x=dr_bang.gate_x-dr_state.x; 
    pos_error_y=dr_bang.gate_y-dr_state.y;
    pos_error_z=dr_bang.gate_z-dr_state.z;
    pos_error_x_vel=cosf(dr_state.psi)*pos_error_x+sinf(dr_state.psi)*pos_error_y;
    // dist2gate=sqrtf((pos_error_x*pos_error_x)+(pos_error_y*pos_error_y));
    // error_speed=dr_bang.gate_speed_sat-((dr_state.vx*dr_state.vx)+(dr_state.vy*dr_state.vy));
    // printf("dist: %f\n", dist2gate);
    if(dist2gate<0.7){
        if(fabs(dr_bang.gate_speed_sat)<=0.3 && dist2gate<0.5){ //assume we want to stop at the gate
            timer1+=1;
            if((fabs(dr_state.psi-dr_bang.gate_psi)<0.3)){
                // printf("reached 1");
                target_reached=1; //when passed waypoint in x-direction it counts as target reached (but we do not yet switch to the next waypoint)
            }
            // printf("theta: %f, phi: %f, banggates[index].psi: %f,Banggates[nextindex].psi: %f psi: %f, psi gate: %f, next index: %d\n",dr_state.theta,dr_state.phi,Banggates[dr_bang.gate_nr].gate_psi,Banggates[next_gate_nr].gate_psi ,dr_state.psi,dr_bang.gate_psi,next_gate_nr);
            if( fabs(dr_state.theta)<0.25 && timer1>128){
                // printf("reached 2");
                dr_bang.controller_type=PID;
                if(fabs(dr_state.theta)<0.05 && fabs(dr_state.phi)<0.05){
                dr_bang.gate_psi=Banggates[next_gate_nr].gate_psi;//atan2f(Banggates[next_gate_nr].gate_y-dr_state.y,Banggates[next_gate_nr].gate_x-dr_state.x);
                dr_bang.psi_forced=Banggates[next_gate_nr].psi_forced;
                // printf("\n Gate_psi: %f\n",dr_bang.gate_psi);
                dr_bang.turning=TURNING;       
                }     
            }
            // printf("psi thresh: %f, current psi_cmd: %f, next gate psi: %f\n",fabs(dr_state.psi-dr_bang.gate_psi),dr_bang.gate_psi,Banggates[next_gate_nr].gate_psi);
            if(abs(dr_state.psi-dr_bang.gate_psi)<0.05){ //only go toward next waypoint after a pause
                    // printf("Reached 3\n");
                    if(fabs(dr_state.psi-dr_bang.gate_psi)<0.1 && timer1>1024){
                    dr_bang.gate_nr=next_gate_nr;
                    printf("new gate: %i\n",next_gate_nr);
                    timer1=0;
                    brake=false;
                    controllerstate.in_transition=false;
                    // printf("Reached 4 \n");
                    target_reached=0;
                    }
                }
        }
        else if(fabs(dr_bang.gate_speed_sat)>0.3){ // asummed we don't want to stop at the gate. 
            dr_bang.gate_nr=next_gate_nr;
            printf("new gate: %i\n",next_gate_nr);
            brake = false; 
            controllerstate.in_transition=false;
            target_reached=0;
        }
    }

    if(dr_bang.gate_nr==next_gate_nr){ //only update setpoints if the gate identifier has changed (todo: fix discrepancy when there is only one gate)
        update_gate_setpoints();
        if(angle_index<NR_ANGLEVAR-1){
            angle_index+=1;
        }
        else{
            angle_index=0;
            printf("\n\n\n END ANGLE VARIATIONS \n\n\n");
        }
    }

    #ifdef LOG
        fprintf(fp_logger_t,"%f, %d, %d, %d, %f, %f, %f, %f, %d, %f\n",get_sys_time_float(),dr_bang.gate_nr,dr_bang.gate_type,dr_bang.controller_type,dr_bang.gate_x,dr_bang.gate_y,dr_bang.gate_z,dr_bang.gate_psi,target_reached,dr_bang.gate_speed_sat);
    #endif
  
    // printf("Controltype: %d, error_speed: %f, dist2gate: %f, psi: %f \n",dr_bang.controller_type,error_speed,dist2gate,dr_state.psi);
}

float wrap_angle(float ang,float ang2){
    if(ang>ang2){
        ang=ang-2*M_PI;
    }
    if(ang<-ang2){
        ang=ang+2*M_PI;
    }
    return ang;
}