///@brief Define the dynamics group class
///
///
///@author Yonglin
///@version 0.1
///@date 2-Nov-2023
///




#ifndef _ROB_DYN_GRP_H_
#define _ROB_DYN_GRP_H_

#include "robconf.hpp"
#include "robot.hpp"
#include "dynmia.hpp"
#include "accopt.hpp"
#include "load.hpp"




/*
 * Number of joints in dynamic group
 */
#define DYNGRP_MAX_NO_OF_JOINTS                 MAN_DYNGRP_MAX_NO_OF_JOINTS

/*
 * Maximal number of dynamic models in a dynamic group
 */
#define DYNGRP_MAX_NO_OF_DYN_MODELS             MAN_MAX_NO_OF_DYNAMIC_MODELS


/*
 * The number of resultant torques, forces and accelerations for one cross section
 */
#define DYNGRP_NO_OF_RES_AUX_PER_CROSS_SEC          3

/*
 * The maximum size for all supported dynamic models
 * These parameters may have to be updated when adding a new dynamic model
 */
#define DYNGRP_MAX_SIZE_OF_Y_VECTOR         1900
#define DYNGRP_MAX_SIZE_OF_Z_VECTOR         2500
#define DYNGRP_MAX_NO_OF_AUX                MAN_MAX_NO_OF_AUX
#define DYNGRP_MAX_NO_OF_CROSS_SECTIONS     20
#define DYNGRP_MAX_NO_OF_RESULTING_AUX      DYNGRP_MAX_NO_OF_CROSS_SECTIONS*DYNGRP_NO_OF_RES_AUX_PER_CROSS_SEC


/*
 * Error codes
 */

#define DYNGRP_ACC_TORQUE_NOT_SUFFICIENT   -101
#define DYNGRP_CONS_DEC_TORQUE_NOT_SUFFICIENT   -102
#define DYNGRP_METHOD_NOT_ALLOWED_FOR_DEC   -103
#define DYNGRP_DEC_TORQUE_NOT_SUFFICIENT   -104
#define DYNGRP_POS_DEP_TORQUE_EXCEEDED   -105
#define DYNGRP_TORQUE_ERR_DUE_TO_HIGH_SPEED  -200
#define DYNGRP_DYNAMIC_JOINT_TORQUE_LIMIT_EXCEEDED  -201
#define DYNGRP_DYNAMIC_AUX_TORQUE_LIMIT_EXCEEDED  -202

/*------------------------- Public Typedef ------------------------*/

typedef enum
{
  DYNGRP_DYNRX1 = 1,
  DYNGRP_DYNMIA = 2,
  DYNGRP_DYNRX2 = 3,
  DYNGRP_DYNMI2 = 4,
  DYNGRP_DYNHL8 = 5,
  DYNGRP_TEMPO = 6,
  DYNGRP_DYNPLM = 7,
  DYNGRP_DYNVOY = 8,
  DYNGRP_DYNTRI = 9,
  DYNGRP_DYNROF = 10,
  DYNGRP_DYNELPMX = 11,
  DYNGRP_DYNPRPMX = 12,
  DYNGRP_DYNPSPMX = 13,
  DYNGRP_DYNPRP = 14,
  DYNGRP_DYNBBTMX = 15,
  DYNGRP_DYNPRTMX = 16,
  DYNGRP_DYNTRACK = 17,
  DYNGRP_DYNBBT = 18,
  DYNGRP_DYNPRT = 19,
  DYNGRP_DYNPIC = 20,
  DYNGRP_DYNLMAR = 21,
  DYNGRP_DYNLMAA = 22,
  DYNGRP_DYNLMAK = 23,
  DYNGRP_DYNLMAL = 24,
  DYNGRP_DYNPICMX = 25,
  DYNGRP_DYNSPIDER = 26,
  DYNGRP_DYNBBL = 27,
  DYNGRP_DYNNEZHA = 28//7dof by zrx
}DYNGRP_MODEL;





typedef struct
{
  int aux_no;
  MAN_AUX_TYPE aux_type;
  int cross_section;
}DYNGRP_AUX_NO_TYPE;

typedef struct
{
	int aux_1;
	int aux_2;
	int aux_3; 
	int aux_4;
	int cross_section_rot_type;
}DYNGRP_PREV_CS_INFO;

/*
 * Coupled gear ratio to the axis ahead
 * and the second axis ahead.
 */
typedef struct
{
  float gear_coupled_1;
  float gear_coupled_2;
}DYNGRP_JOINT_COUPLING;

/*
* Dynamic group specific data
*/
typedef struct
{
	bool active;            /* MIMO Control Master active */
	int  algorithm;
	float spring_offset[3];
	float wcp_offset;
	float svd_tol;
	float q_vector[9];
	float A39_jacobian[3][9];
	//yonglin NLSBlock* nls1[13];     /* Nonlinear Stiffness Block */
	float k_db[13]; /* k[13] = [ka1x ka1y kg1 ka2x kg2 ka2z ka3x kg3 ka3z kg4 kg5 kg6 kg7] */
	float d_db[13]; /* d[13] = [ca1x ca1y cg1 ca2x cg2 ca2z ca3x cg3 ca3z cg4 cg5 cg6 cg7] */
	float k[13];    /* k[13] = [ka1x ka1y kg1 ka2x kg2 ka2z ka3x kg3 ka3z kg4 kg5 kg6 kg7] */
	float d[13];    /* d[13] = [ca1x ca1y cg1 ca2x cg2 ca2z ca3x cg3 ca3z cg4 cg5 cg6 cg7] */
	bool use_gravity_in_delta_pos;
	bool static_gravity_delta_pos_comp_off;
	bool use_gravity_in_trq_ffw;
	float max_gravity_derivative[13]; /* Maximum limit of derivative of gravity torque */
	float old_gravity_torque[13];
	IIRCFF filter_coeff[13];
	IIRFLT filter[13];
	int  t1_counter;
	int  t1_interval;       /* Time in 4 ms samples */
	int  t2_counter;
	int  t2_interval;       /* Time in 4 ms samples */
	float t2_sampling_time;
}MIMO_CM_DYNAMIC_GROUP;





namespace interpolation
{


//class CAcc_Opt;
//class CDyn_Mia;

class CJoints;



class CDynGrp
{
private:
    /* data */

    
    CDynGrp * dyngrp_new_internal (const MAN_DYNGRP_INFO * dyngrp_info,
								  CRobot * man,
								  const CLoad * load_in,
								  const CJoints * position_in,
								  const CDynGrp * first_dyngrp,
								  const CDynGrp * last_dyngrp, 
								  const float interp_ratio);

    void dynamic_model_get_model_parameters (int * no_of_aux_out,
                                            int * no_of_cross_sections_out,
                                            int * no_of_resulting_aux_out,
											                      int * size_of_y_vector_out);

    void dynamic_model_get_spd_dep_inertia_torque_aux (const MAN_DYNGRP_JOINTS * dyngrp_speed,
                                                           float * speed_dep_inertia_torq);
    void dynamic_model_calc_mass_matrix (float z[]);
    void dynamic_model_calc_gravity (float z[],const MAN_DYNGRP_JOINTS * position_in);
    void dynamic_model_get_wcp_pos ();
    void dynamic_model_get_spd_dep_inertia_torque (const MAN_DYNGRP_JOINTS * dyngrp_speed,
                                                       MAN_DYNGRP_JOINTS * spd_dep_inertia_torque);

    void dynamic_model_calc_gravity_aux (float z[], const MAN_DYNGRP_JOINTS * position_in);
    void dynamic_model_calc_mass_matrix_aux (float z[]);
   void dynamic_model_calc_z_and_y (float * z,
								const MAN_DYNGRP_JOINTS * dyngrp_position,
								CARTESIAN * load_pos2,
								float * load_mass,
								CARTESIAN * load_moment_of_inertia,
								CARTESIAN * load_moment_of_inertia_2);
    bool dynamic_model_get_cross_section_no (int aux_no, int *cross_section);
    MAN_AUX_TYPE dynamic_model_get_cross_section_aux_type (int aux_no);
    bool dynamic_model_get_resulting_aux_numbers (int cross_section_no, MAN_AUX_TYPE res_type, int *aux1, int *aux2, int *aux3);
    void get_disturbance_torque (const MAN_DYNGRP_JOINTS * speed, const MAN_DYNGRP_JOINTS * acc_dep_torque, MAN_DYNGRP_JOINTS * disturbance_torque);
    void get_disturbance_torque_and_spd_dep_inertia_torq (const MAN_DYNGRP_JOINTS * speed,
                                                              const MAN_DYNGRP_JOINTS * acc_dep_torque,
                                                              MAN_DYNGRP_JOINTS * disturbance_torque,
                                                              MAN_DYNGRP_JOINTS * speed_dep_inertia_torq);
  void compute_acc_dep_torque_aux (const MAN_DYNGRP_JOINTS * ref_acc,float * acc_dep_torque);
  void get_dynamic_model_disturbance_torque (const MAN_DYNGRP_JOINTS * speed,
                                                   const MAN_DYNGRP_JOINTS * acc_dep_torque,
                                                   MAN_DYNGRP_JOINTS * disturbance_torque);
  void get_disturbance_torque_cons_dec (const MAN_DYNGRP_JOINTS * speed,
                                              const MAN_DYNGRP_JOINTS * acc_dep_torque,
                                              const MAN_MOTION_MODE current_mode,
                                              MAN_DYNGRP_JOINTS * disturbance_torque,
                                              MAN_DYNGRP_JOINTS * disturbance_torque_cons_dec);                                         
  void get_disturbance_torque_cons_dec_aux (const MAN_DYNGRP_JOINTS * speed,
                                                  const float * acc_dep_torque,
                                                  float * disturbance_torque,
                                                  float * disturbance_torque_cons_dec);
  bool resulting_aux_is_defined(const MAN_AUX_TYPE aux_type,
									  const int cross_sec_no,
									  const MAN_CROSS_SECTIONS * cross_sections,   
                                      int *aux_1, int *aux_2, int *aux_3);
  void set_auxiliary_director();
  void calc_pos_dep_dynamic_model (const MAN_DYNGRP_JOINTS * position_in,const CLoad * load_in);
  void calc_pos_dep_dynamic_model_interpolated (const MAN_DYNGRP_JOINTS * position_in,
													const CLoad * load_in, 
													const CDynGrp *first_dyngrp,
													const CDynGrp *last_dyngrp,
													const float c1);

  void calc_speed_ratio_dep_dynamic_model ();
  bool check_if_enough_torque_due_to_torque_dist (int dyngrp_joint,  /* Joint number within dyngrp */
                                                        const MAN_DYNGRP_JOINTS * speed_0,  /* Start value of speeds */
                                                        const float ratio,  /* speed_ratio scale */
                                                        const float V,  /* Speed dep. inertia torque at speed_0 */
                                                        const float req_speed_ratio,  /* Requested speed / max speed */
                                                        const bool test_zero_dyn_fric,
                                                        const bool test_zero_spd_dep_inertia);


  float reduced_speed (const int dyngrp_joint,                             /* Joint no within dynamic group */
                            const MAN_DYNGRP_JOINTS * speed_0,  /* Start value of speeds */
                            const CJoints * joints_ratio,
                            const float start_ratio,                             /* start value for algorithm */
                            const float V,                                               /* Speed dep. inertia torque at speed_0 */
                            const bool test_zero_dyn_fric,
                            const bool test_zero_spd_dep_inertia,
                            bool * status);

  /*****************************************************************************************************/

  bool compute_rel_perform_ratio (const float ref_acc,
                                  const float max_acc_quota_in,
                                  const float max_torque,
                                  const float max_torque_gear_box,
										              const float max_acc_torque_motor,
										              const float max_acc_torque_gear_box,
                                  const float acc_dep_torque,
                                  float motor_acc_dep_torque,
                                  const float dist_torque,
                                  const bool optimal_gear_box_torque,
                                  float * rel_perform_ratio);

  bool compute_rel_perform_ratio_aux (const float * acc_dep_torque,
                                      const float * dist_torque,
											                const MAN_CROSS_SECTIONS * cross_sections,
                                      float * rel_perform_ratio);

  bool compute_rel_perform_ratio_resulting_aux (const float * acc_dep_torque,
                                                const float * dist_torque,
														                    const MAN_CROSS_SECTIONS * cross_sections,
                                                float * rel_perform_ratio);

  bool compute_rel_perform_ratio_cons_dec_aux (const float * acc_dep_torque,
                                               const float * dist_torque_cons_dec,
															                 const MAN_CROSS_SECTIONS * cross_sections,
                                               float * rel_perform_ratio_cons_dec);

  bool compute_rel_perform_ratio_resulting_aux_and_cons_dec_aux (const float * acc_dep_torque,
                                                                 const float * dist_torque_acc,
																		                             const MAN_CROSS_SECTIONS * cross_sections,
                                                                 float * rel_perform_ratio,
                                                                 float * rel_perform_ratio_cons_dec);

  bool compute_aux_max_dec_cons (const float * gravity_torque,
													   const float * speed_dep_torque,
													   const float * acc_dep_torque,
													   const MAN_CROSS_SECTIONS * cross_sections,
													   float * rel_perform_ratio_max_dec_cons);

  bool compute_norm_aux_max_dec_cons (const float * gravity_torque,
											  const float * speed_dep_torque,
											  const float * acc_dep_torque,
											  const MAN_CROSS_SECTIONS * cross_sections,
											  float * rel_perform_ratio_max_dec_cons);

  bool compute_speed_ratio_aux (const float * gravity_aux_in,
                              const float * spd_dep_inertia_torq,
										          const MAN_CROSS_SECTIONS * cross_sections,
                              float * speed_ratio);
  bool compute_speed_ratio_resulting_aux (const float * spd_dep_inertia_torq,
												                  const MAN_CROSS_SECTIONS * cross_sections,
                                          float * speed_ratio_res);

  void adjust_dyngrp_motor_acc_dep_torque_for_coupled_motor_inertias (const MAN_DYNGRP_JOINTS * acc,
                                                                      MAN_DYNGRP_JOINTS * dyngrp_motor_acc_dep_torque);
  bool dynamic_model_get_prev_cs_info  (int cross_section_no,
                                         int *aux1,
                                         int *aux2,
										 int *aux3,
										 int *aux4, 
										 int *cross_section_rot_type);
  void compute_prev_cs_aux  (const float * torque_in1,float *  torque_out1);
  bool compute_joint_max_dec_cons (const MAN_DYNGRP_JOINTS  * dyngrp_speed,
										   const MAN_DYNGRP_JOINTS  * dyngrp_ref_acc,
										   const MAN_DYNGRP_JOINTS  * dyngrp_max_torque,
										   const MAN_DYNGRP_JOINTS  * dyngrp_max_acc_torque,
										   MAN_DYNGRP_JOINTS  * dyngrp_acc_dep_torque,
										   MAN_DYNGRP_JOINTS  * grav_torque_cons_dec,
										   MAN_DYNGRP_JOINTS  * speed_torque_cons_dec,
										   MAN_DYNGRP_JOINTS  * dyngrp_rel_perform_ratio_cons_dec);
  bool compute_joint_max_dec_cons_rel_perform_ratio (const float ref_acc,
												const float max_acc_quota_in,
												const float max_torque,
												const float max_acc_torque,
												const float acc_dep_torque,
												const float gravity_torque,
												const float speed_torque,
												float * rel_perform_ratio);

                       

  bool compute_joint_acc_bound (bool reduce_speed,   // reduce speed if true and lower bound > 0
	bool use_acc_quota,  // use the acc_quota parameter to save some torque for dec
	bool include_max_dec_cons, // compute and add the conservative lower bound
	float acc_performance_ratio, // parameter set by the user in RAPID (command AccSet)
	float dacc_performance_ratio, // same as above
	const CJoints * max_torque_upper,
	const CJoints * max_torque_lower,
	const CJoints * max_torque_cons,
	const float * gravity_in,
	MAN_DYNGRP_JOINTS * qprimT_Gamma_qprim_joint_in,
	MAN_DYNGRP_JOINTS * Mjoint_qprim,
	MAN_DYNGRP_JOINTS * Mjoint_qbiss,
	MAN_DYNGRP_JOINTS * torque_tolerance,
	MAN_DYNGRP_JOINTS * rel_dist_torque_model_error,
	const double ,
	ACON_TYPE acon_type,
	bool is_definite,
	CAccOpt *acc_cond);
  bool check_joint_gravity (const CJoints * max_torque_gravity);

bool compute_resulting_aux_acc_bound (bool reduce_speed, 
	const MAN_CROSS_SECTIONS * cross_sections,
	float * gravity_aux_in,
	float * qprimT_Gamma_qprim_aux_in,
	float * Maux_qprim_in,
	float * Maux_qbiss_in,
	const double delta_L,
	const bool use_margin,
	bool comp_cons,
	ACON_TYPE acon_type,
	CAccOpt * acc_cond);

  bool compute_aux_acc_bound (bool reduce_speed,   // reduce speed if true and lower bound > 0
										bool is_spd_cond,    // speed constraints
										bool include_max_dec_cons, // compute and add the conservative lower bound
										float acc_performance_ratio, 
										float dacc_performance_ratio,
										const MAN_CROSS_SECTIONS * cross_sections,
										const float *gravity_aux_in,
										const float * qprimT_Gamma_qprim_in,
										const float * Maux_qprim_in,
										const float * Maux_qbiss_in,
										const double delta_L,
										const bool use_margin,
										ACON_TYPE acon_type,
										CAccOpt * acc_cond);

  bool check_aux_gravity (const MAN_CROSS_SECTIONS * cross_sections,
								 const float *gravity_aux_in,
								 const bool use_margin);

  bool check_resulting_aux_gravity (const MAN_CROSS_SECTIONS * cross_sections,
														  const bool use_margin);










public:

  CDynGrp * next;  /* Next free object or NIL */
  CRobot * man;
  int use_count;
  DYNGRP_MODEL model;
  int no_of_aux;
  int no_of_cross_sections;
  int no_of_resulting_aux;
  int size_of_y_vector;
  float mass_matrix[DYNGRP_MAX_NO_OF_JOINTS][DYNGRP_MAX_NO_OF_JOINTS];
  float mass_matrix_arm[DYNGRP_MAX_NO_OF_JOINTS][DYNGRP_MAX_NO_OF_JOINTS];
  float mass_matrix_aux[DYNGRP_MAX_NO_OF_AUX][DYNGRP_MAX_NO_OF_JOINTS];
  float gravity[DYNGRP_MAX_NO_OF_JOINTS];
  float unbalanced_gravity[DYNGRP_MAX_NO_OF_JOINTS];
  float gravity_aux[DYNGRP_MAX_NO_OF_AUX];
  float compl_inertia[DYNGRP_MAX_NO_OF_JOINTS];
  float y[DYNGRP_MAX_SIZE_OF_Y_VECTOR];
  const MAN_DYNGRP_INFO *dyngrp_info_p;
  MAN_DYNGRP_JOINTS abs_torque_margin_at_low_speed_ratio;
  MAN_DYNGRP_JOINTS balance_torque;
  MAN_DYNGRP_JOINTS torque_tolerance;
  MAN_DYNGRP_JOINTS torque_tol_no_dist_err;
  MAN_DYNGRP_JOINTS balance_reaction_1;
  MAN_DYNGRP_JOINTS balance_reaction_2;
  MAN_DYNGRP_JOINTS position;
	MAN_DYNGRP_JOINTS sinj;
	MAN_DYNGRP_JOINTS cosj;
  DYNGRP_JOINT_COUPLING torque_coupling[DYNGRP_MAX_NO_OF_JOINTS];
  bool optimal_gear_box[DYNGRP_MAX_NO_OF_JOINTS];
  bool locked_joint[DYNGRP_MAX_NO_OF_JOINTS];
  bool max_world_acc_is_set;
  CLoad load;

  MAN_DYNGRP_JOINTS qprimT_Gamma_qprim_joint;
  MAN_DYNGRP_JOINTS Mmotor_qprim;
  MAN_DYNGRP_JOINTS Mmotor_qbiss;
  MAN_DYNGRP_JOINTS Mgear_box_qprim;
  MAN_DYNGRP_JOINTS Mgear_box_qbiss;
  float qprimT_Gamma_qprim_aux[DYNGRP_MAX_NO_OF_AUX];
  float Maux_qprim[DYNGRP_MAX_NO_OF_AUX];
  float Maux_qbiss[DYNGRP_MAX_NO_OF_AUX];
  MAN_DYNGRP_JOINTS rel_dist_torque_model_error;
  MAN_DYNGRP_JOINTS max_acc_quota;
  float wcp_x;
  float wcp_z;





  CDynGrp(/* args */);
  ~CDynGrp();

  void dyngrp_init (void);
  void dyngrp_configurate (void);
  void dyngrp_attach (void);






  CDynGrp * dyngrp_new (const MAN_DYNGRP_INFO * dyngrp_info,
                          CRobot * man_in,
                          const CLoad * load_in,
                          const CJoints * position_in);

  CDynGrp * dyngrp_new_interpolated (const MAN_DYNGRP_INFO * dyngrp_info,
													CRobot * man_in,
													const CLoad * load_in,
													const CJoints * position_in, 
													const CDynGrp * first_dyngrp,
													const CDynGrp * last_dyngrp, 
													const float interp_ratio);

  int dyngrp_free (CDynGrp * object);
  void dyngrp_connect (CDynGrp * object);
  CDynGrp *dyngrp_pointer (CDynGrp * object);
  int dyngrp_no_of_used_objects (void);


  bool dyngrp_set_world_acc (const float max_world_acc, const bool max_world_acc_is_set_in, MAN_CROSS_SECTIONS * cross_sections);
  bool dyngrp_get_cross_section_no (int aux_no,int *cross_section);
  MAN_AUX_TYPE dyngrp_get_cross_section_aux_type (int aux_no);
  bool dyngrp_get_resulting_aux_numbers (int cross_section_no, MAN_AUX_TYPE res_type,
                                                int *aux1, int *aux2, int *aux3);
  void dyngrp_get_disturbance_torque_and_spd_dep_inertia_torq_aux (const MAN_DYNGRP_JOINTS * speed,
                                                                        float * disturbance_torque,
                                                                        float * speed_dep_inertia_torq);


  bool dyngrp_get_rel_perform_ratio (const CJoints * speed,
                                    const CJoints * ref_acc,
                                    const CJoints * motor_acc_dep_torque,
                                    const CJoints * max_torque,
                                    const CJoints * max_torque_gear_box,
									                  const CJoints * max_acc_torque_motor,
									                  const CJoints * max_acc_torque_gear_box,
                                    CJoints * rel_perform_ratio);
  bool dyngrp_get_rel_perform_ratio_aux (const CJoints * speed,
                                       const CJoints * ref_acc,
										                    const MAN_CROSS_SECTIONS * cross_sections,
										                    const MAN_CROSS_SECTIONS * cross_sections_prev_cs,
                                        float * rel_perform_ratio_aux);

  bool dyngrp_get_rel_perform_ratio_speed_ratio (const CJoints * speed,
                                               const CJoints * ref_acc,
                                                        const CJoints * motor_acc_dep_torque,
                                                        const CJoints * max_torque,
                                                        const CJoints * max_torque_gear_box,
														                          const CJoints * max_acc_torque_motor,
														                            const CJoints * max_acc_torque_gear_box,
                                                        CJoints * rel_perform_ratio,
                                                        CJoints * speed_ratio,
														                            const int motion_group);
  bool dyngrp_get_rel_perform_ratio_speed_ratio_aux (const CJoints * speed,
                                                  const CJoints * ref_acc,
	                                                const MAN_CROSS_SECTIONS * cross_sections,
	                                                const MAN_CROSS_SECTIONS * cross_sections_prev_cs,
                                                  float * rel_perform_ratio_aux,
                                                  float * speed_ratio_aux);

  bool dyngrp_get_rel_perf_cons_dec (const CJoints * speed,
                                            const CJoints * ref_acc,
                                            const CJoints * motor_acc_dep_torque,
                                            const CJoints * max_torque,
                                            const CJoints * max_torque_gear_box,
                                            const CJoints * max_torque_gear_box_dec,
                                            const CJoints * max_torque_dec,
														  const CJoints * max_acc_torque_motor,
														  const CJoints * max_acc_torque_gear_box,
                                            const MAN_MOTION_MODE current_mode,
                                            CJoints * rel_perform_ratio_acc,
                                            CJoints * rel_perform_ratio_cons_dec);

  bool dyngrp_get_rel_perf_cons_dec_aux (const CJoints * speed,
                                    const CJoints * ref_acc,
	 								const MAN_CROSS_SECTIONS * cross_sections,
	 								const MAN_CROSS_SECTIONS * cross_sections_prev_cs,
                                    float * rel_perform_ratio_acc_aux,
                                    float * rel_perform_ratio_cons_dec_aux);




  void dyngrp_get_torque (const CJoints * speed,
                        const CJoints * acc,
                        const CJoints * motor_acc_dep_torque,
                        CJoints * torque);

void dyngrp_get_aux_info (int * no_of_cross_sections_out,
								 MAN_CROSS_SECTION_TYPE * cross_section_type_out,
								 int * cross_section_physical_joint_out,
								 int * no_of_aux_out,
								 int * cross_section_index_out,
								 MAN_AUX_TYPE * aux_type_out);
void dyngrp_get_gravity_auxiliary_torque (float * auxiliary_torque,int * size_of_auxiliary_torque);
void dyngrp_get_auxiliary_torque (const CJoints * speed,
                                  const CJoints * acc,
                                  float * auxiliary_torque,
                                  int * size_of_auxiliary_torque);
void dyngrp_get_inertia (CJoints * inertia);
void dyngrp_get_compl_inertia (CJoints * compl_inertia_out);
void dyngrp_get_gravity (CJoints * gravity_out);
void dyngrp_get_unbalanced_gravity (CJoints * gravity_out);
void dyngrp_get_gravity_aux (MAN_GRAVITY_AUX * gravity_aux_out);
void dyngrp_get_spd_dep_dist_torque (const CJoints * speed,
                                     const CJoints * acceleration,
                                     CJoints * spd_dep_dist_torque);
void dyngrp_get_acc_dep_dist_torque (const CJoints * acceleration,CJoints * acc_dep_dist_torque);
void dyngrp_get_pos_and_acc_dep_dist_torque (const CJoints * acceleration,
                                             CJoints * pos_and_acc_dep_dist_torque);
bool dyngrp_get_avaible_acc_torque (const CJoints * speed,
                                    const CJoints * ref_acc,
                                    const CJoints * max_torque,
                                    CJoints * avaible_acc_torque,
                                    CJoints * torque_utilized);
bool dyngrp_check_torque (const CJoints * speed,const CJoints * acc,const CJoints * motor_acc_dep_torque,
                          const CJoints * spd_dep_dist_torque,const MAN_CROSS_SECTIONS * cross_sections,
                          const bool spd_dep_dist_torque_calculated,const bool check_micro_sample_torque);

bool dyngrp_check_dynamic_limits (const CJoints * speed,
								   const CJoints * acc,
								   const CJoints * gear_box_torque_lower_limit,
								   const CJoints * gear_box_torque_upper_limit,
								   const CJoints * motor_torque_lower_limit,
								   const CJoints * motor_torque_upper_limit,
								   const MAN_CROSS_SECTIONS * cross_sections,
								   const MAN_CROSS_SECTIONS * cross_sections_prev_cs,
								   const bool check_cross_sections_limits,
								   float utilization_rate_limit[MAN_NO_OF_ROBOTS], 
								   float * max_utilization_rate_out);
bool dyngrp_reduce_torque_wrt_power(ROBOT_EMERGENCY_STOP_PHASE * phase, 
										   const CJoints * speed,CJoints * emergency_stop_torque_max);

bool dyngrp_calc_cart_pos (CJoints * joints,CARTESIAN * pos);
bool dyngrp_get_joints_acc_bound (
   const CJoints * speed,
   const CJoints * max_torque_motor_upper,
   const CJoints * max_torque_motor_lower,
   const CJoints * max_torque_gear_box_upper,
   const CJoints * max_torque_gear_box_lower,
   const CJoints * max_torque_motor_upper_cons,
   const CJoints * max_torque_motor_lower_cons,
   const CJoints * max_torque_gear_box_upper_cons,
   const CJoints * max_torque_gear_box_lower_cons,
   const CJoints * dq_dl,
   const CJoints * d2q_dl2,
   const float v,
   const float delta_L,
   CJoints * upper_bound,
   CJoints * lower_bound,
   float * cons_acc_upper,
   float * cons_acc_lower);

bool dyngrp_get_acc_bound_denominator (const CJoints * dq_dl,
                                       const CJoints * d2q_dl2,
                                       const float delta_L,
                                       CJoints * denom_motor,
                                       CJoints * denom_gearbox);

bool dyngrp_get_zero_crossing_v (const CJoints * speed,
                                          const CJoints * max_torque_motor_upper,
                                          const CJoints * max_torque_motor_lower,
                                          const CJoints * max_torque_gear_box_upper,
                                          const CJoints * max_torque_gear_box_lower,
                                          const CJoints * dq_dl,
                                          const CJoints * d2q_dl2,
                                          const float v,
                                          const float delta_L,
                                          CJoints * v_motor,
                                          CJoints * v_gearbox);

bool dyngrp_get_indefinite_limit_acc_bound (bool reduce_speed,   // reduce speed if true and lower bound > 0
	float acc_performance_ratio, 
	float dacc_performance_ratio, 
	const CJoints * max_motor_torque_upper,
	const CJoints * max_motor_torque_lower,
	const CJoints * max_motor_torque_cons,
	const CJoints * max_gear_box_torque_upper,
	const CJoints * max_gear_box_torque_lower,
	const CJoints * max_gear_box_torque_cons,
	const MAN_CROSS_SECTIONS * cross_sections,
	const double delta_L,
	CAccOpt * acc_cond);

  bool dyngrp_get_speed_dep_limit_dist_acc_bound_aux ( 
	float acc_performance_ratio, // parameter set by the user in RAPID (command AccSet)
	float dacc_performance_ratio, 
	const MAN_CROSS_SECTIONS * cross_sections,
	const MAN_CROSS_SECTIONS * cross_sections_prev_cs,
	const double delta_L,
	CAccOpt * acc_cond);

  bool dyngrp_get_indefinite_speed_dep_limit_dist_acc_bound ( 

	float acc_performance_ratio, // parameter set by the user in RAPID (command AccSet)
	float dacc_performance_ratio, 
	const CJoints * max_gear_box_dist_torque_upper,
	const CJoints * max_gear_box_dist_torque_lower,
	const double delta_L,
	CAccOpt * acc_cond);

  bool dyngrp_get_speed_dep_limit_dist_acc_bound_gear_box ( 
	const CJoints * max_torque_lower,
	const CJoints * max_torque_upper,
	const double v2,
	const double Mqbiss_factor,
	bool *feasible);

  bool dyngrp_get_max_dec_cons ( const CJoints * speed,
								const CJoints * ref_acc,
								const CJoints * max_motor_torque,
								const CJoints * max_motor_acc_torque,
								const CJoints * max_gear_box_torque,
								const CJoints * max_gear_box_acc_torque,
								const MAN_CROSS_SECTIONS * cross_sections,
								const MAN_CROSS_SECTIONS * cross_sections_prev_cs,
								const int motion_group,
								CJoints * max_dec_cons,
								float *rel_perform_ratio);

  bool dyngrp_check_gravity ( const CJoints * max_gear_box_torque_upper,
							const MAN_CROSS_SECTIONS * cross_sections,
							const MAN_CROSS_SECTIONS * cross_sections_prev_cs);

  bool dyngrp_get_definite_limit_acc_bound ( 
	bool reduce_speed,   // reduce speed if true and lower bound > 0
	float acc_performance_ratio,    // parameter set by the user in RAPID (command AccSet)
	float dacc_performance_ratio,   // same as above
   const CJoints * max_motor_acc_torque_upper,
	const CJoints * max_motor_acc_torque_lower,
	const CJoints * max_motor_torque_cons,
	const CJoints * max_gear_box_acc_torque_upper,
	const CJoints * max_gear_box_acc_torque_lower,
	const CJoints * max_gear_box_dist_torque_upper,
	const CJoints * max_gear_box_dist_torque_lower,
	const CJoints * max_gear_box_torque_cons,
	const MAN_CROSS_SECTIONS * cross_sections,
	const MAN_CROSS_SECTIONS * cross_sections_prev_cs,
	const double delta_L,
	CAccOpt * acc_cond);

  void dyngrp_get_moved_J123_torque (const CJoints * speed,
									const CJoints * acc,
									const MIMO_CM_DYNAMIC_GROUP * mimo_dyngrp_ptr,
									float J123torque[MAN_MAX_NO_OF_DYNAMIC_GROUPS][13]);

  void dyngrp_get_moved_J123_gravity_torque (const MIMO_CM_DYNAMIC_GROUP * mimo_dyngrp_ptr,
									float J123torque[MAN_MAX_NO_OF_DYNAMIC_GROUPS][13]);

  void dyngrp_get_moved_J123_speed_dep_torque (const CJoints * speed,
									const MIMO_CM_DYNAMIC_GROUP * mimo_dyngrp_ptr,
									float J123torque[MAN_MAX_NO_OF_DYNAMIC_GROUPS][13]);


  void dyngrp_get_moved_J123_acc_dep_torque (const CJoints * acc,
									const MIMO_CM_DYNAMIC_GROUP * mimo_dyngrp_ptr,
									float J123torque[MAN_MAX_NO_OF_DYNAMIC_GROUPS][13]);

  bool dyngrp_compute_common_parameters_aux (const CJoints * dq_dl,
												const CJoints * d2q_dl2);

  bool dyngrp_compute_common_parameters_joint (const CJoints * dq_dl,
												const CJoints * d2q_dl2,
												const bool emergency_stop);

  bool dyngrp_get_acc_bound_aux (
   const CJoints * speed,
   const MAN_CROSS_SECTIONS * cross_sections,
   const CJoints * dq_dl,
   const CJoints * d2q_dl2,
   const float v,
   const float delta_L,
   float * upper_bound_aux,
   float * lower_bound_aux,
   float * v_upper_aux,
   float * v_lower_aux,
   float * cons_acc_aux);

  bool dyngrp_get_joints_max_feasible_speed (
   const CJoints * max_torque_motor_upper,
   const CJoints * max_torque_motor_lower,
   const CJoints * max_torque_gear_box_upper,
   const CJoints * max_torque_gear_box_lower,
   const CJoints * max_torque_motor_upper_cons,
   const CJoints * max_torque_motor_lower_cons,
   const CJoints * max_torque_gear_box_upper_cons,
   const CJoints * max_torque_gear_box_lower_cons,
   const CJoints * dq_dl,
   const CJoints * d2q_dl2,   
   const float delta_L,
   const float vp,
   float * maxv,
   CJoints * upper_bound,
   CJoints * lower_bound,
   float * cons_acc_upper,
   float * cons_acc_lower);

  bool dyngrp_get_acc_bound_dyn_info(
   const CJoints * max_torque_motor_upper,
   const CJoints * max_torque_motor_lower,
   const CJoints * max_torque_gear_box_upper,
   const CJoints * max_torque_gear_box_lower,
   const CJoints * dq_dl,
   const CJoints * d2q_dl2,   
   const float delta_L,
   CJoints * denom_motor,
   CJoints * denom_gearbox,
   CJoints * torque_grav_motor_max,
   CJoints * torque_grav_motor_min,
   CJoints * torque_grav_gearbox_max,
   CJoints * torque_grav_gearbox_min,
   CJoints * speed_dep_motor,
   CJoints * speed_dep_gearbox,
   CJoints * Mgearbox_qbiss_joints,
   CJoints * denom_gearbox_acc_torque);

  void dyngrp_compute_prev_cs_aux  (const CDynGrp * p,
								  const float * torque_in1,
								  float *  torque_out1);

bool dyngrp_get_emergency_stop_dec (const CJoints * speed,
			const CJoints * max_dec,
			const CJoints * emergency_stop_torque_max,
			const CJoints * emergency_stop_torque_min,
			CJoints *decs,
			int no_of_micro_steps);

};





}

#endif