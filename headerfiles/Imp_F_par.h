/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int Imp_F_par(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int Imp_F_par_alloc_mem(void);
int Imp_F_par_init_mem(int mem);
void Imp_F_par_free_mem(int mem);
int Imp_F_par_checkout(void);
void Imp_F_par_release(int mem);
void Imp_F_par_incref(void);
void Imp_F_par_decref(void);
casadi_int Imp_F_par_n_out(void);
casadi_int Imp_F_par_n_in(void);
casadi_real Imp_F_par_default_in(casadi_int i);
const char* Imp_F_par_name_in(casadi_int i);
const char* Imp_F_par_name_out(casadi_int i);
const casadi_int* Imp_F_par_sparsity_in(casadi_int i);
const casadi_int* Imp_F_par_sparsity_out(casadi_int i);
int Imp_F_par_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#ifdef __cplusplus
} /* extern "C" */
#endif