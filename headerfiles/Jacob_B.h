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

int Jacob_B(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int Jacob_B_alloc_mem(void);
int Jacob_B_init_mem(int mem);
void Jacob_B_free_mem(int mem);
int Jacob_B_checkout(void);
void Jacob_B_release(int mem);
void Jacob_B_incref(void);
void Jacob_B_decref(void);
casadi_int Jacob_B_n_out(void);
casadi_int Jacob_B_n_in(void);
casadi_real Jacob_B_default_in(casadi_int i);
const char* Jacob_B_name_in(casadi_int i);
const char* Jacob_B_name_out(casadi_int i);
const casadi_int* Jacob_B_sparsity_in(casadi_int i);
const casadi_int* Jacob_B_sparsity_out(casadi_int i);
int Jacob_B_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#ifdef __cplusplus
} /* extern "C" */
#endif
