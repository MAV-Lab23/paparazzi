/** @file solveActiveSet_qr.c
 * @brief Use active-set QR algorithm to solve constrained least squares
 *
 * This algorithm will find the optimal solution to the least squares problem 
 * setup with setup_wls.c
 *
 * Uses an active-set algorithm with QR subspace optimisation. The QR components
 * are updated at every step instead of naively re-factorised.
 *
 * written by Till Blaha
 * MAVLab Delft University of Technology
 */

#include "solveActiveSet.h"
#include "size_defines.h"
#include <stdio.h>
/*#include "std.h"*/
#include <inttypes.h>
#include <stdbool.h>
#include <math.h>

#include <string.h>
#include <math.h>
#include <float.h>
#include "qr_wrapper.h"
#include "qr_updates.h"
#include "qr_solve/qr_solve.h"
#include "qr_solve/r8lib_min.h"
#include "sparse_math.h"

// provide loop feedback
#define WLS_VERBOSE FALSE

//#define debug_qr

/**
 * @brief active set algorithm for constrained least squares, QR factorisation
 *
 * Takes a representation of the problem
 * 
 *    min || Au - b ||^2 
 *      st. lb <= u <= lb
 * 
 * and returns the optimal solution u and the active-set, while respective a
 * certain iteration limit.
 * 
 * Inputs to this function can be derived from setup_wls.c
 * 
 * @param A_col column major representation of A
 * @param b the rhs
 * @param umin The minimum u vector
 * @param umax The maximum u vector
 * @param us Input/Output: initial guess to be iteratively improved
 * @param Ws Input/Output: indicates whether us[i] is bounded at umin[i] (-1),
 * at umax[i] (+1), or not bounded (0)
 * @param updating unused. Could be used to switch between updating and naive 
 * factorisations
 * @param imax Maximum number of iterations taken by the algorithm 
 * @param n_u Length of u
 * @param n_v Lenght of v
 * @param iter Iterations taken
 * 
 * @note n_u+n_v must be <= 256
 *
 * @return 0: exact solution found. 1: imax hit, solution not optimal
 */
int8_t solveActiveSet_qr(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C],
  const num_t umin[CA_N_U], const num_t umax[CA_N_U], num_t us[CA_N_U],
  int8_t Ws[CA_N_U], bool updating, int imax, const int n_u, const int n_v,
  int *iter, int *n_free)
{
  (void)(updating);

  if(!imax) imax = 100;

  uint8_t i;
  uint8_t j;
  uint8_t n_c = n_u + n_v;

  for (i = 0; i < n_u; i++) {
    if (Ws[i] == 0) {
      us[i] = (us[i] > umax[i]) ? umax[i] : ((us[i] < umin[i]) ? umin[i] : us[i]);
    } else {
      us[i] = (Ws[i] > 0) ? umax[i] : umin[i];
    }
  }

  num_t A[CA_N_C][CA_N_U];
  num_t Q[CA_N_C][CA_N_C];
  num_t R[CA_N_C][CA_N_U];

  // Create a pointer array to the rows of A
  // such that we can pass it to a function
  num_t * A_ptr[CA_N_C];
  num_t * Q_ptr[CA_N_C];
  num_t * R_ptr[CA_N_C];
  for(i = 0; i < n_c; i++) {
    A_ptr[i] = A[i];
    Q_ptr[i] = Q[i];
    R_ptr[i] = R[i];
  }

  int permutation[CA_N_U]; memset(permutation, 0, sizeof(int)*n_u);
  (*n_free) = 0;
  uint8_t i_bnd = 0;
  for (i = 0; i < n_u; i++) {
    if (Ws[i] == 0) {
      permutation[(*n_free)++] = i;
    }
  }
  for (i = 0; i < n_u; i++) {
    if (Ws[i] != 0) {
      permutation[(i_bnd++)+(*n_free)] = i;
    }
  }

  // convert col major input to 2d array
  for (i = 0; i < n_c; i++) {
    for (j = 0; j < n_u; j++) {
      A[i][j] = A_col[i + n_c * j];
    }
  }

  // initial factorisation
  qr_wrapper(n_c, n_u, permutation, A_ptr, Q_ptr, R_ptr);
  #ifdef debug_qr
  print_debug(A_ptr, Q_ptr, R_ptr, &n_u, &n_c);
  #endif

  num_t q[CA_N_U];
  num_t z[CA_N_U];

  // -------------- Start loop ------------
  *iter = 0;
  while (++(*iter) <= imax) {
    num_t c[CA_N_U];
    for (i=0; i < (*n_free); i++) {
      c[i] = 0;
      for (j=0; j < n_c; j++) {
        c[i] += Q_ptr[j][i]*b[j];
      }
    }

    num_t u_bound_perm[CA_N_U];
    for (i=0; i < n_u - (*n_free); i++) {
      if (Ws[permutation[i+(*n_free)]] > 0)
        u_bound_perm[i] = umax[permutation[i+(*n_free)]];
      else if (Ws[permutation[i+(*n_free)]] < 0) 
        u_bound_perm[i] = umin[permutation[i+(*n_free)]];
#ifdef debug_qr
      else
        printf("W out of bounds for bounded variables");
#endif
    }

    for (i=0; i < (*n_free); i++) {
      for (j=0; j < n_u - (*n_free); j++) {
        c[i] -= R_ptr[i][(*n_free)+j] * u_bound_perm[j];
      }
    }

    backward_tri_solve((*n_free), R_ptr, c, q);
    for (i = 0; i < (*n_free); i++) {
      z[permutation[i]] = q[i];
    }
    for (i = (*n_free); i < n_u; i++) {
      z[permutation[i]] = us[permutation[i]];
    }

    uint8_t n_violated = 0;
    int8_t W_temp[CA_N_U];
    n_violated = check_limits_tol((*n_free), TOL, z, umin, umax, W_temp, permutation);

    if (!n_violated) {
      for (i = 0; i < (*n_free); i++)
        us[permutation[i]] = z[permutation[i]];

      if ((*n_free) == n_u) {
        // no active constraints, we are optinal and feasible
        return 0;
      } else {
        // active constraints, check for optimality
        num_t d[CA_N_U];
        for (i=(*n_free); i < n_u; i++) {
          d[i] = 0;
          for (j=0; j < n_c; j++) {
            d[i] += Q_ptr[j][i]*b[j];
          }
        }

        for (i=(*n_free); i < n_u; i++) {
          for (j=i; j < n_u; j++){
            d[i] -= R_ptr[i][j]*us[permutation[j]];
          }
        }

        num_t lambda_perm[CA_N_U];
        uint8_t f_free = 0;
        num_t maxlam = -INFINITY;
        for (i=(*n_free); i<n_u; i++) {
          lambda_perm[i] = 0;
          for (j=(*n_free); j <= i; j++) {
            lambda_perm[i] += R_ptr[j][i]*d[j];
          }
          lambda_perm[i] *= -Ws[permutation[i]];
          if (lambda_perm[i] > maxlam) {
            maxlam = lambda_perm[i];
            f_free = i-(*n_free);
          }
        }

        if (maxlam <= 0) {
          return 0; // feasible and optimal
        }

        // free variable
        qr_shift(n_c, n_u, Q_ptr, R_ptr, (*n_free), (*n_free)+f_free);
        #ifdef debug_qr
        print_debug(A_ptr, Q_ptr, R_ptr, &n_u, &n_c);
        #endif

        Ws[permutation[(*n_free)+f_free]] = 0;
        uint8_t last_val = permutation[(*n_free)+f_free];
        for (i = f_free; i > 0; i--) {
          permutation[(*n_free)+i] = permutation[(*n_free)+i-1];
        }
        permutation[(*n_free)++] = last_val;

      }
    } else {

      num_t a = INFINITY;
      uint8_t i_a = 0;
      uint8_t f_bound = 0;
      int8_t i_s = 0;
      num_t temp;
      int8_t temp_s;
      for (uint8_t f=0; f < (*n_free); f++) {
        i = permutation[f];
        if (W_temp[i] == -1) {
          temp = (us[i] - umin[i]) / (us[i] - z[i]);
          temp_s = -1;
        } else if (W_temp[i] == +1) {
          temp = (umax[i] - us[i]) / (z[i] - us[i]);
          temp_s = +1;
        } else {
          continue;
        }
        if (temp < a) {
          a = temp;
          i_a = i;
          f_bound = f;
          i_s = temp_s;
        }
      }

      // update us
      for (i=0; i<n_u; i++) {
        num_t incr = a * (z[i] - us[i]);
        if (i == i_a) {
          us[i] = (incr > 0) ? umax[i] : umin[i];
        } else {
          us[i] += incr;
        }
      }

      qr_shift(n_c, n_u, Q_ptr, R_ptr, (*n_free)-1, f_bound);
      #ifdef debug_qr
      print_debug(A_ptr, Q_ptr, R_ptr, &n_u, &n_c);
      #endif

      Ws[i_a] = i_s;
      uint8_t first_val = permutation[f_bound];
      for (i = 0; i < (*n_free)-f_bound-1; i++) {
        permutation[f_bound+i] = permutation[f_bound+i+1];
      }
      permutation[--(*n_free)] = first_val;

    }

  }
  return 1; // hit iteration limit without correct solution
}

#if WLS_VERBOSE
void print_in_and_outputs(int n_c, int (*n_free), num_t** A_free_ptr, num_t* d, num_t* p_free) {

  printf("n_c = %d (*n_free) = %d\n", n_c, (*n_free));

  printf("A_free =\n");
  for(int i = 0; i < n_c; i++) {
    for (int j = 0; j < (*n_free); j++) {
      printf("%f ", A_free_ptr[i][j]);
    }
    printf("\n");
  }

  printf("d = ");
  for (int j = 0; j < n_c; j++) {
    printf("%f ", d[j]);
  }

  printf("\noutput = ");
  for (int j = 0; j < (*n_free); j++) {
    printf("%f ", p_free[j]);
  }
  printf("\n\n");
}

void print_final_values(int n_u, int n_v, num_t* u, num_t** B, num_t* v, num_t* umin, num_t* umax) {
  printf("n_u = %d n_v = %d\n", n_u, n_v);

  printf("B =\n");
  for(int i = 0; i < n_v; i++) {
    for (int j = 0; j < n_u; j++) {
      printf("%f ", B[i][j]);
    }
    printf("\n");
  }

  printf("v = ");
  for (int j = 0; j < n_v; j++) {
    printf("%f ", v[j]);
  }

  printf("\nu = ");
  for (int j = 0; j < n_u; j++) {
    printf("%f ", u[j]);
  }
  printf("\n");

  printf("\numin = ");
  for (int j = 0; j < n_u; j++) {
    printf("%f ", umin[j]);
  }
  printf("\n");

  printf("\numax = ");
  for (int j = 0; j < n_u; j++) {
    printf("%f ", umax[j]);
  }
  printf("\n\n");

}
#endif

#ifdef debug_qr
void print_debug(num_t** A_ptr, num_t** Q_ptr, num_t** R_ptr, const int* n_u, const int* n_c) {
  printf("A_c = [");
  for (int i = 0; i < *n_c; i++) {
    for (int j = 0; j < *n_u; j++) {
      printf("%f ", A_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");

  printf("Q_c = [");
  for (int i = 0; i < *n_c; i++) {
    for (int j = 0; j < *n_c; j++) {
      printf("%f ", Q_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");

  printf("R_c = [");
  for (int i = 0; i < *n_c; i++) {
    for (int j = 0; j < *n_u; j++) {
      printf("%f ", R_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");
}
#endif