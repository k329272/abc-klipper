// PAROL6 endpoint IK helper.  The Python module owns G-code; this performs
// the hot numerical solve in Klipper's compiled helper library.
#include <math.h>

#include <stdlib.h>

#include <string.h>

#include "compiler.h"

#define P 3.14159265358979323846
struct parol_ik {
  double a[7], tool[3], scale;
};
static void mm(double o[16], double a[16], double b[16]) {
  double r[16];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      r[i * 4 + j] = 0;
      for (int k = 0; k < 4; k++) r[i * 4 + j] += a[i * 4 + k] * b[k * 4 + j];
    }
  memcpy(o, r, sizeof(r));
}
static void dh(double o[16], double t, double x, double r, double d) {
  double c = cos(t), s = sin(t), ca = cos(x), sa = sin(x);
  double v[16] = {
    c,
    -s * ca,
    s * sa,
    r * c,
    s,
    c * ca,
    -c * sa,
    r * s,
    0,
    sa,
    ca,
    d,
    0,
    0,
    0,
    1
  };
  memcpy(o, v, sizeof(v));
}
static void fk(struct parol_ik * i, double * q, double o[16]) {
  double t[16] = {
    1,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    0,
    1,
    0,
    0,
    0,
    0,
    1
  }, n[16];
  double th[6] = {
    q[0] * P / 180,
    q[1] * P / 180 - P / 2,
    q[2] * P / 180 + P,
    q[3] * P / 180,
    q[4] * P / 180,
    q[5] * P / 180 + P
  }, al[6] = {
    -P / 2,
    P,
    P / 2,
    -P / 2,
    P / 2,
    P
  }, r[6] = {
    i -> a[1],
    i -> a[2],
    -i -> a[3],
    0,
    0,
    -i -> a[6]
  }, d[6] = {
    i -> a[0],
    0,
    0,
    -i -> a[4],
    0,
    -i -> a[5]
  };
  for (int k = 0; k < 6; k++) {
    dh(n, th[k], al[k], r[k], d[k]);
    mm(t, t, n);
  }
  double z[16] = {
    1,
    0,
    0,
    i -> tool[0],
    0,
    1,
    0,
    i -> tool[1],
    0,
    0,
    1,
    i -> tool[2],
    0,
    0,
    0,
    1
  };
  mm(o, t, z);
}
static void er(struct parol_ik * i, double * t, double * q, double * e) {
  double c[16], r[9];
  fk(i, q, c);
  for (int k = 0; k < 3; k++) e[k] = t[9 + k] - c[k * 4 + 3];
  for (int x = 0; x < 3; x++)
    for (int y = 0; y < 3; y++) {
      r[x * 3 + y] = 0;
      for (int k = 0; k < 3; k++) r[x * 3 + y] += t[x * 3 + k] * c[y * 4 + k];
    }
  e[3] = .5 * (r[7] - r[5]) * i -> scale;
  e[4] = .5 * (r[2] - r[6]) * i -> scale;
  e[5] = .5 * (r[3] - r[1]) * i -> scale;
}
static int gauss(double a[6][6], double * b, double * x) {
  for (int c = 0; c < 6; c++) {
    int p = c;
    for (int r = c + 1; r < 6; r++)
      if (fabs(a[r][c]) > fabs(a[p][c])) p = r;
    if (fabs(a[p][c]) < 1e-12) return 1;
    for (int j = c; j < 6; j++) {
      double v = a[c][j];
      a[c][j] = a[p][j];
      a[p][j] = v;
    }
    double v = b[c];
    b[c] = b[p];
    b[p] = v;
    v = a[c][c];
    for (int j = c; j < 6; j++) a[c][j] /= v;
    b[c] /= v;
    for (int r = 0; r < 6; r++)
      if (r != c) {
        v = a[r][c];
        for (int j = c; j < 6; j++) a[r][j] -= v * a[c][j];
        b[r] -= v * b[c];
      }
  }
  memcpy(x, b, 48);
  return 0;
}
struct parol_ik * __visible parol_ik_alloc(void) {
  return calloc(1, sizeof(struct parol_ik));
}
void __visible parol_ik_set_params(
  struct parol_ik * i,
  double a1, double a2, double a3, double a4,
  double a5, double a6, double a7, double x,
  double y, double z, double s) {
  double a[7] = {
    a1, a2, a3, a4,
    a5, a6, a7
  };
  memcpy(i -> a, a, sizeof(a));
  i -> tool[0] = x;
  i -> tool[1] = y;
  i -> tool[2] = z;
  i -> scale = s;
}
int __visible parol_ik_solve(
  struct parol_ik * i, double * t, double * seed,
  int n, double tol, double * out) {
  double q[6];
  memcpy(q, seed, 48);
  for (int it = 0; it < n; it++) {
    double e[6], j[6][6], a[6][6], b[6], d[6];
    er(i, t, q, e);
    double mx = 0;
    for (int k = 0; k < 6; k++)
      if (fabs(e[k]) > mx) mx = fabs(e[k]);
    if (mx <= tol) {
      memcpy(out, q, 48);
      return 0;
    }
    for (int c = 0; c < 6; c++) {
      double q2[6], e2[6];
      memcpy(q2, q, 48);
      q2[c] += .02;
      er(i, t, q2, e2);
      for (int r = 0; r < 6; r++) j[r][c] = -(e2[r] - e[r]) / .02;
    }
    for (int r = 0; r < 6; r++) {
      b[r] = 0;
      for (int c = 0; c < 6; c++) {
        a[r][c] = (r == c) ? .01 : 0;
        for (int k = 0; k < 6; k++) a[r][c] += j[k][r] * j[k][c];
        b[r] += j[c][r] * e[c];
      }
    }
    if (gauss(a, b, d)) return 1;
    for (int k = 0; k < 6; k++) {
      if (d[k] > 5) d[k] = 5;
      if (d[k] < -5) d[k] = -5;
      q[k] += d[k];
    }
  }
  return 1;
}
