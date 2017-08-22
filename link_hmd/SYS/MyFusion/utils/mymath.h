#ifndef __MATH_H__
#define __MATH_H__




#define asinf  arm_asinf
#define arm_sinf   arm_sin_f32
#define arm_cosf   arm_cos_f32
#define expf   __ieee754_expf
#define sqrtf  arm_sqrtf


static inline float arm_sqrtf(float val)
{
    float ret;
    __asm("VSQRT.F32 %0,%1" : "=t"(ret) : "t"(val));
    return ret;
}
float atan2f(float, float);

static inline float arm_asinf(float x)
{
    return atan2f(x, sqrtf(1.0f - x * x));
}
typedef union
{
  float value;
  /* FIXME: Assumes 32 bit int.  */
  unsigned int word;
} ieee_float_shape_type;


#define SET_FLOAT_WORD(d,i)					\
do {								\
  ieee_float_shape_type sf_u;					\
  sf_u.word = (i);						\
  (d) = sf_u.value;						\
} while (0)

#define GET_FLOAT_WORD(i,d)                 \
do {                                \
  ieee_float_shape_type gf_u;                   \
  gf_u.value = (d);                     \
  (i) = gf_u.word;                      \
} while (0)

#define STRICT_ASSIGN(type, lval, rval) do {    \
    volatile type __lval;           \
                        \
    if (sizeof(type) >= sizeof(long double))    \
        (lval) = (rval);        \
    else {                  \
        __lval = (rval);        \
        (lval) = __lval;        \
    }                   \
} while (0)

#endif
