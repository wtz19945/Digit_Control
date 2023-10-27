/*
 * Automatically Generated from Mathematica.
 * Mon 4 Jul 2022 20:55:20 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "dT_shoulder_pitch_joint_right_src.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }
//INLINE double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(double *p_output1,const double *var1,const double *var2)
{
  double t316;
  double t1045;
  double t974;
  double t977;
  double t990;
  double t1154;
  double t1052;
  double t1125;
  double t1160;
  double t317;
  double t464;
  double t1148;
  double t1290;
  double t1320;
  double t1447;
  double t1455;
  double t1462;
  double t1898;
  double t1596;
  double t1599;
  double t1896;
  double t1951;
  double t1955;
  double t1970;
  double t1982;
  double t1988;
  double t1330;
  double t1353;
  double t1963;
  double t1989;
  double t1997;
  double t2090;
  double t2094;
  double t2101;
  double t1827;
  double t1828;
  double t1790;
  double t1792;
  double t2224;
  double t2231;
  double t2232;
  double t2219;
  double t2233;
  double t2250;
  double t2295;
  double t2308;
  double t2314;
  double t2693;
  double t2694;
  double t329;
  double t408;
  double t2879;
  double t2885;
  double t2896;
  double t2940;
  double t2944;
  double t2953;
  double t1567;
  double t1575;
  double t2930;
  double t2954;
  double t2958;
  double t2987;
  double t2996;
  double t3006;
  double t1707;
  double t1724;
  double t3226;
  double t3236;
  double t3245;
  double t3277;
  double t3279;
  double t3280;
  double t2260;
  double t3635;
  double t3636;
  double t3657;
  double t3677;
  double t3679;
  double t3704;
  double t3712;
  double t3718;
  double t3887;
  double t3899;
  double t3904;
  double t3875;
  double t3916;
  double t3952;
  double t3970;
  double t3972;
  double t3984;
  double t4163;
  double t4178;
  double t4353;
  double t4368;
  double t4373;
  double t4392;
  double t4393;
  double t4401;
  double t4529;
  double t4530;
  double t4544;
  double t4585;
  double t4590;
  double t4601;
  double t5025;
  double t5039;
  double t5046;
  double t5068;
  double t5096;
  double t5108;
  double t556;
  double t1326;
  double t1467;
  double t1552;
  double t1589;
  double t1611;
  double t1659;
  double t1687;
  double t1783;
  double t1805;
  double t1843;
  double t1870;
  double t1880;
  double t2064;
  double t2105;
  double t2106;
  double t2145;
  double t2148;
  double t2150;
  double t2178;
  double t2184;
  double t2185;
  double t2187;
  double t2396;
  double t2404;
  double t2411;
  double t2413;
  double t2423;
  double t2446;
  double t2448;
  double t2449;
  double t2463;
  double t2524;
  double t2638;
  double t2698;
  double t2708;
  double t2714;
  double t2741;
  double t2767;
  double t2810;
  double t2814;
  double t2820;
  double t2822;
  double t2823;
  double t2831;
  double t2868;
  double t2982;
  double t3012;
  double t3036;
  double t3071;
  double t3088;
  double t3141;
  double t3159;
  double t3172;
  double t3175;
  double t3182;
  double t3195;
  double t3198;
  double t3224;
  double t3275;
  double t3281;
  double t3316;
  double t3332;
  double t3338;
  double t3357;
  double t3358;
  double t3382;
  double t3389;
  double t3398;
  double t3400;
  double t3403;
  double t3439;
  double t3443;
  double t3453;
  double t3463;
  double t3469;
  double t3474;
  double t3515;
  double t3542;
  double t3588;
  double t3605;
  double t3626;
  double t3627;
  double t3690;
  double t3736;
  double t3737;
  double t3756;
  double t3768;
  double t3794;
  double t3801;
  double t3816;
  double t3840;
  double t3844;
  double t3967;
  double t3989;
  double t3990;
  double t4004;
  double t4014;
  double t4021;
  double t4038;
  double t4043;
  double t4058;
  double t4064;
  double t4104;
  double t4113;
  double t4182;
  double t4183;
  double t4187;
  double t4195;
  double t4228;
  double t4229;
  double t4255;
  double t4270;
  double t4272;
  double t4276;
  double t4287;
  double t4391;
  double t4422;
  double t4423;
  double t4428;
  double t4434;
  double t4451;
  double t4473;
  double t4475;
  double t4477;
  double t4482;
  double t4584;
  double t4613;
  double t4620;
  double t4629;
  double t4632;
  double t4634;
  double t4659;
  double t4681;
  double t4689;
  double t4694;
  double t4716;
  double t4765;
  double t4766;
  double t4771;
  double t4779;
  double t4807;
  double t4818;
  double t4867;
  double t4868;
  double t4893;
  double t4915;
  double t4920;
  double t4933;
  double t5012;
  double t5063;
  double t5110;
  double t5111;
  double t5120;
  double t5122;
  double t5132;
  double t5133;
  double t5138;
  double t5146;
  double t5148;
  double t5150;
  double t5152;
  double t5793;
  double t5794;
  double t5842;
  double t5847;
  double t5853;
  double t5799;
  double t5804;
  double t5805;
  double t5866;
  double t5884;
  double t5890;
  double t5895;
  double t5857;
  double t5858;
  double t5860;
  double t5863;
  double t5823;
  double t5827;
  double t5830;
  double t5831;
  double t5961;
  double t5965;
  double t5975;
  double t5978;
  double t5980;
  double t6008;
  double t6010;
  double t6011;
  double t6013;
  double t6014;
  double t6017;
  double t6027;
  double t6035;
  double t6036;
  t316 = Cos(var1[3]);
  t1045 = Cos(var1[4]);
  t974 = Cos(var1[25]);
  t977 = -1.*t974;
  t990 = 1. + t977;
  t1154 = Cos(var1[24]);
  t1052 = Cos(var1[5]);
  t1125 = Sin(var1[24]);
  t1160 = Sin(var1[5]);
  t317 = Sin(var1[25]);
  t464 = Sin(var1[4]);
  t1148 = -1.*t316*t1045*t1052*t1125;
  t1290 = t1154*t316*t1045*t1160;
  t1320 = t1148 + t1290;
  t1447 = t1154*t316*t1045*t1052;
  t1455 = t316*t1045*t1125*t1160;
  t1462 = t1447 + t1455;
  t1898 = Sin(var1[3]);
  t1596 = -0.010926102783999999*t990;
  t1599 = 1. + t1596;
  t1896 = t316*t1052*t464;
  t1951 = t1898*t1160;
  t1955 = t1896 + t1951;
  t1970 = t1052*t1898;
  t1982 = -1.*t316*t464*t1160;
  t1988 = t1970 + t1982;
  t1330 = -0.9890740084840001*t990;
  t1353 = 1. + t1330;
  t1963 = t1125*t1955;
  t1989 = t1154*t1988;
  t1997 = t1963 + t1989;
  t2090 = t1154*t1955;
  t2094 = -1.*t1125*t1988;
  t2101 = t2090 + t2094;
  t1827 = 0.994522*t317;
  t1828 = 0. + t1827;
  t1790 = 0.104528*t317;
  t1792 = 0. + t1790;
  t2224 = -1.*t1052*t1898;
  t2231 = t316*t464*t1160;
  t2232 = t2224 + t2231;
  t2219 = -1.*t1125*t1955;
  t2233 = t1154*t2232;
  t2250 = t2219 + t2233;
  t2295 = -1.*t1154*t1955;
  t2308 = -1.*t1125*t2232;
  t2314 = t2295 + t2308;
  t2693 = t1125*t2232;
  t2694 = t2090 + t2693;
  t329 = -0.994522*t317;
  t408 = 0. + t329;
  t2879 = -1.*t1052*t1898*t464;
  t2885 = t316*t1160;
  t2896 = t2879 + t2885;
  t2940 = -1.*t316*t1052;
  t2944 = -1.*t1898*t464*t1160;
  t2953 = t2940 + t2944;
  t1567 = -0.104528*t317;
  t1575 = 0. + t1567;
  t2930 = -1.*t1125*t2896;
  t2954 = t1154*t2953;
  t2958 = t2930 + t2954;
  t2987 = t1154*t2896;
  t2996 = t1125*t2953;
  t3006 = t2987 + t2996;
  t1707 = -1.0000001112680001*t990;
  t1724 = 1. + t1707;
  t3226 = -1.*t1045*t1052*t1125*t1898;
  t3236 = t1154*t1045*t1898*t1160;
  t3245 = t3226 + t3236;
  t3277 = t1154*t1045*t1052*t1898;
  t3279 = t1045*t1125*t1898*t1160;
  t3280 = t3277 + t3279;
  t2260 = -0.103955395616*t990*t2250;
  t3635 = t1052*t1898*t464;
  t3636 = -1.*t316*t1160;
  t3657 = t3635 + t3636;
  t3677 = t1125*t3657;
  t3679 = t3677 + t2954;
  t3704 = t1154*t3657;
  t3712 = -1.*t1125*t2953;
  t3718 = t3704 + t3712;
  t3887 = t316*t1052;
  t3899 = t1898*t464*t1160;
  t3904 = t3887 + t3899;
  t3875 = -1.*t1125*t3657;
  t3916 = t1154*t3904;
  t3952 = t3875 + t3916;
  t3970 = -1.*t1154*t3657;
  t3972 = -1.*t1125*t3904;
  t3984 = t3970 + t3972;
  t4163 = t1125*t3904;
  t4178 = t3704 + t4163;
  t4353 = -1.*t1045*t1052*t1125;
  t4368 = t1154*t1045*t1160;
  t4373 = t4353 + t4368;
  t4392 = -1.*t1154*t1045*t1052;
  t4393 = -1.*t1045*t1125*t1160;
  t4401 = t4392 + t4393;
  t4529 = t1045*t1052*t1125;
  t4530 = -1.*t1154*t1045*t1160;
  t4544 = t4529 + t4530;
  t4585 = t1154*t1045*t1052;
  t4590 = t1045*t1125*t1160;
  t4601 = t4585 + t4590;
  t5025 = t1052*t1125*t464;
  t5039 = -1.*t1154*t464*t1160;
  t5046 = t5025 + t5039;
  t5068 = -1.*t1154*t1052*t464;
  t5096 = -1.*t1125*t464*t1160;
  t5108 = t5068 + t5096;
  t556 = -1.*t316*t408*t464;
  t1326 = -0.103955395616*t990*t1320;
  t1467 = t1353*t1462;
  t1552 = t556 + t1326 + t1467;
  t1589 = -1.*t316*t1575*t464;
  t1611 = t1599*t1320;
  t1659 = -0.103955395616*t990*t1462;
  t1687 = t1589 + t1611 + t1659;
  t1783 = -1.*t1724*t316*t464;
  t1805 = t1792*t1320;
  t1843 = t1828*t1462;
  t1870 = t1783 + t1805 + t1843;
  t1880 = 0.707107*t1870;
  t2064 = -0.103955395616*t990*t1997;
  t2105 = t1599*t2101;
  t2106 = t2064 + t2105;
  t2145 = t1353*t1997;
  t2148 = -0.103955395616*t990*t2101;
  t2150 = t2145 + t2148;
  t2178 = t1828*t1997;
  t2184 = t1792*t2101;
  t2185 = t2178 + t2184;
  t2187 = 0.707107*t2185;
  t2396 = t1599*t2314;
  t2404 = t2260 + t2396;
  t2411 = t1353*t2250;
  t2413 = -0.103955395616*t990*t2314;
  t2423 = t2411 + t2413;
  t2446 = t1828*t2250;
  t2448 = t1792*t2314;
  t2449 = t2446 + t2448;
  t2463 = 0.707107*t2449;
  t2524 = -1.0000001112680001*t316*t1045*t317;
  t2638 = 0.104528*t974*t2250;
  t2698 = 0.994522*t974*t2694;
  t2708 = t2524 + t2638 + t2698;
  t2714 = 0.707107*t2708;
  t2741 = -0.994522*t974*t316*t1045;
  t2767 = -0.103955395616*t317*t2250;
  t2810 = -0.9890740084840001*t317*t2694;
  t2814 = t2741 + t2767 + t2810;
  t2820 = -0.104528*t974*t316*t1045;
  t2822 = -0.010926102783999999*t317*t2250;
  t2823 = -0.103955395616*t317*t2694;
  t2831 = t2820 + t2822 + t2823;
  t2868 = -1.*t1045*t408*t1898;
  t2982 = -0.103955395616*t990*t2958;
  t3012 = t1353*t3006;
  t3036 = t2868 + t2982 + t3012;
  t3071 = -1.*t1045*t1575*t1898;
  t3088 = t1599*t2958;
  t3141 = -0.103955395616*t990*t3006;
  t3159 = t3071 + t3088 + t3141;
  t3172 = -1.*t1724*t1045*t1898;
  t3175 = t1792*t2958;
  t3182 = t1828*t3006;
  t3195 = t3172 + t3175 + t3182;
  t3198 = 0.707107*t3195;
  t3224 = -1.*t408*t1898*t464;
  t3275 = -0.103955395616*t990*t3245;
  t3281 = t1353*t3280;
  t3316 = t3224 + t3275 + t3281;
  t3332 = -1.*t1575*t1898*t464;
  t3338 = t1599*t3245;
  t3357 = -0.103955395616*t990*t3280;
  t3358 = t3332 + t3338 + t3357;
  t3382 = -1.*t1724*t1898*t464;
  t3389 = t1792*t3245;
  t3398 = t1828*t3280;
  t3400 = t3382 + t3389 + t3398;
  t3403 = 0.707107*t3400;
  t3439 = t316*t1045*t408;
  t3443 = t1353*t2694;
  t3453 = t3439 + t2260 + t3443;
  t3463 = t316*t1045*t1575;
  t3469 = t1599*t2250;
  t3474 = -0.103955395616*t990*t2694;
  t3515 = t3463 + t3469 + t3474;
  t3542 = t1724*t316*t1045;
  t3588 = t1792*t2250;
  t3605 = t1828*t2694;
  t3626 = t3542 + t3588 + t3605;
  t3627 = 0.707107*t3626;
  t3690 = -0.103955395616*t990*t3679;
  t3736 = t1599*t3718;
  t3737 = t3690 + t3736;
  t3756 = t1353*t3679;
  t3768 = -0.103955395616*t990*t3718;
  t3794 = t3756 + t3768;
  t3801 = t1828*t3679;
  t3816 = t1792*t3718;
  t3840 = t3801 + t3816;
  t3844 = 0.707107*t3840;
  t3967 = -0.103955395616*t990*t3952;
  t3989 = t1599*t3984;
  t3990 = t3967 + t3989;
  t4004 = t1353*t3952;
  t4014 = -0.103955395616*t990*t3984;
  t4021 = t4004 + t4014;
  t4038 = t1828*t3952;
  t4043 = t1792*t3984;
  t4058 = t4038 + t4043;
  t4064 = 0.707107*t4058;
  t4104 = -1.0000001112680001*t1045*t317*t1898;
  t4113 = 0.104528*t974*t3952;
  t4182 = 0.994522*t974*t4178;
  t4183 = t4104 + t4113 + t4182;
  t4187 = 0.707107*t4183;
  t4195 = -0.994522*t974*t1045*t1898;
  t4228 = -0.103955395616*t317*t3952;
  t4229 = -0.9890740084840001*t317*t4178;
  t4255 = t4195 + t4228 + t4229;
  t4270 = -0.104528*t974*t1045*t1898;
  t4272 = -0.010926102783999999*t317*t3952;
  t4276 = -0.103955395616*t317*t4178;
  t4287 = t4270 + t4272 + t4276;
  t4391 = -0.103955395616*t990*t4373;
  t4422 = t1599*t4401;
  t4423 = t4391 + t4422;
  t4428 = t1353*t4373;
  t4434 = -0.103955395616*t990*t4401;
  t4451 = t4428 + t4434;
  t4473 = t1828*t4373;
  t4475 = t1792*t4401;
  t4477 = t4473 + t4475;
  t4482 = 0.707107*t4477;
  t4584 = -0.103955395616*t990*t4544;
  t4613 = t1599*t4601;
  t4620 = t4584 + t4613;
  t4629 = t1353*t4544;
  t4632 = -0.103955395616*t990*t4601;
  t4634 = t4629 + t4632;
  t4659 = t1828*t4544;
  t4681 = t1792*t4601;
  t4689 = t4659 + t4681;
  t4694 = 0.707107*t4689;
  t4716 = 1.0000001112680001*t317*t464;
  t4765 = 0.104528*t974*t4373;
  t4766 = 0.994522*t974*t4601;
  t4771 = t4716 + t4765 + t4766;
  t4779 = 0.707107*t4771;
  t4807 = 0.994522*t974*t464;
  t4818 = -0.103955395616*t317*t4373;
  t4867 = -0.9890740084840001*t317*t4601;
  t4868 = t4807 + t4818 + t4867;
  t4893 = 0.104528*t974*t464;
  t4915 = -0.010926102783999999*t317*t4373;
  t4920 = -0.103955395616*t317*t4601;
  t4933 = t4893 + t4915 + t4920;
  t5012 = -1.*t1045*t408;
  t5063 = -0.103955395616*t990*t5046;
  t5110 = t1353*t5108;
  t5111 = t5012 + t5063 + t5110;
  t5120 = -1.*t1045*t1575;
  t5122 = t1599*t5046;
  t5132 = -0.103955395616*t990*t5108;
  t5133 = t5120 + t5122 + t5132;
  t5138 = -1.*t1724*t1045;
  t5146 = t1792*t5046;
  t5148 = t1828*t5108;
  t5150 = t5138 + t5146 + t5148;
  t5152 = 0.707107*t5150;
  t5793 = -1.*t1154;
  t5794 = 1. + t5793;
  t5842 = -0.12*t5794;
  t5847 = -0.4*t1125;
  t5853 = 0. + t5842 + t5847;
  t5799 = 0.4*t5794;
  t5804 = -0.12*t1125;
  t5805 = 0. + t5799 + t5804;
  t5866 = -1.1924972351948546e-8*var1[25];
  t5884 = 0.38315655000705834*t990;
  t5890 = -0.05650052807*t408;
  t5895 = t5866 + t5884 + t5890;
  t5857 = 1.1345904784751044e-7*var1[25];
  t5858 = 0.04027119345689465*t990;
  t5860 = -0.05650052807*t1575;
  t5863 = t5857 + t5858 + t5860;
  t5823 = -0.056500534356700764*t990;
  t5827 = 0.040271188976*t1792;
  t5830 = 0.38315650737400003*t1828;
  t5831 = 0. + t5823 + t5827 + t5830;
  t5961 = -0.12*t1154;
  t5965 = 0.4*t1125;
  t5975 = t5961 + t5965;
  t5978 = -0.4*t1154;
  t5980 = t5978 + t5804;
  t6008 = 0.3852670428678886*t974;
  t6010 = -0.056500534356700764*t317;
  t6011 = t6008 + t6010;
  t6013 = 0.0059058871981009595*t974;
  t6014 = 0.04027119345689465*t317;
  t6017 = 1.1345904784751044e-7 + t6013 + t6014;
  t6027 = 0.05619101817723254*t974;
  t6035 = 0.38315655000705834*t317;
  t6036 = -1.1924972351948546e-8 + t6027 + t6035;
  p_output1[0]=(0.703234*t3036 + 0.073913*t3159 + t3198)*var2[3] + (0.703234*t1552 + 0.073913*t1687 + t1880)*var2[4] + (0.073913*t2106 + 0.703234*t2150 + t2187)*var2[5] + (0.073913*t2404 + 0.703234*t2423 + t2463)*var2[24] + (t2714 + 0.703234*t2814 + 0.073913*t2831)*var2[25];
  p_output1[1]=(0.703234*t3453 + 0.073913*t3515 + t3627)*var2[3] + (0.703234*t3316 + 0.073913*t3358 + t3403)*var2[4] + (0.073913*t3737 + 0.703234*t3794 + t3844)*var2[5] + (0.073913*t3990 + 0.703234*t4021 + t4064)*var2[24] + (t4187 + 0.703234*t4255 + 0.073913*t4287)*var2[25];
  p_output1[2]=(0.703234*t5111 + 0.073913*t5133 + t5152)*var2[4] + (0.073913*t4620 + 0.703234*t4634 + t4694)*var2[5] + (0.073913*t4423 + 0.703234*t4451 + t4482)*var2[24] + (t4779 + 0.703234*t4868 + 0.073913*t4933)*var2[25];
  p_output1[3]=0;
  p_output1[4]=(-0.703234*t3036 - 0.073913*t3159 + t3198)*var2[3] + (-0.703234*t1552 - 0.073913*t1687 + t1880)*var2[4] + (-0.073913*t2106 - 0.703234*t2150 + t2187)*var2[5] + (-0.073913*t2404 - 0.703234*t2423 + t2463)*var2[24] + (t2714 - 0.703234*t2814 - 0.073913*t2831)*var2[25];
  p_output1[5]=(-0.703234*t3453 - 0.073913*t3515 + t3627)*var2[3] + (-0.703234*t3316 - 0.073913*t3358 + t3403)*var2[4] + (-0.073913*t3737 - 0.703234*t3794 + t3844)*var2[5] + (-0.073913*t3990 - 0.703234*t4021 + t4064)*var2[24] + (t4187 - 0.703234*t4255 - 0.073913*t4287)*var2[25];
  p_output1[6]=(-0.703234*t5111 - 0.073913*t5133 + t5152)*var2[4] + (-0.073913*t4620 - 0.703234*t4634 + t4694)*var2[5] + (-0.073913*t4423 - 0.703234*t4451 + t4482)*var2[24] + (t4779 - 0.703234*t4868 - 0.073913*t4933)*var2[25];
  p_output1[7]=0;
  p_output1[8]=(-0.104528*t3036 + 0.994522*t3159)*var2[3] + (-0.104528*t1552 + 0.994522*t1687)*var2[4] + (0.994522*t2106 - 0.104528*t2150)*var2[5] + (0.994522*t2404 - 0.104528*t2423)*var2[24] + (-0.104528*t2814 + 0.994522*t2831)*var2[25];
  p_output1[9]=(-0.104528*t3453 + 0.994522*t3515)*var2[3] + (-0.104528*t3316 + 0.994522*t3358)*var2[4] + (0.994522*t3737 - 0.104528*t3794)*var2[5] + (0.994522*t3990 - 0.104528*t4021)*var2[24] + (-0.104528*t4255 + 0.994522*t4287)*var2[25];
  p_output1[10]=(-0.104528*t5111 + 0.994522*t5133)*var2[4] + (0.994522*t4620 - 0.104528*t4634)*var2[5] + (0.994522*t4423 - 0.104528*t4451)*var2[24] + (-0.104528*t4868 + 0.994522*t4933)*var2[25];
  p_output1[11]=0;
  p_output1[12]=var2[0] + (0.398799*t3036 - 0.108558*t3159 - 0.0565*t3195 + t2896*t5805 - 1.*t1045*t1898*t5831 + t2953*t5853 + t2958*t5863 + t3006*t5895)*var2[3] + (0.398799*t1552 - 0.108558*t1687 - 0.0565*t1870 + t1045*t1052*t316*t5805 - 1.*t316*t464*t5831 + t1045*t1160*t316*t5853 + t1320*t5863 + t1462*t5895)*var2[4] + (-0.108558*t2106 + 0.398799*t2150 - 0.0565*t2185 + t1988*t5805 + t1955*t5853 + t2101*t5863 + t1997*t5895)*var2[5] + (-0.108558*t2404 + 0.398799*t2423 - 0.0565*t2449 + t2314*t5863 + t2250*t5895 + t1955*t5975 + t2232*t5980)*var2[24] + (-0.0565*t2708 + 0.398799*t2814 - 0.108558*t2831 + t1045*t316*t6011 + t2250*t6017 + t2694*t6036)*var2[25];
  p_output1[13]=var2[1] + (0.398799*t3453 - 0.108558*t3515 - 0.0565*t3626 + t1955*t5805 + t1045*t316*t5831 + t2232*t5853 + t2250*t5863 + t2694*t5895)*var2[3] + (0.398799*t3316 - 0.108558*t3358 - 0.0565*t3400 + t1045*t1052*t1898*t5805 - 1.*t1898*t464*t5831 + t1045*t1160*t1898*t5853 + t3245*t5863 + t3280*t5895)*var2[4] + (-0.108558*t3737 + 0.398799*t3794 - 0.0565*t3840 + t2953*t5805 + t3657*t5853 + t3718*t5863 + t3679*t5895)*var2[5] + (-0.108558*t3990 + 0.398799*t4021 - 0.0565*t4058 + t3984*t5863 + t3952*t5895 + t3657*t5975 + t3904*t5980)*var2[24] + (-0.0565*t4183 + 0.398799*t4255 - 0.108558*t4287 + t1045*t1898*t6011 + t3952*t6017 + t4178*t6036)*var2[25];
  p_output1[14]=var2[2] + (0.398799*t5111 - 0.108558*t5133 - 0.0565*t5150 - 1.*t1052*t464*t5805 - 1.*t1045*t5831 - 1.*t1160*t464*t5853 + t5046*t5863 + t5108*t5895)*var2[4] + (-0.108558*t4620 + 0.398799*t4634 - 0.0565*t4689 - 1.*t1045*t1160*t5805 + t1045*t1052*t5853 + t4601*t5863 + t4544*t5895)*var2[5] + (-0.108558*t4423 + 0.398799*t4451 - 0.0565*t4477 + t4401*t5863 + t4373*t5895 + t1045*t1052*t5975 + t1045*t1160*t5980)*var2[24] + (-0.0565*t4771 + 0.398799*t4868 - 0.108558*t4933 - 1.*t464*t6011 + t4373*t6017 + t4601*t6036)*var2[25];
  p_output1[15]=0;
}



void dT_shoulder_pitch_joint_right_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
