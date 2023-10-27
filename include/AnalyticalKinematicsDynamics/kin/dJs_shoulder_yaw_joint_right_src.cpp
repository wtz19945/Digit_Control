/*
 * Automatically Generated from Mathematica.
 * Mon 4 Jul 2022 20:55:30 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "dJs_shoulder_yaw_joint_right_src.h"

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
  double t26;
  double t54;
  double t201;
  double t214;
  double t782;
  double t830;
  double t877;
  double t879;
  double t883;
  double t1051;
  double t1053;
  double t1060;
  double t1717;
  double t1693;
  double t1349;
  double t1359;
  double t1364;
  double t1911;
  double t1912;
  double t1916;
  double t981;
  double t987;
  double t1028;
  double t2066;
  double t2079;
  double t1243;
  double t1289;
  double t1300;
  double t1695;
  double t1740;
  double t1742;
  double t2179;
  double t2211;
  double t2236;
  double t2257;
  double t2261;
  double t2299;
  double t1772;
  double t1783;
  double t1784;
  double t2393;
  double t2419;
  double t2420;
  double t2058;
  double t2249;
  double t2311;
  double t2321;
  double t932;
  double t942;
  double t951;
  double t2522;
  double t1841;
  double t2021;
  double t2658;
  double t2664;
  double t2665;
  double t2786;
  double t2802;
  double t2806;
  double t2744;
  double t2746;
  double t2759;
  double t2721;
  double t2330;
  double t2338;
  double t2340;
  double t2761;
  double t2771;
  double t2780;
  double t3060;
  double t3063;
  double t3064;
  double t3076;
  double t3099;
  double t1934;
  double t1937;
  double t1977;
  double t3168;
  double t3174;
  double t3186;
  double t3075;
  double t3119;
  double t3122;
  double t2967;
  double t2980;
  double t3413;
  double t3416;
  double t3430;
  double t3447;
  double t3456;
  double t3467;
  double t2894;
  double t2917;
  double t2941;
  double t3149;
  double t3159;
  double t3162;
  double t2526;
  double t2552;
  double t2880;
  double t2728;
  double t2736;
  double t2647;
  double t2651;
  double t3716;
  double t3723;
  double t3724;
  double t2633;
  double t2640;
  double t2643;
  double t3706;
  double t3709;
  double t3710;
  double t3759;
  double t3765;
  double t3786;
  double t3802;
  double t3734;
  double t3735;
  double t3736;
  double t3031;
  double t3032;
  double t3047;
  double t3980;
  double t3988;
  double t4006;
  double t3937;
  double t3945;
  double t3948;
  double t3383;
  double t3384;
  double t3386;
  double t3533;
  double t3535;
  double t3544;
  double t3351;
  double t3353;
  double t3354;
  double t3487;
  double t3489;
  double t3497;
  double t4090;
  double t4091;
  double t4092;
  double t3361;
  double t3372;
  double t3380;
  double t3547;
  double t3552;
  double t3559;
  double t4098;
  double t4106;
  double t4109;
  double t2015;
  double t2029;
  double t4243;
  double t4259;
  double t4262;
  double t2036;
  double t2037;
  double t2046;
  double t4452;
  double t4417;
  double t4436;
  double t4443;
  double t4484;
  double t4490;
  double t4501;
  double t4502;
  double t4458;
  double t4460;
  double t4464;
  double t4579;
  double t4586;
  double t4592;
  double t4604;
  double t4612;
  double t4616;
  double t4696;
  double t4700;
  double t4701;
  double t4673;
  double t4679;
  double t4680;
  double t4770;
  double t4787;
  double t4788;
  double t4790;
  double t3687;
  double t3693;
  double t3698;
  double t5129;
  double t5131;
  double t5137;
  double t5122;
  double t5124;
  double t5125;
  double t5139;
  double t5141;
  double t5162;
  double t5164;
  double t5168;
  double t5172;
  double t5185;
  double t5188;
  double t5198;
  double t5206;
  double t5218;
  double t5221;
  double t5244;
  double t5294;
  double t5354;
  double t5126;
  double t5138;
  double t5143;
  double t5145;
  double t5497;
  double t5501;
  double t5518;
  double t5521;
  double t5540;
  double t5548;
  double t5551;
  double t5556;
  double t5581;
  double t5584;
  double t5601;
  double t5614;
  double t5528;
  double t5562;
  double t5616;
  double t5621;
  double t5165;
  double t5173;
  double t5176;
  double t5179;
  double t5649;
  double t5651;
  double t5656;
  double t5669;
  double t5197;
  double t5210;
  double t5223;
  double t5228;
  double t5624;
  double t5915;
  double t5918;
  double t5919;
  double t5928;
  double t5937;
  double t5945;
  double t5909;
  double t5973;
  double t5976;
  double t5977;
  double t5982;
  double t5946;
  double t5958;
  double t5960;
  double t5963;
  double t6290;
  double t6301;
  double t6304;
  double t6308;
  double t6332;
  double t6350;
  double t6022;
  double t6028;
  double t6032;
  double t6156;
  double t6164;
  double t6172;
  double t6173;
  double t6644;
  double t6646;
  double t6648;
  double t6650;
  double t6652;
  double t6653;
  double t6658;
  double t6660;
  double t6929;
  double t6932;
  double t6936;
  double t6940;
  double t6942;
  double t6945;
  double t6948;
  double t6950;
  double t6955;
  double t6982;
  double t6984;
  double t6986;
  double t6999;
  double t6939;
  double t6946;
  double t6959;
  double t6962;
  double t5622;
  double t5629;
  double t5642;
  double t5679;
  double t5683;
  double t5686;
  double t5696;
  double t5780;
  double t5788;
  double t5800;
  double t5804;
  double t5910;
  double t5912;
  double t6000;
  double t6001;
  double t6002;
  double t6066;
  double t6102;
  double t6127;
  double t5852;
  double t5855;
  double t5970;
  double t5971;
  double t7201;
  double t7203;
  double t7207;
  double t7211;
  double t5877;
  double t5880;
  double t5884;
  double t7229;
  double t7230;
  double t7231;
  double t7235;
  double t7236;
  double t7237;
  double t6038;
  double t6039;
  double t6041;
  double t7254;
  double t7256;
  double t7258;
  double t7262;
  double t5888;
  double t5895;
  double t5896;
  double t6136;
  double t6144;
  double t6145;
  double t6265;
  double t6266;
  double t6272;
  double t6400;
  double t6407;
  double t6408;
  double t6448;
  double t6453;
  double t6460;
  double t6213;
  double t6215;
  double t6218;
  double t6356;
  double t6359;
  double t6364;
  double t6232;
  double t6233;
  double t6245;
  double t7325;
  double t7326;
  double t7327;
  double t7329;
  double t7330;
  double t7331;
  double t6420;
  double t6429;
  double t6251;
  double t6258;
  double t6259;
  double t6471;
  double t6476;
  double t6482;
  double t6627;
  double t6633;
  double t6637;
  double t6639;
  double t6711;
  double t6712;
  double t6739;
  double t6742;
  double t6803;
  double t6804;
  double t6816;
  double t6817;
  double t6552;
  double t6554;
  double t6567;
  double t6574;
  double t6666;
  double t6671;
  double t6678;
  double t6680;
  double t6588;
  double t6599;
  double t6600;
  double t6604;
  double t7437;
  double t7438;
  double t7441;
  double t7444;
  double t7451;
  double t7465;
  double t7466;
  double t7468;
  double t6775;
  double t6778;
  double t6783;
  double t6785;
  double t6607;
  double t6610;
  double t6618;
  double t6621;
  double t6823;
  double t6824;
  double t6832;
  double t6833;
  double t6901;
  double t6902;
  double t6903;
  double t6911;
  double t7051;
  double t7054;
  double t7055;
  double t7057;
  double t7091;
  double t7097;
  double t7114;
  double t7118;
  double t6849;
  double t6850;
  double t6851;
  double t6856;
  double t6916;
  double t6922;
  double t6923;
  double t6926;
  double t7542;
  double t7544;
  double t7545;
  double t7546;
  double t7022;
  double t7023;
  double t7035;
  double t7040;
  double t7522;
  double t7531;
  double t7532;
  double t7540;
  double t6858;
  double t6861;
  double t6862;
  double t6868;
  double t7077;
  double t7078;
  double t7079;
  double t7081;
  double t6872;
  double t6874;
  double t6879;
  double t6882;
  double t5422;
  double t5429;
  double t5430;
  double t5453;
  double t5454;
  double t5456;
  double t7595;
  double t7596;
  double t7597;
  double t5458;
  double t5468;
  double t5490;
  double t7688;
  double t7690;
  double t7691;
  double t7694;
  double t7698;
  double t7701;
  double t7702;
  double t7703;
  double t7704;
  double t7717;
  double t7718;
  double t7725;
  double t7730;
  double t7783;
  double t7784;
  double t7785;
  double t7787;
  double t7789;
  double t7790;
  double t7844;
  double t7845;
  double t7849;
  double t7855;
  double t7860;
  double t7862;
  double t7899;
  double t7902;
  double t7903;
  double t7909;
  double t7968;
  double t7983;
  double t7984;
  double t7986;
  double t7960;
  double t7961;
  double t7962;
  double t7963;
  double t7180;
  double t7181;
  double t7185;
  double t7187;
  t26 = Cos(var1[3]);
  t54 = Sin(var1[3]);
  t201 = Cos(var1[4]);
  t214 = Sin(var1[4]);
  t782 = Cos(var1[5]);
  t830 = Sin(var1[5]);
  t877 = t26*t782*t214;
  t879 = t54*t830;
  t883 = t877 + t879;
  t1051 = -1.*t26*t782;
  t1053 = -1.*t54*t214*t830;
  t1060 = t1051 + t1053;
  t1717 = Cos(var1[24]);
  t1693 = Sin(var1[24]);
  t1349 = t782*t54*t214;
  t1359 = -1.*t26*t830;
  t1364 = t1349 + t1359;
  t1911 = t26*t782;
  t1912 = t54*t214*t830;
  t1916 = t1911 + t1912;
  t981 = -1.*t782*t54*t214;
  t987 = t26*t830;
  t1028 = t981 + t987;
  t2066 = -1.*t1717;
  t2079 = 1. + t2066;
  t1243 = -1.*t782*t54;
  t1289 = t26*t214*t830;
  t1300 = t1243 + t1289;
  t1695 = -1.*t201*t782*t1693;
  t1740 = t1717*t201*t830;
  t1742 = t1695 + t1740;
  t2179 = 0.4*t2079;
  t2211 = -0.12*t1693;
  t2236 = 0. + t2179 + t2211;
  t2257 = -0.12*t2079;
  t2261 = -0.4*t1693;
  t2299 = 0. + t2257 + t2261;
  t1772 = t1717*t201*t782;
  t1783 = t201*t1693*t830;
  t1784 = t1772 + t1783;
  t2393 = t2236*t883;
  t2419 = t2299*t1300;
  t2420 = t2393 + t2419;
  t2058 = -1.*var1[2];
  t2249 = -1.*t201*t782*t2236;
  t2311 = -1.*t201*t2299*t830;
  t2321 = 0. + t2058 + t2249 + t2311;
  t932 = t782*t54;
  t942 = -1.*t26*t214*t830;
  t951 = t932 + t942;
  t2522 = t1717*t883;
  t1841 = t1693*t1364;
  t2021 = t1717*t1060;
  t2658 = t2299*t1364;
  t2664 = t2236*t1060;
  t2665 = t2658 + t2664;
  t2786 = t2236*t1364;
  t2802 = t2299*t1916;
  t2806 = 0. + var1[1] + t2786 + t2802;
  t2744 = -1.*t201*t782*t2299;
  t2746 = t201*t2236*t830;
  t2759 = t2744 + t2746;
  t2721 = t1717*t1364;
  t2330 = -1.*t1693*t883;
  t2338 = t1717*t1300;
  t2340 = t2330 + t2338;
  t2761 = -1.*t1693*t1364;
  t2771 = t1717*t1916;
  t2780 = t2761 + t2771;
  t3060 = -0.12*t1717;
  t3063 = 0.4*t1693;
  t3064 = t3060 + t3063;
  t3076 = -0.4*t1717;
  t3099 = t3076 + t2211;
  t1934 = -1.*t1717*t1364;
  t1937 = -1.*t1693*t1916;
  t1977 = t1934 + t1937;
  t3168 = t3064*t1364;
  t3174 = t3099*t1916;
  t3186 = t3168 + t3174;
  t3075 = -1.*t201*t782*t3064;
  t3119 = -1.*t201*t3099*t830;
  t3122 = t3075 + t3119;
  t2967 = t1693*t1916;
  t2980 = t2721 + t2967;
  t3413 = t201*t782*t2236*t54;
  t3416 = t201*t2299*t54*t830;
  t3430 = t3413 + t3416;
  t3447 = t782*t2236*t214;
  t3456 = t2299*t214*t830;
  t3467 = t3447 + t3456;
  t2894 = t201*t782*t1693;
  t2917 = -1.*t1717*t201*t830;
  t2941 = t2894 + t2917;
  t3149 = -1.*t1717*t201*t782;
  t3159 = -1.*t201*t1693*t830;
  t3162 = t3149 + t3159;
  t2526 = t1693*t1300;
  t2552 = t2522 + t2526;
  t2880 = t1841 + t2021;
  t2728 = -1.*t1693*t1060;
  t2736 = t2721 + t2728;
  t2647 = -1.*t1693*t951;
  t2651 = t2522 + t2647;
  t3716 = t201*t782*t2236;
  t3723 = t201*t2299*t830;
  t3724 = 0. + var1[2] + t3716 + t3723;
  t2633 = t1693*t883;
  t2640 = t1717*t951;
  t2643 = t2633 + t2640;
  t3706 = -1.*t2299*t883;
  t3709 = -1.*t2236*t951;
  t3710 = t3706 + t3709;
  t3759 = -1.*var1[0];
  t3765 = -1.*t2236*t883;
  t3786 = -1.*t2299*t1300;
  t3802 = 0. + t3759 + t3765 + t3786;
  t3734 = t201*t782*t2299;
  t3735 = -1.*t201*t2236*t830;
  t3736 = t3734 + t3735;
  t3031 = -1.*t1717*t883;
  t3032 = -1.*t1693*t1300;
  t3047 = t3031 + t3032;
  t3980 = -1.*t3064*t883;
  t3988 = -1.*t3099*t1300;
  t4006 = t3980 + t3988;
  t3937 = t201*t782*t3064;
  t3945 = t201*t3099*t830;
  t3948 = t3937 + t3945;
  t3383 = -1.*t201*t782*t1693*t54;
  t3384 = t1717*t201*t54*t830;
  t3386 = t3383 + t3384;
  t3533 = t1717*t201*t782*t54;
  t3535 = t201*t1693*t54*t830;
  t3544 = t3533 + t3535;
  t3351 = -1.*t26*t201*t782*t1693;
  t3353 = t1717*t26*t201*t830;
  t3354 = t3351 + t3353;
  t3487 = t782*t1693*t214;
  t3489 = -1.*t1717*t214*t830;
  t3497 = t3487 + t3489;
  t4090 = -1.*t26*t201*t782*t2236;
  t4091 = -1.*t26*t201*t2299*t830;
  t4092 = t4090 + t4091;
  t3361 = t1717*t26*t201*t782;
  t3372 = t26*t201*t1693*t830;
  t3380 = t3361 + t3372;
  t3547 = -1.*t1717*t782*t214;
  t3552 = -1.*t1693*t214*t830;
  t3559 = t3547 + t3552;
  t4098 = -1.*t782*t2236*t214;
  t4106 = -1.*t2299*t214*t830;
  t4109 = t4098 + t4106;
  t2015 = -1.*t1693*t1028;
  t2029 = t2015 + t2021;
  t4243 = -1.*t2236*t1028;
  t4259 = -1.*t2299*t1060;
  t4262 = t4243 + t4259;
  t2036 = t1717*t1028;
  t2037 = t1693*t1060;
  t2046 = t2036 + t2037;
  t4452 = 0. + var1[0] + t2393 + t2419;
  t4417 = -1.*t201*t782*t2236*t54;
  t4436 = -1.*t201*t2299*t54*t830;
  t4443 = t4417 + t4436;
  t4484 = -1.*var1[1];
  t4490 = -1.*t2236*t1364;
  t4501 = -1.*t2299*t1916;
  t4502 = 0. + t4484 + t4490 + t4501;
  t4458 = t26*t201*t782*t2236;
  t4460 = t26*t201*t2299*t830;
  t4464 = t4458 + t4460;
  t4579 = -1.*t2299*t1364;
  t4586 = -1.*t2236*t1060;
  t4592 = t4579 + t4586;
  t4604 = t2299*t883;
  t4612 = t2236*t951;
  t4616 = t4604 + t4612;
  t4696 = -1.*t3064*t1364;
  t4700 = -1.*t3099*t1916;
  t4701 = t4696 + t4700;
  t4673 = t3064*t883;
  t4679 = t3099*t1300;
  t4680 = t4673 + t4679;
  t4770 = t3765 + t3786;
  t4787 = t2236*t1028;
  t4788 = t2299*t1060;
  t4790 = t4787 + t4788;
  t3687 = -0.994522*t2340;
  t3693 = 0.104528*t2552;
  t3698 = t3687 + t3693;
  t5129 = Cos(var1[25]);
  t5131 = -1.*t5129;
  t5137 = 1. + t5131;
  t5122 = Sin(var1[25]);
  t5124 = -0.994522*t5122;
  t5125 = 0. + t5124;
  t5139 = -0.9890740084840001*t5137;
  t5141 = 1. + t5139;
  t5162 = -0.104528*t5122;
  t5164 = 0. + t5162;
  t5168 = -0.010926102783999999*t5137;
  t5172 = 1. + t5168;
  t5185 = -1.0000001112680001*t5137;
  t5188 = 1. + t5185;
  t5198 = 0.104528*t5122;
  t5206 = 0. + t5198;
  t5218 = 0.994522*t5122;
  t5221 = 0. + t5218;
  t5244 = -1.*t201*t5125*t54;
  t5294 = -1.*t201*t5164*t54;
  t5354 = -1.*t5188*t201*t54;
  t5126 = -1.*t5125*t214;
  t5138 = -0.103955395616*t5137*t1742;
  t5143 = t5141*t1784;
  t5145 = t5126 + t5138 + t5143;
  t5497 = -0.056500534356700764*t5137;
  t5501 = 0.040271188976*t5206;
  t5518 = 0.38315650737400003*t5221;
  t5521 = 0. + t5497 + t5501 + t5518;
  t5540 = 1.1345904784751044e-7*var1[25];
  t5548 = 0.04027119345689465*t5137;
  t5551 = -0.05650052807*t5164;
  t5556 = t5540 + t5548 + t5551;
  t5581 = -1.1924972351948546e-8*var1[25];
  t5584 = 0.38315655000705834*t5137;
  t5601 = -0.05650052807*t5125;
  t5614 = t5581 + t5584 + t5601;
  t5528 = t5521*t214;
  t5562 = -1.*t5556*t1742;
  t5616 = -1.*t5614*t1784;
  t5621 = 0. + t2058 + t2249 + t5528 + t2311 + t5562 + t5616;
  t5165 = -1.*t5164*t214;
  t5173 = t5172*t1742;
  t5176 = -0.103955395616*t5137*t1784;
  t5179 = t5165 + t5173 + t5176;
  t5649 = t26*t201*t5521;
  t5651 = t5556*t2340;
  t5656 = t5614*t2552;
  t5669 = t5649 + t2393 + t2419 + t5651 + t5656;
  t5197 = -1.*t5188*t214;
  t5210 = t5206*t1742;
  t5223 = t5221*t1784;
  t5228 = t5197 + t5210 + t5223;
  t5624 = -0.103955395616*t5137*t2340;
  t5915 = t5614*t2780;
  t5918 = t5556*t1977;
  t5919 = t3168 + t3174 + t5915 + t5918;
  t5928 = -1.*t5614*t1742;
  t5937 = -1.*t5556*t3162;
  t5945 = t3075 + t3119 + t5928 + t5937;
  t5909 = -0.103955395616*t5137*t2780;
  t5973 = t201*t5521*t54;
  t5976 = t5556*t2780;
  t5977 = t5614*t2980;
  t5982 = 0. + var1[1] + t5973 + t2786 + t2802 + t5976 + t5977;
  t5946 = t201*t5164*t54;
  t5958 = t5172*t2780;
  t5960 = -0.103955395616*t5137*t2980;
  t5963 = t5946 + t5958 + t5960;
  t6290 = t5614*t2880;
  t6301 = t5556*t2736;
  t6304 = t2658 + t2664 + t6290 + t6301;
  t6308 = -1.*t5614*t2941;
  t6332 = -1.*t5556*t1784;
  t6350 = t2744 + t2746 + t6308 + t6332;
  t6022 = t201*t5125*t54;
  t6028 = t5141*t2980;
  t6032 = t6022 + t5909 + t6028;
  t6156 = t5188*t201*t54;
  t6164 = t5206*t2780;
  t6172 = t5221*t2980;
  t6173 = t6156 + t6164 + t6172;
  t6644 = -1.*t5521*t54*t214;
  t6646 = t5556*t3386;
  t6648 = t5614*t3544;
  t6650 = t3413 + t6644 + t3416 + t6646 + t6648;
  t6652 = t201*t5521;
  t6653 = -1.*t5556*t3497;
  t6658 = -1.*t5614*t3559;
  t6660 = t6652 + t3447 + t3456 + t6653 + t6658;
  t6929 = 0.3852670428678886*t5129;
  t6932 = -0.056500534356700764*t5122;
  t6936 = t6929 + t6932;
  t6940 = 0.0059058871981009595*t5129;
  t6942 = 0.04027119345689465*t5122;
  t6945 = 1.1345904784751044e-7 + t6940 + t6942;
  t6948 = 0.05619101817723254*t5129;
  t6950 = 0.38315655000705834*t5122;
  t6955 = -1.1924972351948546e-8 + t6948 + t6950;
  t6982 = t6936*t214;
  t6984 = -1.*t6945*t1742;
  t6986 = -1.*t6955*t1784;
  t6999 = t6982 + t6984 + t6986;
  t6939 = t201*t6936*t54;
  t6946 = t6945*t2780;
  t6959 = t6955*t2980;
  t6962 = t6939 + t6946 + t6959;
  t5622 = t26*t201*t5125;
  t5629 = t5141*t2552;
  t5642 = t5622 + t5624 + t5629;
  t5679 = t26*t201*t5164;
  t5683 = t5172*t2340;
  t5686 = -0.103955395616*t5137*t2552;
  t5696 = t5679 + t5683 + t5686;
  t5780 = t5188*t26*t201;
  t5788 = t5206*t2340;
  t5800 = t5221*t2552;
  t5804 = t5780 + t5788 + t5800;
  t5910 = t5172*t1977;
  t5912 = t5909 + t5910;
  t6000 = t5141*t2780;
  t6001 = -0.103955395616*t5137*t1977;
  t6002 = t6000 + t6001;
  t6066 = t5221*t2780;
  t6102 = t5206*t1977;
  t6127 = t6066 + t6102;
  t5852 = t5172*t3047;
  t5855 = t5624 + t5852;
  t5970 = t5172*t3162;
  t5971 = t5138 + t5970;
  t7201 = -1.*t5521*t214;
  t7203 = t5556*t1742;
  t7207 = t5614*t1784;
  t7211 = 0. + var1[2] + t3716 + t7201 + t3723 + t7203 + t7207;
  t5877 = t5141*t2340;
  t5880 = -0.103955395616*t5137*t3047;
  t5884 = t5877 + t5880;
  t7229 = -1.*t5614*t2340;
  t7230 = -1.*t5556*t3047;
  t7231 = t3980 + t3988 + t7229 + t7230;
  t7235 = t5614*t1742;
  t7236 = t5556*t3162;
  t7237 = t3937 + t3945 + t7235 + t7236;
  t6038 = t5141*t1742;
  t6039 = -0.103955395616*t5137*t3162;
  t6041 = t6038 + t6039;
  t7254 = -1.*t26*t201*t5521;
  t7256 = -1.*t5556*t2340;
  t7258 = -1.*t5614*t2552;
  t7262 = 0. + t3759 + t7254 + t3765 + t3786 + t7256 + t7258;
  t5888 = t5221*t2340;
  t5895 = t5206*t3047;
  t5896 = t5888 + t5895;
  t6136 = t5221*t1742;
  t6144 = t5206*t3162;
  t6145 = t6136 + t6144;
  t6265 = -0.103955395616*t5137*t2880;
  t6266 = t5172*t2736;
  t6272 = t6265 + t6266;
  t6400 = t5141*t2880;
  t6407 = -0.103955395616*t5137*t2736;
  t6408 = t6400 + t6407;
  t6448 = t5221*t2880;
  t6453 = t5206*t2736;
  t6460 = t6448 + t6453;
  t6213 = -0.103955395616*t5137*t2643;
  t6215 = t5172*t2651;
  t6218 = t6213 + t6215;
  t6356 = -0.103955395616*t5137*t2941;
  t6359 = t5172*t1784;
  t6364 = t6356 + t6359;
  t6232 = t5141*t2643;
  t6233 = -0.103955395616*t5137*t2651;
  t6245 = t6232 + t6233;
  t7325 = -1.*t5614*t2643;
  t7326 = -1.*t5556*t2651;
  t7327 = t3706 + t3709 + t7325 + t7326;
  t7329 = t5614*t2941;
  t7330 = t5556*t1784;
  t7331 = t3734 + t3735 + t7329 + t7330;
  t6420 = t5141*t2941;
  t6429 = t6420 + t5176;
  t6251 = t5221*t2643;
  t6258 = t5206*t2651;
  t6259 = t6251 + t6258;
  t6471 = t5221*t2941;
  t6476 = t5206*t1784;
  t6482 = t6471 + t6476;
  t6627 = -1.*t5125*t54*t214;
  t6633 = -0.103955395616*t5137*t3386;
  t6637 = t5141*t3544;
  t6639 = t6627 + t6633 + t6637;
  t6711 = -1.*t5164*t54*t214;
  t6712 = t5172*t3386;
  t6739 = -0.103955395616*t5137*t3544;
  t6742 = t6711 + t6712 + t6739;
  t6803 = -1.*t5188*t54*t214;
  t6804 = t5206*t3386;
  t6816 = t5221*t3544;
  t6817 = t6803 + t6804 + t6816;
  t6552 = -1.*t26*t5125*t214;
  t6554 = -0.103955395616*t5137*t3354;
  t6567 = t5141*t3380;
  t6574 = t6552 + t6554 + t6567;
  t6666 = -1.*t201*t5125;
  t6671 = -0.103955395616*t5137*t3497;
  t6678 = t5141*t3559;
  t6680 = t6666 + t6671 + t6678;
  t6588 = -1.*t26*t5164*t214;
  t6599 = t5172*t3354;
  t6600 = -0.103955395616*t5137*t3380;
  t6604 = t6588 + t6599 + t6600;
  t7437 = t26*t5521*t214;
  t7438 = -1.*t5556*t3354;
  t7441 = -1.*t5614*t3380;
  t7444 = t4090 + t7437 + t4091 + t7438 + t7441;
  t7451 = -1.*t201*t5521;
  t7465 = t5556*t3497;
  t7466 = t5614*t3559;
  t7468 = t7451 + t4098 + t4106 + t7465 + t7466;
  t6775 = -1.*t201*t5164;
  t6778 = t5172*t3497;
  t6783 = -0.103955395616*t5137*t3559;
  t6785 = t6775 + t6778 + t6783;
  t6607 = -1.*t5188*t26*t214;
  t6610 = t5206*t3354;
  t6618 = t5221*t3380;
  t6621 = t6607 + t6610 + t6618;
  t6823 = -1.*t5188*t201;
  t6824 = t5206*t3497;
  t6832 = t5221*t3559;
  t6833 = t6823 + t6824 + t6832;
  t6901 = -1.0000001112680001*t201*t5122*t54;
  t6902 = 0.104528*t5129*t2780;
  t6903 = 0.994522*t5129*t2980;
  t6911 = t6901 + t6902 + t6903;
  t7051 = -0.994522*t5129*t201*t54;
  t7054 = -0.103955395616*t5122*t2780;
  t7055 = -0.9890740084840001*t5122*t2980;
  t7057 = t7051 + t7054 + t7055;
  t7091 = -0.104528*t5129*t201*t54;
  t7097 = -0.010926102783999999*t5122*t2780;
  t7114 = -0.103955395616*t5122*t2980;
  t7118 = t7091 + t7097 + t7114;
  t6849 = -1.0000001112680001*t26*t201*t5122;
  t6850 = 0.104528*t5129*t2340;
  t6851 = 0.994522*t5129*t2552;
  t6856 = t6849 + t6850 + t6851;
  t6916 = 1.0000001112680001*t5122*t214;
  t6922 = 0.104528*t5129*t1742;
  t6923 = 0.994522*t5129*t1784;
  t6926 = t6916 + t6922 + t6923;
  t7542 = -1.*t6936*t214;
  t7544 = t6945*t1742;
  t7545 = t6955*t1784;
  t7546 = t7542 + t7544 + t7545;
  t7022 = 0.994522*t5129*t214;
  t7023 = -0.103955395616*t5122*t1742;
  t7035 = -0.9890740084840001*t5122*t1784;
  t7040 = t7022 + t7023 + t7035;
  t7522 = -1.*t26*t201*t6936;
  t7531 = -1.*t6945*t2340;
  t7532 = -1.*t6955*t2552;
  t7540 = t7522 + t7531 + t7532;
  t6858 = -0.994522*t5129*t26*t201;
  t6861 = -0.103955395616*t5122*t2340;
  t6862 = -0.9890740084840001*t5122*t2552;
  t6868 = t6858 + t6861 + t6862;
  t7077 = 0.104528*t5129*t214;
  t7078 = -0.010926102783999999*t5122*t1742;
  t7079 = -0.103955395616*t5122*t1784;
  t7081 = t7077 + t7078 + t7079;
  t6872 = -0.104528*t5129*t26*t201;
  t6874 = -0.010926102783999999*t5122*t2340;
  t6879 = -0.103955395616*t5122*t2552;
  t6882 = t6872 + t6874 + t6879;
  t5422 = -0.103955395616*t5137*t2029;
  t5429 = t5141*t2046;
  t5430 = t5244 + t5422 + t5429;
  t5453 = t5172*t2029;
  t5454 = -0.103955395616*t5137*t2046;
  t5456 = t5294 + t5453 + t5454;
  t7595 = -1.*t5556*t2029;
  t7596 = -1.*t5614*t2046;
  t7597 = t5973 + t4243 + t4259 + t7595 + t7596;
  t5458 = t5206*t2029;
  t5468 = t5221*t2046;
  t5490 = t5354 + t5458 + t5468;
  t7688 = t5521*t54*t214;
  t7690 = -1.*t5556*t3386;
  t7691 = -1.*t5614*t3544;
  t7694 = t4417 + t7688 + t4436 + t7690 + t7691;
  t7698 = 0. + var1[0] + t5649 + t2393 + t2419 + t5651 + t5656;
  t7701 = -1.*t26*t5521*t214;
  t7702 = t5556*t3354;
  t7703 = t5614*t3380;
  t7704 = t4458 + t7701 + t4460 + t7702 + t7703;
  t7717 = -1.*t201*t5521*t54;
  t7718 = -1.*t5556*t2780;
  t7725 = -1.*t5614*t2980;
  t7730 = 0. + t4484 + t7717 + t4490 + t4501 + t7718 + t7725;
  t7783 = -1.*t5614*t2880;
  t7784 = -1.*t5556*t2736;
  t7785 = t4579 + t4586 + t7783 + t7784;
  t7787 = t5614*t2643;
  t7789 = t5556*t2651;
  t7790 = t4604 + t4612 + t7787 + t7789;
  t7844 = -1.*t5614*t2780;
  t7845 = -1.*t5556*t1977;
  t7849 = t4696 + t4700 + t7844 + t7845;
  t7855 = t5614*t2340;
  t7860 = t5556*t3047;
  t7862 = t4673 + t4679 + t7855 + t7860;
  t7899 = t7254 + t3765 + t3786 + t7256 + t7258;
  t7902 = t5556*t2029;
  t7903 = t5614*t2046;
  t7909 = t7717 + t4787 + t4788 + t7902 + t7903;
  t7968 = t26*t201*t6936;
  t7983 = t6945*t2340;
  t7984 = t6955*t2552;
  t7986 = t7968 + t7983 + t7984;
  t7960 = -1.*t201*t6936*t54;
  t7961 = -1.*t6945*t2780;
  t7962 = -1.*t6955*t2980;
  t7963 = t7960 + t7961 + t7962;
  t7180 = -0.703234*t5642;
  t7181 = -0.073913*t5696;
  t7185 = 0.707107*t5804;
  t7187 = t7180 + t7181 + t7185;
  p_output1[0]=0;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=0;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=0;
  p_output1[13]=0;
  p_output1[14]=0;
  p_output1[15]=0;
  p_output1[16]=0;
  p_output1[17]=0;
  p_output1[18]=var2[1];
  p_output1[19]=-1.*var2[0];
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=-1.*t26*var2[2] + t54*var1[2]*var2[3];
  p_output1[25]=-1.*t54*var2[2] - 1.*t26*var1[2]*var2[3];
  p_output1[26]=t26*var2[0] + t54*var2[1] + (-1.*t54*var1[0] + t26*var1[1])*var2[3];
  p_output1[27]=-1.*t26*var2[3];
  p_output1[28]=-1.*t54*var2[3];
  p_output1[29]=0;
  p_output1[30]=-1.*t214*var2[1] - 1.*t201*t54*var2[2] - 1.*t201*t26*var1[2]*var2[3] + (-1.*t201*var1[1] + t214*t54*var1[2])*var2[4];
  p_output1[31]=t214*var2[0] + t201*t26*var2[2] - 1.*t201*t54*var1[2]*var2[3] + (t201*var1[0] - 1.*t214*t26*var1[2])*var2[4];
  p_output1[32]=t201*t54*var2[0] - 1.*t201*t26*var2[1] + (t201*t26*var1[0] + t201*t54*var1[1])*var2[3] + (-1.*t214*t54*var1[0] + t214*t26*var1[1])*var2[4];
  p_output1[33]=-1.*t201*t54*var2[3] - 1.*t214*t26*var2[4];
  p_output1[34]=t201*t26*var2[3] - 1.*t214*t54*var2[4];
  p_output1[35]=-1.*t201*var2[4];
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
  p_output1[54]=0;
  p_output1[55]=0;
  p_output1[56]=0;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
  p_output1[60]=0;
  p_output1[61]=0;
  p_output1[62]=0;
  p_output1[63]=0;
  p_output1[64]=0;
  p_output1[65]=0;
  p_output1[66]=0;
  p_output1[67]=0;
  p_output1[68]=0;
  p_output1[69]=0;
  p_output1[70]=0;
  p_output1[71]=0;
  p_output1[72]=0;
  p_output1[73]=0;
  p_output1[74]=0;
  p_output1[75]=0;
  p_output1[76]=0;
  p_output1[77]=0;
  p_output1[78]=0;
  p_output1[79]=0;
  p_output1[80]=0;
  p_output1[81]=0;
  p_output1[82]=0;
  p_output1[83]=0;
  p_output1[84]=0;
  p_output1[85]=0;
  p_output1[86]=0;
  p_output1[87]=0;
  p_output1[88]=0;
  p_output1[89]=0;
  p_output1[90]=0;
  p_output1[91]=0;
  p_output1[92]=0;
  p_output1[93]=0;
  p_output1[94]=0;
  p_output1[95]=0;
  p_output1[96]=0;
  p_output1[97]=0;
  p_output1[98]=0;
  p_output1[99]=0;
  p_output1[100]=0;
  p_output1[101]=0;
  p_output1[102]=0;
  p_output1[103]=0;
  p_output1[104]=0;
  p_output1[105]=0;
  p_output1[106]=0;
  p_output1[107]=0;
  p_output1[108]=0;
  p_output1[109]=0;
  p_output1[110]=0;
  p_output1[111]=0;
  p_output1[112]=0;
  p_output1[113]=0;
  p_output1[114]=0;
  p_output1[115]=0;
  p_output1[116]=0;
  p_output1[117]=0;
  p_output1[118]=0;
  p_output1[119]=0;
  p_output1[120]=0;
  p_output1[121]=0;
  p_output1[122]=0;
  p_output1[123]=0;
  p_output1[124]=0;
  p_output1[125]=0;
  p_output1[126]=0;
  p_output1[127]=0;
  p_output1[128]=0;
  p_output1[129]=0;
  p_output1[130]=0;
  p_output1[131]=0;
  p_output1[132]=0;
  p_output1[133]=0;
  p_output1[134]=0;
  p_output1[135]=0;
  p_output1[136]=0;
  p_output1[137]=0;
  p_output1[138]=0;
  p_output1[139]=0;
  p_output1[140]=0;
  p_output1[141]=0;
  p_output1[142]=0;
  p_output1[143]=0;
  p_output1[144]=t214*var2[1] + t201*t54*var2[2] + (-0.12*t1028 - 0.4*t1060 + t201*t26*var1[2])*var2[3] + (-0.12*t201*t26*t782 - 0.4*t201*t26*t830 + t201*var1[1] - 1.*t214*t54*var1[2])*var2[4] + (-0.4*t883 - 0.12*t951)*var2[5];
  p_output1[145]=-1.*t214*var2[0] - 1.*t201*t26*var2[2] + (-0.4*t1300 - 0.12*t883 + t201*t54*var1[2])*var2[3] + (-0.12*t201*t54*t782 - 0.4*t201*t54*t830 - 1.*t201*var1[0] + t214*t26*var1[2])*var2[4] + (-0.12*t1060 - 0.4*t1364)*var2[5];
  p_output1[146]=-1.*t201*t54*var2[0] + t201*t26*var2[1] + (-1.*t201*t26*var1[0] - 1.*t201*t54*var1[1])*var2[3] + (0.12*t214*t782 + 0.4*t214*t830 + t214*t54*var1[0] - 1.*t214*t26*var1[1])*var2[4] + (-0.4*t201*t782 + 0.12*t201*t830)*var2[5];
  p_output1[147]=t201*t54*var2[3] + t214*t26*var2[4];
  p_output1[148]=-1.*t201*t26*var2[3] + t214*t54*var2[4];
  p_output1[149]=t201*var2[4];
  p_output1[150]=(-0.994522*t1742 + 0.104528*t1784)*var2[1] + (-0.994522*(t1841 - 1.*t1717*t1916) + 0.104528*t1977)*var2[2] + (0.005906*t2029 + 0.056191*t2046 - 0.994522*(t2321*t2340 + t1742*t2420) + 0.104528*(t1784*t2420 + t2321*t2552) - 0.385267*t201*t54)*var2[3] + (-0.385267*t214*t26 + 0.005906*t3354 + 0.056191*t3380 - 0.994522*(t2321*t3386 + t1742*t3430 + t2780*t3467 + t2806*t3497) + 0.104528*(t1784*t3430 + t2980*t3467 + t2321*t3544 + t2806*t3559))*var2[4] + (0.056191*t2643 + 0.005906*t2651 - 0.994522*(t1742*t2665 + t2321*t2736 + t2759*t2780 + t1784*t2806) + 0.104528*(t1784*t2665 + t2321*t2880 + t2806*t2941 + t2759*t2980))*var2[5] + (0.056191*t2340 + 0.005906*t3047 - 0.994522*(t1977*t2321 + t2780*t3122 + t2806*t3162 + t1742*t3186) + 0.104528*(t2321*t2780 + t1742*t2806 + t2980*t3122 + t1784*t3186))*var2[24];
  p_output1[151]=(-0.994522*t2941 + 0.104528*t3162)*var2[0] + t3698*var2[2] + (0.005906*t2340 + 0.056191*t2552 + 0.385267*t201*t26 - 0.994522*(t2029*t3724 + t1742*t4262) + 0.104528*(t2046*t3724 + t1784*t4262))*var2[3] + (0.005906*t3386 + 0.056191*t3544 - 0.994522*(t3354*t3724 + t3497*t3802 + t1742*t4092 + t2340*t4109) + 0.104528*(t3380*t3724 + t3559*t3802 + t1784*t4092 + t2552*t4109) - 0.385267*t214*t54)*var2[4] + (0.005906*t2736 + 0.056191*t2880 - 0.994522*(t1742*t3710 + t2651*t3724 + t2340*t3736 + t1784*t3802) + 0.104528*(t1784*t3710 + t2643*t3724 + t2552*t3736 + t2941*t3802))*var2[5] + (0.005906*t1977 + 0.056191*t2780 - 0.994522*(t3047*t3724 + t3162*t3802 + t2340*t3948 + t1742*t4006) + 0.104528*(t2340*t3724 + t1742*t3802 + t2552*t3948 + t1784*t4006))*var2[24];
  p_output1[152]=(-0.994522*t2780 + 0.104528*t2980)*var2[0] + (-0.994522*(-1.*t1300*t1717 + t2633) + 0.104528*t3047)*var2[1] + (-0.994522*(t2340*t4452 + t2029*t4502 + t2340*t4770 + t2780*t4790) + 0.104528*(t2552*t4452 + t2046*t4502 + t2552*t4770 + t2980*t4790))*var2[3] + (-0.385267*t201 + 0.005906*t3497 + 0.056191*t3559 - 0.994522*(t2340*t4443 + t3386*t4452 + t2780*t4464 + t3354*t4502) + 0.104528*(t2552*t4443 + t3544*t4452 + t2980*t4464 + t3380*t4502))*var2[4] + (0.005906*t1784 + 0.056191*t2941 - 0.994522*(t2736*t4452 + t2651*t4502 + t2340*t4592 + t2780*t4616) + 0.104528*(t2880*t4452 + t2643*t4502 + t2552*t4592 + t2980*t4616))*var2[5] + (0.056191*t1742 + 0.005906*t3162 - 0.994522*(t1977*t4452 + t3047*t4502 + t2780*t4680 + t2340*t4701) + 0.104528*(t2780*t4452 + t2340*t4502 + t2980*t4680 + t2552*t4701))*var2[24];
  p_output1[153]=(-0.994522*t2029 + 0.104528*t2046)*var2[3] + (-0.994522*t3354 + 0.104528*t3380)*var2[4] + (0.104528*t2643 - 0.994522*t2651)*var2[5] + (0.104528*t2340 - 0.994522*t3047)*var2[24];
  p_output1[154]=t3698*var2[3] + (-0.994522*t3386 + 0.104528*t3544)*var2[4] + (-0.994522*t2736 + 0.104528*t2880)*var2[5] + (-0.994522*t1977 + 0.104528*t2780)*var2[24];
  p_output1[155]=(-0.994522*t3497 + 0.104528*t3559)*var2[4] + (-0.994522*t1784 + 0.104528*t2941)*var2[5] + (0.104528*t1742 - 0.994522*t3162)*var2[24];
  p_output1[156]=(-0.703234*t5145 - 0.073913*t5179 + 0.707107*t5228)*var2[1] + (-0.703234*(0.103955395616*t2780*t5137 - 1.*t2980*t5141 + t5244) - 0.073913*(0.103955395616*t2980*t5137 - 1.*t2780*t5172 + t5294) + 0.707107*(-1.*t2780*t5206 - 1.*t2980*t5221 + t5354))*var2[2] + (0.151261*t5430 + 0.249652*t5456 + 0.176528*t5490 - 0.703234*(t5621*t5642 + t5145*t5669) - 0.073913*(t5179*t5669 + t5621*t5696) + 0.707107*(t5228*t5669 + t5621*t5804))*var2[3] + (0.151261*t6574 + 0.249652*t6604 + 0.176528*t6621 - 0.703234*(t5621*t6639 + t5145*t6650 + t6032*t6660 + t5982*t6680) - 0.073913*(t5179*t6650 + t5963*t6660 + t5621*t6742 + t5982*t6785) + 0.707107*(t5228*t6650 + t6173*t6660 + t5621*t6817 + t5982*t6833))*var2[4] + (0.249652*t6218 + 0.151261*t6245 + 0.176528*t6259 - 0.073913*(t5621*t6272 + t5179*t6304 + t5963*t6350 + t5982*t6364) - 0.703234*(t5145*t6304 + t6032*t6350 + t5621*t6408 + t5982*t6429) + 0.707107*(t5228*t6304 + t6173*t6350 + t5621*t6460 + t5982*t6482))*var2[5] + (0.249652*t5855 + 0.151261*t5884 + 0.176528*t5896 - 0.073913*(t5621*t5912 + t5179*t5919 + t5945*t5963 + t5971*t5982) - 0.703234*(t5145*t5919 + t5621*t6002 + t5945*t6032 + t5982*t6041) + 0.707107*(t5228*t5919 + t5621*t6127 + t5982*t6145 + t5945*t6173))*var2[24] + (0.176528*t6856 + 0.151261*t6868 + 0.249652*t6882 + 0.707107*(t5621*t6911 + t5982*t6926 + t5228*t6962 + t6173*t6999) - 0.703234*(t5145*t6962 + t6032*t6999 + t5982*t7040 + t5621*t7057) - 0.073913*(t5179*t6962 + t5963*t6999 + t5982*t7081 + t5621*t7118))*var2[25];
  p_output1[157]=(-0.703234*(t214*t5125 + 0.103955395616*t1742*t5137 - 1.*t1784*t5141) - 0.073913*(0.103955395616*t1784*t5137 + t214*t5164 - 1.*t1742*t5172) + 0.707107*(t214*t5188 - 1.*t1742*t5206 - 1.*t1784*t5221))*var2[0] + t7187*var2[2] + (0.151261*t5642 + 0.249652*t5696 + 0.176528*t5804 - 0.703234*(t5430*t7211 + t5145*t7597) - 0.073913*(t5456*t7211 + t5179*t7597) + 0.707107*(t5490*t7211 + t5228*t7597))*var2[3] + (0.151261*t6639 + 0.249652*t6742 + 0.176528*t6817 - 0.703234*(t6574*t7211 + t6680*t7262 + t5145*t7444 + t5642*t7468) - 0.073913*(t6604*t7211 + t6785*t7262 + t5179*t7444 + t5696*t7468) + 0.707107*(t6621*t7211 + t6833*t7262 + t5228*t7444 + t5804*t7468))*var2[4] + (0.249652*t6272 + 0.151261*t6408 + 0.176528*t6460 - 0.703234*(t6245*t7211 + t6429*t7262 + t5145*t7327 + t5642*t7331) - 0.073913*(t6218*t7211 + t6364*t7262 + t5179*t7327 + t5696*t7331) + 0.707107*(t6259*t7211 + t6482*t7262 + t5228*t7327 + t5804*t7331))*var2[5] + (0.249652*t5912 + 0.151261*t6002 + 0.176528*t6127 - 0.073913*(t5855*t7211 + t5179*t7231 + t5696*t7237 + t5971*t7262) - 0.703234*(t5884*t7211 + t5145*t7231 + t5642*t7237 + t6041*t7262) + 0.707107*(t5896*t7211 + t5228*t7231 + t5804*t7237 + t6145*t7262))*var2[24] + (0.176528*t6911 + 0.151261*t7057 + 0.249652*t7118 - 0.703234*(t6868*t7211 + t7040*t7262 + t5145*t7540 + t5642*t7546) - 0.073913*(t6882*t7211 + t7081*t7262 + t5179*t7540 + t5696*t7546) + 0.707107*(t6856*t7211 + t6926*t7262 + t5228*t7540 + t5804*t7546))*var2[25];
  p_output1[158]=(-0.073913*t5963 - 0.703234*t6032 + 0.707107*t6173)*var2[0] + (-0.703234*(-1.*t201*t26*t5125 + 0.103955395616*t2340*t5137 - 1.*t2552*t5141) - 0.073913*(0.103955395616*t2552*t5137 - 1.*t201*t26*t5164 - 1.*t2340*t5172) + 0.707107*(-1.*t201*t26*t5188 - 1.*t2340*t5206 - 1.*t2552*t5221))*var2[1] + (-0.073913*(t5696*t7698 + t5456*t7730 + t5696*t7899 + t5963*t7909) - 0.703234*(t5642*t7698 + t5430*t7730 + t5642*t7899 + t6032*t7909) + 0.707107*(t5804*t7698 + t5490*t7730 + t5804*t7899 + t6173*t7909))*var2[3] + (0.151261*t6680 + 0.249652*t6785 + 0.176528*t6833 - 0.703234*(t5642*t7694 + t6639*t7698 + t6032*t7704 + t6574*t7730) - 0.073913*(t5696*t7694 + t6742*t7698 + t5963*t7704 + t6604*t7730) + 0.707107*(t5804*t7694 + t6817*t7698 + t6173*t7704 + t6621*t7730))*var2[4] + (0.249652*t6364 + 0.151261*t6429 + 0.176528*t6482 - 0.073913*(t6272*t7698 + t6218*t7730 + t5696*t7785 + t5963*t7790) - 0.703234*(t6408*t7698 + t6245*t7730 + t5642*t7785 + t6032*t7790) + 0.707107*(t6460*t7698 + t6259*t7730 + t5804*t7785 + t6173*t7790))*var2[5] + (0.249652*t5971 + 0.151261*t6041 + 0.176528*t6145 - 0.073913*(t5912*t7698 + t5855*t7730 + t5696*t7849 + t5963*t7862) - 0.703234*(t6002*t7698 + t5884*t7730 + t5642*t7849 + t6032*t7862) + 0.707107*(t6127*t7698 + t5896*t7730 + t5804*t7849 + t6173*t7862))*var2[24] + (0.176528*t6926 + 0.151261*t7040 + 0.249652*t7081 - 0.073913*(t7118*t7698 + t6882*t7730 + t5696*t7963 + t5963*t7986) - 0.703234*(t7057*t7698 + t6868*t7730 + t5642*t7963 + t6032*t7986) + 0.707107*(t6911*t7698 + t6856*t7730 + t5804*t7963 + t6173*t7986))*var2[25];
  p_output1[159]=(-0.703234*t5430 - 0.073913*t5456 + 0.707107*t5490)*var2[3] + (-0.703234*t6574 - 0.073913*t6604 + 0.707107*t6621)*var2[4] + (-0.073913*t6218 - 0.703234*t6245 + 0.707107*t6259)*var2[5] + (-0.073913*t5855 - 0.703234*t5884 + 0.707107*t5896)*var2[24] + (0.707107*t6856 - 0.703234*t6868 - 0.073913*t6882)*var2[25];
  p_output1[160]=t7187*var2[3] + (-0.703234*t6639 - 0.073913*t6742 + 0.707107*t6817)*var2[4] + (-0.073913*t6272 - 0.703234*t6408 + 0.707107*t6460)*var2[5] + (-0.073913*t5912 - 0.703234*t6002 + 0.707107*t6127)*var2[24] + (0.707107*t6911 - 0.703234*t7057 - 0.073913*t7118)*var2[25];
  p_output1[161]=(-0.703234*t6680 - 0.073913*t6785 + 0.707107*t6833)*var2[4] + (-0.073913*t6364 - 0.703234*t6429 + 0.707107*t6482)*var2[5] + (-0.073913*t5971 - 0.703234*t6041 + 0.707107*t6145)*var2[24] + (0.707107*t6926 - 0.703234*t7040 - 0.073913*t7081)*var2[25];
  p_output1[162]=0;
  p_output1[163]=0;
  p_output1[164]=0;
  p_output1[165]=0;
  p_output1[166]=0;
  p_output1[167]=0;
}



void dJs_shoulder_yaw_joint_right_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
