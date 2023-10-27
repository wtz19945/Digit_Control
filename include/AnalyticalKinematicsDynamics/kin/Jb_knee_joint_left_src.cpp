/*
 * Automatically Generated from Mathematica.
 * Mon 4 Jul 2022 20:33:37 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jb_knee_joint_left_src.h"

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
static void output1(double *p_output1,const double *var1)
{
  double t340;
  double t341;
  double t343;
  double t323;
  double t327;
  double t339;
  double t712;
  double t133;
  double t136;
  double t137;
  double t344;
  double t345;
  double t416;
  double t667;
  double t733;
  double t786;
  double t837;
  double t843;
  double t1017;
  double t1018;
  double t1027;
  double t1063;
  double t1068;
  double t1080;
  double t1096;
  double t1175;
  double t1289;
  double t1308;
  double t1315;
  double t1342;
  double t1383;
  double t1384;
  double t1412;
  double t1445;
  double t1452;
  double t1591;
  double t1616;
  double t919;
  double t999;
  double t1013;
  double t1287;
  double t1639;
  double t1660;
  double t1711;
  double t1733;
  double t1828;
  double t1830;
  double t1835;
  double t1846;
  double t1906;
  double t2000;
  double t2190;
  double t2235;
  double t2301;
  double t2378;
  double t2502;
  double t2515;
  double t2531;
  double t2536;
  double t2548;
  double t2565;
  double t2616;
  double t2713;
  double t2714;
  double t2813;
  double t1740;
  double t1751;
  double t1758;
  double t2291;
  double t2316;
  double t2355;
  double t2733;
  double t2734;
  double t131;
  double t2819;
  double t2839;
  double t2869;
  double t2940;
  double t2985;
  double t3027;
  double t3069;
  double t3076;
  double t3135;
  double t2796;
  double t3084;
  double t3088;
  double t84;
  double t3199;
  double t3280;
  double t3294;
  double t81;
  double t3957;
  double t4039;
  double t4043;
  double t4067;
  double t4123;
  double t4144;
  double t4042;
  double t4083;
  double t4147;
  double t4170;
  double t4226;
  double t4233;
  double t4361;
  double t4365;
  double t4435;
  double t4483;
  double t4510;
  double t4592;
  double t4178;
  double t4392;
  double t4645;
  double t4708;
  double t4718;
  double t4751;
  double t4790;
  double t4817;
  double t3327;
  double t3340;
  double t4709;
  double t4840;
  double t4891;
  double t5158;
  double t5232;
  double t5239;
  double t3484;
  double t3524;
  double t3610;
  double t3694;
  double t3736;
  double t3807;
  double t3810;
  double t6142;
  double t6181;
  double t6207;
  double t6061;
  double t6087;
  double t6138;
  double t6262;
  double t6269;
  double t6277;
  double t6140;
  double t6208;
  double t6300;
  double t6302;
  double t6329;
  double t6331;
  double t6337;
  double t6351;
  double t6368;
  double t6388;
  double t6394;
  double t6405;
  double t6322;
  double t6364;
  double t6464;
  double t6471;
  double t6490;
  double t6498;
  double t6514;
  double t6519;
  double t6479;
  double t6559;
  double t6565;
  double t6609;
  double t6615;
  double t6618;
  double t3124;
  double t3299;
  double t3303;
  double t3408;
  double t3410;
  double t3421;
  double t3437;
  double t3614;
  double t3737;
  double t3817;
  double t3824;
  double t3832;
  double t3835;
  double t5072;
  double t5256;
  double t5289;
  double t5355;
  double t5464;
  double t5515;
  double t5595;
  double t5625;
  double t5664;
  double t5704;
  double t5854;
  double t5875;
  double t5900;
  double t6588;
  double t6621;
  double t6624;
  double t6628;
  double t6630;
  double t6635;
  double t6637;
  double t6639;
  double t6640;
  double t6644;
  double t6647;
  double t6648;
  double t6658;
  double t6763;
  double t6770;
  double t6780;
  double t6782;
  double t6787;
  double t6788;
  double t6792;
  double t6795;
  double t6798;
  double t6802;
  double t6803;
  double t6810;
  double t6813;
  double t6816;
  double t6820;
  double t6823;
  double t6824;
  double t6826;
  double t6827;
  double t6749;
  double t6762;
  double t6791;
  double t6811;
  double t6812;
  double t6828;
  double t6832;
  double t6880;
  double t6905;
  double t6908;
  double t6921;
  double t6940;
  double t6941;
  double t6950;
  double t6953;
  double t6955;
  double t6959;
  double t6967;
  double t6969;
  double t6972;
  double t6982;
  double t6740;
  double t6743;
  double t6745;
  double t6837;
  double t6952;
  double t6986;
  double t7012;
  double t7015;
  double t7017;
  double t7019;
  double t7020;
  double t7021;
  double t7022;
  double t7028;
  double t6734;
  double t7014;
  double t7032;
  double t7036;
  double t7044;
  double t7046;
  double t7047;
  double t7048;
  double t7049;
  double t7060;
  double t7079;
  double t7083;
  double t7087;
  double t7092;
  double t7095;
  double t7098;
  double t7102;
  double t7107;
  double t7110;
  double t7111;
  double t6720;
  double t6723;
  double t6724;
  double t6725;
  double t6731;
  double t6732;
  double t7038;
  double t7061;
  double t7069;
  double t7088;
  double t7113;
  double t7114;
  double t7117;
  double t7118;
  double t7120;
  double t6706;
  double t6711;
  double t6713;
  double t7130;
  double t7073;
  double t7145;
  double t7138;
  double t7162;
  double t7156;
  double t7181;
  double t7173;
  double t7190;
  double t7186;
  double t7202;
  double t7197;
  double t7302;
  double t7303;
  double t7304;
  double t7307;
  double t7314;
  double t7315;
  double t7317;
  double t7318;
  double t7336;
  double t7337;
  double t7340;
  double t7344;
  double t7347;
  double t7349;
  double t7350;
  double t7353;
  double t7381;
  double t7382;
  double t7383;
  double t7384;
  double t7387;
  double t7389;
  double t7391;
  double t7392;
  double t7424;
  double t7425;
  double t7426;
  double t7427;
  double t7430;
  double t7434;
  double t7435;
  double t7436;
  double t7449;
  double t7450;
  double t7451;
  double t7452;
  double t7454;
  double t7455;
  double t7456;
  double t7457;
  double t7472;
  double t7473;
  double t7474;
  double t7475;
  double t7477;
  double t7478;
  double t7479;
  double t7480;
  t340 = Cos(var1[9]);
  t341 = -1.*t340;
  t343 = 1. + t341;
  t323 = Cos(var1[8]);
  t327 = -1.*t323;
  t339 = 1. + t327;
  t712 = Sin(var1[9]);
  t133 = Cos(var1[7]);
  t136 = -1.*t133;
  t137 = 1. + t136;
  t344 = -0.134322983001*t343;
  t345 = 1. + t344;
  t416 = 0.259155*t345;
  t667 = -0.22434503092393926*t343;
  t733 = -0.366501*t712;
  t786 = 0. + t733;
  t837 = -0.707107*t786;
  t843 = t416 + t667 + t837;
  t1017 = -0.8656776547239999*t343;
  t1018 = 1. + t1017;
  t1027 = -0.657905*t1018;
  t1063 = 0.0883716288660118*t343;
  t1068 = 0.930418*t712;
  t1080 = 0. + t1068;
  t1096 = -0.707107*t1080;
  t1175 = t1027 + t1063 + t1096;
  t1289 = -1.000000637725*t343;
  t1308 = 1. + t1289;
  t1315 = -0.707107*t1308;
  t1342 = -0.930418*t712;
  t1383 = 0. + t1342;
  t1384 = -0.657905*t1383;
  t1412 = 0.366501*t712;
  t1445 = 0. + t1412;
  t1452 = 0.259155*t1445;
  t1591 = t1315 + t1384 + t1452;
  t1616 = Sin(var1[8]);
  t919 = 0.340999127418*t339*t843;
  t999 = -0.8656776547239999*t339;
  t1013 = 1. + t999;
  t1287 = t1013*t1175;
  t1639 = -0.930418*t1616;
  t1660 = 0. + t1639;
  t1711 = t1591*t1660;
  t1733 = 0. + t919 + t1287 + t1711;
  t1828 = -0.134322983001*t339;
  t1830 = 1. + t1828;
  t1835 = t1830*t843;
  t1846 = 0.340999127418*t339*t1175;
  t1906 = 0.366501*t1616;
  t2000 = 0. + t1906;
  t2190 = t1591*t2000;
  t2235 = 0. + t1835 + t1846 + t2190;
  t2301 = Sin(var1[7]);
  t2378 = -1.000000637725*t339;
  t2502 = 1. + t2378;
  t2515 = t2502*t1591;
  t2531 = -0.366501*t1616;
  t2536 = 0. + t2531;
  t2548 = t843*t2536;
  t2565 = 0.930418*t1616;
  t2616 = 0. + t2565;
  t2713 = t1175*t2616;
  t2714 = 0. + t2515 + t2548 + t2713;
  t2813 = Cos(var1[6]);
  t1740 = -0.340999127418*t137*t1733;
  t1751 = -0.8656776547239999*t137;
  t1758 = 1. + t1751;
  t2291 = t1758*t2235;
  t2316 = -0.930418*t2301;
  t2355 = 0. + t2316;
  t2733 = t2355*t2714;
  t2734 = 0. + t1740 + t2291 + t2733;
  t131 = Sin(var1[6]);
  t2819 = -0.134322983001*t137;
  t2839 = 1. + t2819;
  t2869 = t2839*t1733;
  t2940 = -0.340999127418*t137*t2235;
  t2985 = -0.366501*t2301;
  t3027 = 0. + t2985;
  t3069 = t3027*t2714;
  t3076 = 0. + t2869 + t2940 + t3069;
  t3135 = Cos(var1[5]);
  t2796 = -1.*t131*t2734;
  t3084 = t2813*t3076;
  t3088 = 0. + t2796 + t3084;
  t84 = Sin(var1[5]);
  t3199 = t2813*t2734;
  t3280 = t131*t3076;
  t3294 = 0. + t3199 + t3280;
  t81 = Sin(var1[3]);
  t3957 = 0.707107*t786;
  t4039 = t416 + t667 + t3957;
  t4043 = 0.707107*t1080;
  t4067 = t1027 + t1063 + t4043;
  t4123 = 0.707107*t1308;
  t4144 = t4123 + t1384 + t1452;
  t4042 = 0.340999127418*t339*t4039;
  t4083 = t1013*t4067;
  t4147 = t4144*t1660;
  t4170 = 0. + t4042 + t4083 + t4147;
  t4226 = t1830*t4039;
  t4233 = 0.340999127418*t339*t4067;
  t4361 = t4144*t2000;
  t4365 = 0. + t4226 + t4233 + t4361;
  t4435 = t2502*t4144;
  t4483 = t4039*t2536;
  t4510 = t4067*t2616;
  t4592 = 0. + t4435 + t4483 + t4510;
  t4178 = -0.340999127418*t137*t4170;
  t4392 = t1758*t4365;
  t4645 = t2355*t4592;
  t4708 = 0. + t4178 + t4392 + t4645;
  t4718 = t2839*t4170;
  t4751 = -0.340999127418*t137*t4365;
  t4790 = t3027*t4592;
  t4817 = 0. + t4718 + t4751 + t4790;
  t3327 = Cos(var1[3]);
  t3340 = Sin(var1[4]);
  t4709 = -1.*t131*t4708;
  t4840 = t2813*t4817;
  t4891 = 0. + t4709 + t4840;
  t5158 = t2813*t4708;
  t5232 = t131*t4817;
  t5239 = 0. + t5158 + t5232;
  t3484 = Cos(var1[4]);
  t3524 = 0.366501*t2301;
  t3610 = 0. + t3524;
  t3694 = 0.930418*t2301;
  t3736 = 0. + t3694;
  t3807 = -1.000000637725*t137;
  t3810 = 1. + t3807;
  t6142 = -0.930418*t345;
  t6181 = -0.12497652119782442*t343;
  t6207 = t6142 + t6181;
  t6061 = -0.366501*t1018;
  t6087 = -0.3172717261340007*t343;
  t6138 = t6061 + t6087;
  t6262 = -0.366501*t1383;
  t6269 = -0.930418*t1445;
  t6277 = t6262 + t6269;
  t6140 = t6138*t1013;
  t6208 = 0.340999127418*t6207*t339;
  t6300 = t6277*t1660;
  t6302 = 0. + t6140 + t6208 + t6300;
  t6329 = t6207*t1830;
  t6331 = 0.340999127418*t6138*t339;
  t6337 = t6277*t2000;
  t6351 = 0. + t6329 + t6331 + t6337;
  t6368 = t2502*t6277;
  t6388 = t6207*t2536;
  t6394 = t6138*t2616;
  t6405 = 0. + t6368 + t6388 + t6394;
  t6322 = -0.340999127418*t137*t6302;
  t6364 = t1758*t6351;
  t6464 = t2355*t6405;
  t6471 = 0. + t6322 + t6364 + t6464;
  t6490 = t2839*t6302;
  t6498 = -0.340999127418*t137*t6351;
  t6514 = t3027*t6405;
  t6519 = 0. + t6490 + t6498 + t6514;
  t6479 = -1.*t131*t6471;
  t6559 = t2813*t6519;
  t6565 = 0. + t6479 + t6559;
  t6609 = t2813*t6471;
  t6615 = t131*t6519;
  t6618 = 0. + t6609 + t6615;
  t3124 = -1.*t84*t3088;
  t3299 = t3135*t3294;
  t3303 = 0. + t3124 + t3299;
  t3408 = t3135*t3088;
  t3410 = t84*t3294;
  t3421 = 0. + t3408 + t3410;
  t3437 = t3340*t3421;
  t3614 = t3610*t1733;
  t3737 = t3736*t2235;
  t3817 = t3810*t2714;
  t3824 = 0. + t3614 + t3737 + t3817;
  t3832 = t3484*t3824;
  t3835 = 0. + t3437 + t3832;
  t5072 = -1.*t84*t4891;
  t5256 = t3135*t5239;
  t5289 = 0. + t5072 + t5256;
  t5355 = t3135*t4891;
  t5464 = t84*t5239;
  t5515 = 0. + t5355 + t5464;
  t5595 = t3340*t5515;
  t5625 = t3610*t4170;
  t5664 = t3736*t4365;
  t5704 = t3810*t4592;
  t5854 = 0. + t5625 + t5664 + t5704;
  t5875 = t3484*t5854;
  t5900 = 0. + t5595 + t5875;
  t6588 = -1.*t84*t6565;
  t6621 = t3135*t6618;
  t6624 = 0. + t6588 + t6621;
  t6628 = t3135*t6565;
  t6630 = t84*t6618;
  t6635 = 0. + t6628 + t6630;
  t6637 = t3340*t6635;
  t6639 = t3610*t6302;
  t6640 = t3736*t6351;
  t6644 = t3810*t6405;
  t6647 = 0. + t6639 + t6640 + t6644;
  t6648 = t3484*t6647;
  t6658 = 0. + t6637 + t6648;
  t6763 = -1.5981976069815686e-7*var1[9];
  t6770 = 0.165064*t345;
  t6780 = 0.022172213784128716*t343;
  t6782 = 0.039853*t786;
  t6787 = 0.039853013046*t1445;
  t6788 = t6763 + t6770 + t6780 + t6782 + t6787;
  t6792 = -6.295460977284962e-8*var1[9];
  t6795 = -0.189386*t1018;
  t6798 = -0.16394805317098082*t343;
  t6802 = 0.039853013046*t1383;
  t6803 = 0.039853*t1080;
  t6810 = t6792 + t6795 + t6798 + t6802 + t6803;
  t6813 = 0.039853*t1308;
  t6816 = 0.039853038461262744*t343;
  t6820 = -0.189386*t1383;
  t6823 = 0.086752619205*t786;
  t6824 = 0.165064*t1445;
  t6826 = -0.22023459268999998*t1080;
  t6827 = 0. + t6813 + t6816 + t6820 + t6823 + t6824 + t6826;
  t6749 = 3.2909349868922137e-7*var1[8];
  t6762 = 0.055653945343889656*t339;
  t6791 = t1830*t6788;
  t6811 = 0.340999127418*t339*t6810;
  t6812 = -0.045000372235*t2536;
  t6828 = t6827*t2000;
  t6832 = t6749 + t6762 + t6791 + t6811 + t6812 + t6828;
  t6880 = 1.296332362046933e-7*var1[8];
  t6905 = -0.14128592423750855*t339;
  t6908 = 0.340999127418*t339*t6788;
  t6921 = t1013*t6810;
  t6940 = t6827*t1660;
  t6941 = -0.045000372235*t2616;
  t6950 = t6880 + t6905 + t6908 + t6921 + t6940 + t6941;
  t6953 = -0.04500040093286238*t339;
  t6955 = t2502*t6827;
  t6959 = -0.141285834136*t1660;
  t6967 = t6788*t2536;
  t6969 = 0.055653909852*t2000;
  t6972 = t6810*t2616;
  t6982 = 0. + t6953 + t6955 + t6959 + t6967 + t6969 + t6972;
  t6740 = 1.296332362046933e-7*var1[7];
  t6743 = 0.07877668146182712*t137;
  t6745 = -0.045000372235*t3736;
  t6837 = t1758*t6832;
  t6952 = -0.340999127418*t137*t6950;
  t6986 = t2355*t6982;
  t7012 = t6740 + t6743 + t6745 + t6837 + t6952 + t6986;
  t7015 = -3.2909349868922137e-7*var1[7];
  t7017 = 0.03103092645718495*t137;
  t7019 = -0.045000372235*t3610;
  t7020 = -0.340999127418*t137*t6832;
  t7021 = t2839*t6950;
  t7022 = t3027*t6982;
  t7028 = t7015 + t7017 + t7019 + t7020 + t7021 + t7022;
  t6734 = 0.091*t131;
  t7014 = -1.*t131*t7012;
  t7032 = t2813*t7028;
  t7036 = 0. + t6734 + t7014 + t7032;
  t7044 = -1.*t2813;
  t7046 = 1. + t7044;
  t7047 = 0.091*t7046;
  t7048 = t2813*t7012;
  t7049 = t131*t7028;
  t7060 = 0. + t7047 + t7048 + t7049;
  t7079 = t3135*t7036;
  t7083 = t84*t7060;
  t7087 = 0. + t7079 + t7083;
  t7092 = -0.04500040093286238*t137;
  t7095 = 0.07877663122399998*t2355;
  t7098 = 0.031030906668*t3027;
  t7102 = t3736*t6832;
  t7107 = t3610*t6950;
  t7110 = t3810*t6982;
  t7111 = 0. + t7092 + t7095 + t7098 + t7102 + t7107 + t7110;
  t6720 = t3484*t5515;
  t6723 = -1.*t3340*t5854;
  t6724 = 0. + t6720 + t6723;
  t6725 = t3484*t6635;
  t6731 = -1.*t3340*t6647;
  t6732 = 0. + t6725 + t6731;
  t7038 = -1.*t84*t7036;
  t7061 = t3135*t7060;
  t7069 = 0. + t7038 + t7061;
  t7088 = t3340*t7087;
  t7113 = t3484*t7111;
  t7114 = 0. + t7088 + t7113;
  t7117 = t3484*t7087;
  t7118 = -1.*t3340*t7111;
  t7120 = 0. + t7117 + t7118;
  t6706 = t3484*t3421;
  t6711 = -1.*t3340*t3824;
  t6713 = 0. + t6706 + t6711;
  t7130 = t6624*t7069;
  t7073 = -1.*t7069*t5289;
  t7145 = -1.*t6624*t7069;
  t7138 = t3303*t7069;
  t7162 = -1.*t3303*t7069;
  t7156 = t7069*t5289;
  t7181 = -1.*t7111*t5854;
  t7173 = t6647*t7111;
  t7190 = t3824*t7111;
  t7186 = -1.*t6647*t7111;
  t7202 = t7111*t5854;
  t7197 = -1.*t3824*t7111;
  t7302 = -1.*t4365*t6832;
  t7303 = -1.*t4170*t6950;
  t7304 = -1.*t6982*t4592;
  t7307 = t7302 + t7303 + t7304;
  t7314 = t6351*t6832;
  t7315 = t6302*t6950;
  t7317 = t6405*t6982;
  t7318 = t7314 + t7315 + t7317;
  t7336 = t2235*t6832;
  t7337 = t1733*t6950;
  t7340 = t2714*t6982;
  t7344 = t7336 + t7337 + t7340;
  t7347 = -1.*t6351*t6832;
  t7349 = -1.*t6302*t6950;
  t7350 = -1.*t6405*t6982;
  t7353 = t7347 + t7349 + t7350;
  t7381 = t4365*t6832;
  t7382 = t4170*t6950;
  t7383 = t6982*t4592;
  t7384 = t7381 + t7382 + t7383;
  t7387 = -1.*t2235*t6832;
  t7389 = -1.*t1733*t6950;
  t7391 = -1.*t2714*t6982;
  t7392 = t7387 + t7389 + t7391;
  t7424 = -1.*t4039*t6788;
  t7425 = -1.*t4144*t6827;
  t7426 = -1.*t6810*t4067;
  t7427 = t7424 + t7425 + t7426;
  t7430 = t6207*t6788;
  t7434 = t6277*t6827;
  t7435 = t6138*t6810;
  t7436 = t7430 + t7434 + t7435;
  t7449 = t843*t6788;
  t7450 = t1591*t6827;
  t7451 = t1175*t6810;
  t7452 = t7449 + t7450 + t7451;
  t7454 = -1.*t6207*t6788;
  t7455 = -1.*t6277*t6827;
  t7456 = -1.*t6138*t6810;
  t7457 = t7454 + t7455 + t7456;
  t7472 = t4039*t6788;
  t7473 = t4144*t6827;
  t7474 = t6810*t4067;
  t7475 = t7472 + t7473 + t7474;
  t7477 = -1.*t843*t6788;
  t7478 = -1.*t1591*t6827;
  t7479 = -1.*t1175*t6810;
  t7480 = t7477 + t7478 + t7479;
  p_output1[0]=0. + t3327*t3835 - 1.*t3303*t81;
  p_output1[1]=0. + t3327*t5900 - 1.*t5289*t81;
  p_output1[2]=0. + t3327*t6658 - 1.*t6624*t81;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0. + t3303*t3327 + t3835*t81;
  p_output1[7]=0. + t3327*t5289 + t5900*t81;
  p_output1[8]=0. + t3327*t6624 + t6658*t81;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=t6713;
  p_output1[13]=t6724;
  p_output1[14]=t6732;
  p_output1[15]=0;
  p_output1[16]=0;
  p_output1[17]=0;
  p_output1[18]=t6732*(t7073 - 1.*t5900*t7114 - 1.*t6724*t7120) + t6724*(t6658*t7114 + t6732*t7120 + t7130);
  p_output1[19]=t6732*(t3835*t7114 + t6713*t7120 + t7138) + t6713*(-1.*t6658*t7114 - 1.*t6732*t7120 + t7145);
  p_output1[20]=t6713*(t5900*t7114 + t6724*t7120 + t7156) + t6724*(-1.*t3835*t7114 - 1.*t6713*t7120 + t7162);
  p_output1[21]=t6713;
  p_output1[22]=t6724;
  p_output1[23]=t6732;
  p_output1[24]=t5289*(t6635*t7087 + t7130 + t7173) + t6624*(t7073 - 1.*t5515*t7087 + t7181);
  p_output1[25]=t3303*(-1.*t6635*t7087 + t7145 + t7186) + t6624*(t3421*t7087 + t7138 + t7190);
  p_output1[26]=t5289*(-1.*t3421*t7087 + t7162 + t7197) + t3303*(t5515*t7087 + t7156 + t7202);
  p_output1[27]=t3303;
  p_output1[28]=t5289;
  p_output1[29]=t6624;
  p_output1[30]=t5854*(t6565*t7036 + t6618*t7060 + t7173) + t6647*(-1.*t4891*t7036 - 1.*t5239*t7060 + t7181);
  p_output1[31]=t3824*(-1.*t6565*t7036 - 1.*t6618*t7060 + t7186) + t6647*(t3088*t7036 + t3294*t7060 + t7190);
  p_output1[32]=t5854*(-1.*t3088*t7036 - 1.*t3294*t7060 + t7197) + t3824*(t4891*t7036 + t5239*t7060 + t7202);
  p_output1[33]=t3824;
  p_output1[34]=t5854;
  p_output1[35]=t6647;
  p_output1[36]=0.091*t3076 - 1.*t5854*(t6471*t7012 + t6519*t7028 + t7173) - 1.*t6647*(-1.*t4708*t7012 - 1.*t4817*t7028 + t7181);
  p_output1[37]=0.091*t4817 - 1.*t3824*(-1.*t6471*t7012 - 1.*t6519*t7028 + t7186) - 1.*t6647*(t2734*t7012 + t3076*t7028 + t7190);
  p_output1[38]=0.091*t6519 - 1.*t5854*(-1.*t2734*t7012 - 1.*t3076*t7028 + t7197) - 1.*t3824*(t4708*t7012 + t4817*t7028 + t7202);
  p_output1[39]=0. - 1.*t1733*t3610 - 1.*t2235*t3736 - 1.*t2714*t3810;
  p_output1[40]=0. - 1.*t3610*t4170 - 1.*t3736*t4365 - 1.*t3810*t4592;
  p_output1[41]=0. - 1.*t3610*t6302 - 1.*t3736*t6351 - 1.*t3810*t6405;
  p_output1[42]=-0.016493*t1733 - 0.041869*t2235 - 0.084668*t2714 - 0.930418*(t6302*t7307 + t4170*t7318) + 0.366501*(t6351*t7307 + t4365*t7318);
  p_output1[43]=-0.016493*t4170 - 0.041869*t4365 - 0.084668*t4592 - 0.930418*(t6302*t7344 + t1733*t7353) + 0.366501*(t6351*t7344 + t2235*t7353);
  p_output1[44]=-0.016493*t6302 - 0.041869*t6351 - 0.084668*t6405 - 0.930418*(t1733*t7384 + t4170*t7392) + 0.366501*(t2235*t7384 + t4365*t7392);
  p_output1[45]=0. - 0.930418*t1733 + 0.366501*t2235;
  p_output1[46]=0. - 0.930418*t4170 + 0.366501*t4365;
  p_output1[47]=0. - 0.930418*t6302 + 0.366501*t6351;
  p_output1[48]=-0.041869*t1175 + 0.151852*t1591 + 0.930418*(t6207*t7427 + t4039*t7436) + 0.366501*(t6138*t7427 + t4067*t7436) + 0.016493*t843;
  p_output1[49]=0.016493*t4039 - 0.041869*t4067 + 0.151852*t4144 + 0.366501*(t6138*t7452 + t1175*t7457) + 0.930418*(t6207*t7452 + t7457*t843);
  p_output1[50]=-0.041869*t6138 + 0.016493*t6207 + 0.151852*t6277 + 0.366501*(t1175*t7475 + t4067*t7480) + 0.930418*(t4039*t7480 + t7475*t843);
  p_output1[51]=0. + 0.366501*t1175 + 0.930418*t843;
  p_output1[52]=0. + 0.930418*t4039 + 0.366501*t4067;
  p_output1[53]=0. + 0.366501*t6138 + 0.930418*t6207;
  p_output1[54]=4.499284959325056e-7;
  p_output1[55]=-4.857531045524e-7;
  p_output1[56]=1.9226558521656667e-7;
  p_output1[57]=3.6361499999859603e-7;
  p_output1[58]=3.6361499999859603e-7;
  p_output1[59]=1.000000637725;
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
  p_output1[144]=0;
  p_output1[145]=0;
  p_output1[146]=0;
  p_output1[147]=0;
  p_output1[148]=0;
  p_output1[149]=0;
  p_output1[150]=0;
  p_output1[151]=0;
  p_output1[152]=0;
  p_output1[153]=0;
  p_output1[154]=0;
  p_output1[155]=0;
  p_output1[156]=0;
  p_output1[157]=0;
  p_output1[158]=0;
  p_output1[159]=0;
  p_output1[160]=0;
  p_output1[161]=0;
  p_output1[162]=0;
  p_output1[163]=0;
  p_output1[164]=0;
  p_output1[165]=0;
  p_output1[166]=0;
  p_output1[167]=0;
}



void Jb_knee_joint_left_src(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
