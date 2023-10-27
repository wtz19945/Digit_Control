/*
 * Automatically Generated from Mathematica.
 * Mon 4 Jul 2022 21:09:10 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "AMWorld_left_elbow_src.h"

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
  double t1091;
  double t1100;
  double t1124;
  double t498;
  double t519;
  double t797;
  double t1528;
  double t939;
  double t1131;
  double t1146;
  double t1334;
  double t1342;
  double t1603;
  double t1605;
  double t1606;
  double t1608;
  double t895;
  double t1651;
  double t1654;
  double t1682;
  double t1694;
  double t1757;
  double t1770;
  double t1857;
  double t1860;
  double t2002;
  double t2065;
  double t2103;
  double t2271;
  double t2277;
  double t2283;
  double t2295;
  double t2329;
  double t2386;
  double t2393;
  double t2655;
  double t1894;
  double t1071;
  double t1072;
  double t1624;
  double t1641;
  double t1648;
  double t1867;
  double t1915;
  double t1990;
  double t2397;
  double t2428;
  double t3811;
  double t2497;
  double t2544;
  double t2563;
  double t2569;
  double t2570;
  double t2572;
  double t2712;
  double t2739;
  double t2915;
  double t2944;
  double t3088;
  double t3227;
  double t3239;
  double t3317;
  double t3368;
  double t3376;
  double t3387;
  double t3394;
  double t3457;
  double t3493;
  double t3614;
  double t3668;
  double t3788;
  double t3807;
  double t3809;
  double t3830;
  double t3848;
  double t3889;
  double t3910;
  double t4372;
  double t4166;
  double t4181;
  double t4206;
  double t4233;
  double t4243;
  double t4282;
  double t4287;
  double t4309;
  double t4162;
  double t4376;
  double t4378;
  double t4401;
  double t4455;
  double t4468;
  double t4484;
  double t4490;
  double t4520;
  double t4027;
  double t4068;
  double t4072;
  double t4073;
  double t4153;
  double t4661;
  double t4698;
  double t4719;
  double t4310;
  double t4535;
  double t4541;
  double t4722;
  double t4910;
  double t4913;
  double t4915;
  double t4886;
  double t4887;
  double t4892;
  double t4925;
  double t4929;
  double t4930;
  double t4907;
  double t4923;
  double t4937;
  double t4959;
  double t4962;
  double t4980;
  double t5014;
  double t5098;
  double t5114;
  double t5120;
  double t5123;
  double t5126;
  double t5258;
  double t5267;
  double t5272;
  double t5274;
  double t5293;
  double t5303;
  double t5309;
  double t5311;
  double t5138;
  double t5141;
  double t5142;
  double t5143;
  double t4742;
  double t4759;
  double t5326;
  double t5327;
  double t5343;
  double t5287;
  double t5313;
  double t5316;
  double t5421;
  double t5424;
  double t5446;
  double t5449;
  double t5453;
  double t5469;
  double t5473;
  double t5476;
  double t5491;
  double t5506;
  double t5509;
  double t5520;
  double t5452;
  double t5490;
  double t5522;
  double t5534;
  double t5536;
  double t5538;
  double t5551;
  double t5558;
  double t5574;
  double t5585;
  double t5588;
  double t5589;
  double t5722;
  double t5723;
  double t5733;
  double t5736;
  double t5744;
  double t5746;
  double t5750;
  double t5757;
  double t5635;
  double t5638;
  double t5639;
  double t5645;
  double t5773;
  double t5776;
  double t5780;
  double t5740;
  double t5768;
  double t5771;
  double t5856;
  double t5873;
  double t5861;
  double t5874;
  double t5875;
  double t5909;
  double t5910;
  double t5912;
  double t5903;
  double t5913;
  double t5919;
  double t5930;
  double t5938;
  double t5939;
  double t5857;
  double t5921;
  double t5940;
  double t5952;
  double t5959;
  double t5965;
  double t5966;
  double t5992;
  double t6013;
  double t6026;
  double t6027;
  double t6029;
  double t5956;
  double t5994;
  double t6030;
  double t6033;
  double t6037;
  double t6043;
  double t6044;
  double t6046;
  double t6054;
  double t6069;
  double t6076;
  double t6081;
  double t389;
  double t2495;
  double t2992;
  double t3003;
  double t3060;
  double t3810;
  double t3879;
  double t3928;
  double t3994;
  double t4000;
  double t4076;
  double t4091;
  double t4109;
  double t4118;
  double t4127;
  double t4131;
  double t4544;
  double t4725;
  double t4734;
  double t4740;
  double t4756;
  double t4777;
  double t4780;
  double t4787;
  double t4789;
  double t4795;
  double t4796;
  double t4810;
  double t4868;
  double t4961;
  double t5101;
  double t5106;
  double t5111;
  double t5127;
  double t5128;
  double t5129;
  double t5133;
  double t5135;
  double t5186;
  double t5191;
  double t5209;
  double t5211;
  double t5248;
  double t5249;
  double t5325;
  double t5344;
  double t5352;
  double t5354;
  double t5363;
  double t5364;
  double t5370;
  double t5372;
  double t5374;
  double t5375;
  double t5376;
  double t5380;
  double t5399;
  double t5535;
  double t5560;
  double t5562;
  double t5568;
  double t5590;
  double t5597;
  double t5615;
  double t5621;
  double t5625;
  double t5662;
  double t5664;
  double t5685;
  double t5694;
  double t5709;
  double t5711;
  double t5772;
  double t5797;
  double t5807;
  double t5810;
  double t5822;
  double t5826;
  double t5831;
  double t5833;
  double t5839;
  double t5841;
  double t5846;
  double t5848;
  double t6036;
  double t6049;
  double t6082;
  double t6084;
  double t6088;
  double t6092;
  double t6096;
  double t6102;
  double t6136;
  double t6143;
  double t6155;
  double t6157;
  double t4823;
  double t5397;
  double t5849;
  double t5853;
  double t6233;
  double t6239;
  double t6240;
  double t6242;
  double t6247;
  double t6251;
  double t6241;
  double t6252;
  double t6256;
  double t6261;
  double t6266;
  double t6277;
  double t6229;
  double t6259;
  double t6283;
  double t6290;
  double t6296;
  double t6308;
  double t6314;
  double t6316;
  double t6320;
  double t6321;
  double t6324;
  double t6325;
  double t6294;
  double t6317;
  double t6326;
  double t6327;
  double t6334;
  double t6340;
  double t6345;
  double t6347;
  double t6351;
  double t6363;
  double t6367;
  double t6371;
  double t6126;
  double t6127;
  double t6130;
  double t6131;
  double t6331;
  double t6348;
  double t6372;
  double t6376;
  double t6387;
  double t6394;
  double t6395;
  double t6396;
  double t6177;
  double t6186;
  double t6198;
  double t6204;
  double t6423;
  double t6426;
  double t6429;
  double t6433;
  double t6510;
  double t6514;
  double t6515;
  double t6527;
  double t6533;
  double t6536;
  double t6497;
  double t6517;
  double t6538;
  double t6540;
  double t6543;
  double t6544;
  double t6545;
  double t6547;
  double t6554;
  double t6556;
  double t6558;
  double t6561;
  double t6542;
  double t6553;
  double t6562;
  double t6564;
  double t6568;
  double t6569;
  double t6570;
  double t6571;
  double t6585;
  double t6586;
  double t6589;
  double t6594;
  double t6567;
  double t6583;
  double t6596;
  double t6597;
  double t6604;
  double t6608;
  double t6609;
  double t6611;
  double t6633;
  double t6634;
  double t6635;
  double t6637;
  t1091 = Cos(var1[16]);
  t1100 = -1.*t1091;
  t1124 = 1. + t1100;
  t498 = Cos(var1[15]);
  t519 = -1.*t498;
  t797 = 1. + t519;
  t1528 = Sin(var1[16]);
  t939 = Sin(var1[15]);
  t1131 = -0.9890740084840001*t1124;
  t1146 = 1. + t1131;
  t1334 = -0.918819*t1146;
  t1342 = 0.010039180465428352*t1124;
  t1603 = -0.994522*t1528;
  t1605 = 0. + t1603;
  t1606 = 0.382684*t1605;
  t1608 = t1334 + t1342 + t1606;
  t895 = -0.051978134642000004*t797;
  t1651 = -0.010926102783999999*t1124;
  t1654 = 1. + t1651;
  t1682 = 0.096572*t1654;
  t1694 = -0.0955161926444975*t1124;
  t1757 = 0.104528*t1528;
  t1770 = 0. + t1757;
  t1857 = 0.382684*t1770;
  t1860 = t1682 + t1694 + t1857;
  t2002 = -1.0000001112680001*t1124;
  t2065 = 1. + t2002;
  t2103 = 0.382684*t2065;
  t2271 = -0.104528*t1528;
  t2277 = 0. + t2271;
  t2283 = 0.096572*t2277;
  t2295 = 0.994522*t1528;
  t2329 = 0. + t2295;
  t2386 = -0.918819*t2329;
  t2393 = t2103 + t2283 + t2386;
  t2655 = -0.49726168403800003*t797;
  t1894 = 0.05226439969100001*t797;
  t1071 = -0.707107*t939;
  t1072 = t895 + t1071;
  t1624 = t1072*t1608;
  t1641 = -0.9945383682050002*t797;
  t1648 = 1. + t1641;
  t1867 = t1648*t1860;
  t1915 = -0.703234*t939;
  t1990 = t1894 + t1915;
  t2397 = t1990*t2393;
  t2428 = 0. + t1624 + t1867 + t2397;
  t3811 = Sin(var1[14]);
  t2497 = -0.5054634410180001*t797;
  t2544 = 1. + t2497;
  t2563 = t2544*t1608;
  t2569 = 0.707107*t939;
  t2570 = t895 + t2569;
  t2572 = t2570*t1860;
  t2712 = -0.073913*t939;
  t2739 = t2655 + t2712;
  t2915 = t2739*t2393;
  t2944 = 0. + t2563 + t2572 + t2915;
  t3088 = Cos(var1[14]);
  t3227 = -1.*t3088;
  t3239 = 1. + t3227;
  t3317 = -1.0000001112680001*t3239;
  t3368 = 1. + t3317;
  t3376 = 0.073913*t939;
  t3387 = t2655 + t3376;
  t3394 = t3387*t1608;
  t3457 = 0.703234*t939;
  t3493 = t1894 + t3457;
  t3614 = t3493*t1860;
  t3668 = -0.500001190325*t797;
  t3788 = 1. + t3668;
  t3807 = t3788*t2393;
  t3809 = 0. + t3394 + t3614 + t3807;
  t3830 = -0.104528*t3811;
  t3848 = 0. + t3830;
  t3889 = 0.994522*t3811;
  t3910 = 0. + t3889;
  t4372 = Cos(var1[13]);
  t4166 = -0.994522*t3811;
  t4181 = 0. + t4166;
  t4206 = t4181*t3809;
  t4233 = 0.103955395616*t3239*t2428;
  t4243 = -0.9890740084840001*t3239;
  t4282 = 1. + t4243;
  t4287 = t4282*t2944;
  t4309 = 0. + t4206 + t4233 + t4287;
  t4162 = Sin(var1[13]);
  t4376 = 0.104528*t3811;
  t4378 = 0. + t4376;
  t4401 = t4378*t3809;
  t4455 = -0.010926102783999999*t3239;
  t4468 = 1. + t4455;
  t4484 = t4468*t2428;
  t4490 = 0.103955395616*t3239*t2944;
  t4520 = 0. + t4401 + t4484 + t4490;
  t4027 = t3368*t3809;
  t4068 = t3848*t2428;
  t4072 = t3910*t2944;
  t4073 = 0. + t4027 + t4068 + t4072;
  t4153 = Cos(var1[5]);
  t4661 = t4372*t4309;
  t4698 = -1.*t4162*t4520;
  t4719 = 0. + t4661 + t4698;
  t4310 = t4162*t4309;
  t4535 = t4372*t4520;
  t4541 = 0. + t4310 + t4535;
  t4722 = Sin(var1[5]);
  t4910 = 0.104528*t1146;
  t4913 = 0.10338592795881554*t1124;
  t4915 = t4910 + t4913;
  t4886 = 0.994522*t1654;
  t4887 = 0.010866249592949247*t1124;
  t4892 = t4886 + t4887;
  t4925 = 0.994522*t2277;
  t4929 = 0.104528*t2329;
  t4930 = t4925 + t4929;
  t4907 = t1648*t4892;
  t4923 = t4915*t1072;
  t4937 = t1990*t4930;
  t4959 = 0. + t4907 + t4923 + t4937;
  t4962 = t2544*t4915;
  t4980 = t4892*t2570;
  t5014 = t2739*t4930;
  t5098 = 0. + t4962 + t4980 + t5014;
  t5114 = t4915*t3387;
  t5120 = t4892*t3493;
  t5123 = t3788*t4930;
  t5126 = 0. + t5114 + t5120 + t5123;
  t5258 = t4181*t5126;
  t5267 = 0.103955395616*t3239*t4959;
  t5272 = t4282*t5098;
  t5274 = 0. + t5258 + t5267 + t5272;
  t5293 = t4378*t5126;
  t5303 = t4468*t4959;
  t5309 = 0.103955395616*t3239*t5098;
  t5311 = 0. + t5293 + t5303 + t5309;
  t5138 = t3368*t5126;
  t5141 = t3848*t4959;
  t5142 = t3910*t5098;
  t5143 = 0. + t5138 + t5141 + t5142;
  t4742 = Sin(var1[4]);
  t4759 = Cos(var1[4]);
  t5326 = t4372*t5274;
  t5327 = -1.*t4162*t5311;
  t5343 = 0. + t5326 + t5327;
  t5287 = t4162*t5274;
  t5313 = t4372*t5311;
  t5316 = 0. + t5287 + t5313;
  t5421 = 0.380588*t1146;
  t5424 = -0.004158319780035616*t1124;
  t5446 = 0.92388*t1605;
  t5449 = t5421 + t5424 + t5446;
  t5453 = -0.040001*t1654;
  t5469 = 0.0395641761067022*t1124;
  t5473 = 0.92388*t1770;
  t5476 = t5453 + t5469 + t5473;
  t5491 = 0.92388*t2065;
  t5506 = -0.040001*t2277;
  t5509 = 0.380588*t2329;
  t5520 = t5491 + t5506 + t5509;
  t5452 = t1072*t5449;
  t5490 = t1648*t5476;
  t5522 = t1990*t5520;
  t5534 = 0. + t5452 + t5490 + t5522;
  t5536 = t2544*t5449;
  t5538 = t2570*t5476;
  t5551 = t2739*t5520;
  t5558 = 0. + t5536 + t5538 + t5551;
  t5574 = t3387*t5449;
  t5585 = t3493*t5476;
  t5588 = t3788*t5520;
  t5589 = 0. + t5574 + t5585 + t5588;
  t5722 = t4181*t5589;
  t5723 = 0.103955395616*t3239*t5534;
  t5733 = t4282*t5558;
  t5736 = 0. + t5722 + t5723 + t5733;
  t5744 = t4378*t5589;
  t5746 = t4468*t5534;
  t5750 = 0.103955395616*t3239*t5558;
  t5757 = 0. + t5744 + t5746 + t5750;
  t5635 = t3368*t5589;
  t5638 = t3848*t5534;
  t5639 = t3910*t5558;
  t5645 = 0. + t5635 + t5638 + t5639;
  t5773 = t4372*t5736;
  t5776 = -1.*t4162*t5757;
  t5780 = 0. + t5773 + t5776;
  t5740 = t4162*t5736;
  t5768 = t4372*t5757;
  t5771 = 0. + t5740 + t5768;
  t5856 = Cos(var1[3]);
  t5873 = Sin(var1[3]);
  t5861 = t5856*t4153*t4742;
  t5874 = t5873*t4722;
  t5875 = t5861 + t5874;
  t5909 = -1.*t4153*t5873;
  t5910 = t5856*t4742*t4722;
  t5912 = t5909 + t5910;
  t5903 = -1.*t4162*t5875;
  t5913 = t4372*t5912;
  t5919 = t5903 + t5913;
  t5930 = t4372*t5875;
  t5938 = t4162*t5912;
  t5939 = t5930 + t5938;
  t5857 = t5856*t4759*t3910;
  t5921 = 0.103955395616*t3239*t5919;
  t5940 = t4282*t5939;
  t5952 = t5857 + t5921 + t5940;
  t5959 = t5856*t4759*t3848;
  t5965 = t4468*t5919;
  t5966 = 0.103955395616*t3239*t5939;
  t5992 = t5959 + t5965 + t5966;
  t6013 = t3368*t5856*t4759;
  t6026 = t4378*t5919;
  t6027 = t4181*t5939;
  t6029 = t6013 + t6026 + t6027;
  t5956 = t2739*t5952;
  t5994 = t1990*t5992;
  t6030 = t3788*t6029;
  t6033 = t5956 + t5994 + t6030;
  t6037 = t2544*t5952;
  t6043 = t1072*t5992;
  t6044 = t3387*t6029;
  t6046 = t6037 + t6043 + t6044;
  t6054 = t2570*t5952;
  t6069 = t1648*t5992;
  t6076 = t3493*t6029;
  t6081 = t6054 + t6069 + t6076;
  t389 = 6.661520000061927e-7*var2[16];
  t2495 = 0.994522*t2428;
  t2992 = 0.104528*t2944;
  t3003 = 0. + t2495 + t2992;
  t3060 = var2[14]*t3003;
  t3810 = -1.*t3368*t3809;
  t3879 = -1.*t3848*t2428;
  t3928 = -1.*t3910*t2944;
  t3994 = 0. + t3810 + t3879 + t3928;
  t4000 = var2[13]*t3994;
  t4076 = var2[5]*t4073;
  t4091 = -0.703234*t1608;
  t4109 = 0.073913*t1860;
  t4118 = 0.707107*t2393;
  t4127 = 0. + t4091 + t4109 + t4118;
  t4131 = var2[15]*t4127;
  t4544 = t4153*t4541;
  t4725 = -1.*t4719*t4722;
  t4734 = 0. + t4544 + t4725;
  t4740 = var2[4]*t4734;
  t4756 = -1.*t4073*t4742;
  t4777 = t4153*t4719;
  t4780 = t4541*t4722;
  t4787 = 0. + t4777 + t4780;
  t4789 = t4759*t4787;
  t4795 = 0. + t4756 + t4789;
  t4796 = var2[3]*t4795;
  t4810 = t389 + t3060 + t4000 + t4076 + t4131 + t4740 + t4796;
  t4868 = 1.0000001112680001*var2[16];
  t4961 = 0.994522*t4959;
  t5101 = 0.104528*t5098;
  t5106 = 0. + t4961 + t5101;
  t5111 = var2[14]*t5106;
  t5127 = -1.*t3368*t5126;
  t5128 = -1.*t3848*t4959;
  t5129 = -1.*t3910*t5098;
  t5133 = 0. + t5127 + t5128 + t5129;
  t5135 = var2[13]*t5133;
  t5186 = var2[5]*t5143;
  t5191 = 0.073913*t4892;
  t5209 = -0.703234*t4915;
  t5211 = 0.707107*t4930;
  t5248 = 0. + t5191 + t5209 + t5211;
  t5249 = var2[15]*t5248;
  t5325 = t4153*t5316;
  t5344 = -1.*t5343*t4722;
  t5352 = 0. + t5325 + t5344;
  t5354 = var2[4]*t5352;
  t5363 = -1.*t5143*t4742;
  t5364 = t4153*t5343;
  t5370 = t5316*t4722;
  t5372 = 0. + t5364 + t5370;
  t5374 = t4759*t5372;
  t5375 = 0. + t5363 + t5374;
  t5376 = var2[3]*t5375;
  t5380 = t4868 + t5111 + t5135 + t5186 + t5249 + t5354 + t5376;
  t5399 = 2.2794199999731646e-7*var2[16];
  t5535 = 0.994522*t5534;
  t5560 = 0.104528*t5558;
  t5562 = 0. + t5535 + t5560;
  t5568 = var2[14]*t5562;
  t5590 = -1.*t3368*t5589;
  t5597 = -1.*t3848*t5534;
  t5615 = -1.*t3910*t5558;
  t5621 = 0. + t5590 + t5597 + t5615;
  t5625 = var2[13]*t5621;
  t5662 = var2[5]*t5645;
  t5664 = -0.703234*t5449;
  t5685 = 0.073913*t5476;
  t5694 = 0.707107*t5520;
  t5709 = 0. + t5664 + t5685 + t5694;
  t5711 = var2[15]*t5709;
  t5772 = t4153*t5771;
  t5797 = -1.*t5780*t4722;
  t5807 = 0. + t5772 + t5797;
  t5810 = var2[4]*t5807;
  t5822 = -1.*t5645*t4742;
  t5826 = t4153*t5780;
  t5831 = t5771*t4722;
  t5833 = 0. + t5826 + t5831;
  t5839 = t4759*t5833;
  t5841 = 0. + t5822 + t5839;
  t5846 = var2[3]*t5841;
  t5848 = t5399 + t5568 + t5625 + t5662 + t5711 + t5810 + t5846;
  t6036 = t2277*t6033;
  t6049 = 0.103955395616*t1124*t6046;
  t6082 = t1654*t6081;
  t6084 = t6036 + t6049 + t6082;
  t6088 = t2329*t6033;
  t6092 = t1146*t6046;
  t6096 = 0.103955395616*t1124*t6081;
  t6102 = t6088 + t6092 + t6096;
  t6136 = t2065*t6033;
  t6143 = t1605*t6046;
  t6155 = t1770*t6081;
  t6157 = t6136 + t6143 + t6155;
  t4823 = 0.00002*t4810;
  t5397 = -0.00001*t5380;
  t5849 = 0.0014*t5848;
  t5853 = t4823 + t5397 + t5849;
  t6233 = t4153*t5873*t4742;
  t6239 = -1.*t5856*t4722;
  t6240 = t6233 + t6239;
  t6242 = t5856*t4153;
  t6247 = t5873*t4742*t4722;
  t6251 = t6242 + t6247;
  t6241 = -1.*t4162*t6240;
  t6252 = t4372*t6251;
  t6256 = t6241 + t6252;
  t6261 = t4372*t6240;
  t6266 = t4162*t6251;
  t6277 = t6261 + t6266;
  t6229 = t4759*t3910*t5873;
  t6259 = 0.103955395616*t3239*t6256;
  t6283 = t4282*t6277;
  t6290 = t6229 + t6259 + t6283;
  t6296 = t4759*t3848*t5873;
  t6308 = t4468*t6256;
  t6314 = 0.103955395616*t3239*t6277;
  t6316 = t6296 + t6308 + t6314;
  t6320 = t3368*t4759*t5873;
  t6321 = t4378*t6256;
  t6324 = t4181*t6277;
  t6325 = t6320 + t6321 + t6324;
  t6294 = t2739*t6290;
  t6317 = t1990*t6316;
  t6326 = t3788*t6325;
  t6327 = t6294 + t6317 + t6326;
  t6334 = t2544*t6290;
  t6340 = t1072*t6316;
  t6345 = t3387*t6325;
  t6347 = t6334 + t6340 + t6345;
  t6351 = t2570*t6290;
  t6363 = t1648*t6316;
  t6367 = t3493*t6325;
  t6371 = t6351 + t6363 + t6367;
  t6126 = 0.00956*t4810;
  t6127 = 0.00002*t5380;
  t6130 = 0.00003*t5848;
  t6131 = t6126 + t6127 + t6130;
  t6331 = t2277*t6327;
  t6348 = 0.103955395616*t1124*t6347;
  t6372 = t1654*t6371;
  t6376 = t6331 + t6348 + t6372;
  t6387 = t2329*t6327;
  t6394 = t1146*t6347;
  t6395 = 0.103955395616*t1124*t6371;
  t6396 = t6387 + t6394 + t6395;
  t6177 = 0.00003*t4810;
  t6186 = 0.0014*t5380;
  t6198 = 0.00048*t5848;
  t6204 = t6177 + t6186 + t6198;
  t6423 = t2065*t6327;
  t6426 = t1605*t6347;
  t6429 = t1770*t6371;
  t6433 = t6423 + t6426 + t6429;
  t6510 = -1.*t4759*t4153*t4162;
  t6514 = t4372*t4759*t4722;
  t6515 = t6510 + t6514;
  t6527 = t4372*t4759*t4153;
  t6533 = t4759*t4162*t4722;
  t6536 = t6527 + t6533;
  t6497 = -1.*t3910*t4742;
  t6517 = 0.103955395616*t3239*t6515;
  t6538 = t4282*t6536;
  t6540 = t6497 + t6517 + t6538;
  t6543 = -1.*t3848*t4742;
  t6544 = t4468*t6515;
  t6545 = 0.103955395616*t3239*t6536;
  t6547 = t6543 + t6544 + t6545;
  t6554 = -1.*t3368*t4742;
  t6556 = t4378*t6515;
  t6558 = t4181*t6536;
  t6561 = t6554 + t6556 + t6558;
  t6542 = t2739*t6540;
  t6553 = t1990*t6547;
  t6562 = t3788*t6561;
  t6564 = t6542 + t6553 + t6562;
  t6568 = t2544*t6540;
  t6569 = t1072*t6547;
  t6570 = t3387*t6561;
  t6571 = t6568 + t6569 + t6570;
  t6585 = t2570*t6540;
  t6586 = t1648*t6547;
  t6589 = t3493*t6561;
  t6594 = t6585 + t6586 + t6589;
  t6567 = t2277*t6564;
  t6583 = 0.103955395616*t1124*t6571;
  t6596 = t1654*t6594;
  t6597 = t6567 + t6583 + t6596;
  t6604 = t2329*t6564;
  t6608 = t1146*t6571;
  t6609 = 0.103955395616*t1124*t6594;
  t6611 = t6604 + t6608 + t6609;
  t6633 = t2065*t6564;
  t6634 = t1605*t6571;
  t6635 = t1770*t6594;
  t6637 = t6633 + t6634 + t6635;
  p_output1[0]=t5853*(0.994522*t6084 + 0.104528*t6102) + t6131*(0.096572*t6084 - 0.918819*t6102 + 0.382684*t6157) + (-0.040001*t6084 + 0.380588*t6102 + 0.92388*t6157)*t6204;
  p_output1[1]=t5853*(0.994522*t6376 + 0.104528*t6396) + t6131*(0.096572*t6376 - 0.918819*t6396 + 0.382684*t6433) + t6204*(-0.040001*t6376 + 0.380588*t6396 + 0.92388*t6433);
  p_output1[2]=t5853*(0.994522*t6597 + 0.104528*t6611) + t6131*(0.096572*t6597 - 0.918819*t6611 + 0.382684*t6637) + t6204*(-0.040001*t6597 + 0.380588*t6611 + 0.92388*t6637);
}



void AMWorld_left_elbow_src(double *p_output1, const double *var1,const double *var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
