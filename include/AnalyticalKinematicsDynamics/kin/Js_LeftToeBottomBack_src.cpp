/*
 * Automatically Generated from Mathematica.
 * Thu 10 Nov 2022 14:34:25 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Js_LeftToeBottomBack_src.h"

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
  double t25;
  double t35;
  double t106;
  double t108;
  double t174;
  double t191;
  double t178;
  double t193;
  double t194;
  double t282;
  double t291;
  double t292;
  double t293;
  double t309;
  double t205;
  double t209;
  double t210;
  double t332;
  double t336;
  double t338;
  double t340;
  double t354;
  double t355;
  double t344;
  double t345;
  double t351;
  double t352;
  double t384;
  double t385;
  double t401;
  double t403;
  double t404;
  double t406;
  double t408;
  double t410;
  double t414;
  double t443;
  double t456;
  double t458;
  double t13;
  double t386;
  double t390;
  double t395;
  double t299;
  double t310;
  double t316;
  double t494;
  double t496;
  double t498;
  double t438;
  double t440;
  double t441;
  double t505;
  double t511;
  double t514;
  double t318;
  double t320;
  double t322;
  double t581;
  double t593;
  double t594;
  double t599;
  double t567;
  double t574;
  double t576;
  double t740;
  double t697;
  double t706;
  double t713;
  double t796;
  double t800;
  double t730;
  double t745;
  double t748;
  double t752;
  double t754;
  double t756;
  double t758;
  double t760;
  double t771;
  double t784;
  double t803;
  double t806;
  double t822;
  double t832;
  double t837;
  double t840;
  double t846;
  double t851;
  double t871;
  double t873;
  double t761;
  double t814;
  double t863;
  double t866;
  double t890;
  double t895;
  double t902;
  double t904;
  double t937;
  double t942;
  double t1000;
  double t1003;
  double t1007;
  double t1010;
  double t905;
  double t906;
  double t917;
  double t920;
  double t1056;
  double t1066;
  double t1068;
  double t1071;
  double t1024;
  double t1030;
  double t1031;
  double t1035;
  double t1074;
  double t1077;
  double t1080;
  double t1086;
  double t955;
  double t962;
  double t965;
  double t969;
  double t979;
  double t980;
  double t870;
  double t875;
  double t879;
  double t880;
  double t945;
  double t948;
  double t951;
  double t953;
  double t1168;
  double t1170;
  double t1179;
  double t1180;
  double t1187;
  double t1188;
  double t1189;
  double t1191;
  double t1121;
  double t1126;
  double t1132;
  double t1139;
  double t1318;
  double t1325;
  double t1327;
  double t1336;
  double t1239;
  double t1240;
  double t1246;
  double t1247;
  double t1339;
  double t1342;
  double t1317;
  double t1330;
  double t1343;
  double t1346;
  double t1350;
  double t1354;
  double t1359;
  double t1362;
  double t1370;
  double t1379;
  double t1381;
  double t1387;
  double t1392;
  double t1393;
  double t1394;
  double t1397;
  double t1401;
  double t1406;
  double t1412;
  double t1413;
  double t1347;
  double t1388;
  double t1408;
  double t1409;
  double t1428;
  double t1430;
  double t1436;
  double t1439;
  double t1466;
  double t1468;
  double t984;
  double t988;
  double t989;
  double t995;
  double t1411;
  double t1414;
  double t1416;
  double t1419;
  double t1510;
  double t1512;
  double t1518;
  double t1527;
  double t1581;
  double t1582;
  double t1585;
  double t1590;
  double t1469;
  double t1471;
  double t1473;
  double t1481;
  double t1595;
  double t1597;
  double t1598;
  double t1605;
  double t1562;
  double t1563;
  double t1564;
  double t1569;
  double t1440;
  double t1446;
  double t1447;
  double t1448;
  double t1534;
  double t1544;
  double t1485;
  double t1486;
  double t1487;
  double t1492;
  double t1677;
  double t1678;
  double t1679;
  double t1680;
  double t1689;
  double t1695;
  double t1696;
  double t1699;
  double t1785;
  double t1790;
  double t1791;
  double t1794;
  double t1728;
  double t1736;
  double t1744;
  double t1747;
  double t1858;
  double t1859;
  double t1643;
  double t1649;
  double t1653;
  double t1666;
  double t1892;
  double t1893;
  double t1784;
  double t1793;
  double t1798;
  double t1807;
  double t1812;
  double t1814;
  double t1823;
  double t1827;
  double t1830;
  double t1837;
  double t1840;
  double t1844;
  double t1848;
  double t1851;
  double t1854;
  double t1855;
  double t1861;
  double t1864;
  double t1820;
  double t1852;
  double t1880;
  double t1884;
  double t1931;
  double t1935;
  double t1912;
  double t1915;
  double t1917;
  double t1926;
  double t1550;
  double t1554;
  double t1555;
  double t1556;
  double t1975;
  double t1977;
  double t1981;
  double t1983;
  double t1906;
  double t1909;
  double t1910;
  double t1911;
  double t2033;
  double t2035;
  double t2037;
  double t2038;
  double t1985;
  double t1986;
  double t1987;
  double t1996;
  double t1958;
  double t1961;
  double t1962;
  double t1963;
  double t2057;
  double t2060;
  double t2061;
  double t2065;
  double t1887;
  double t1888;
  double t1896;
  double t1899;
  double t1936;
  double t1937;
  double t1942;
  double t1943;
  double t2011;
  double t2013;
  double t2109;
  double t2111;
  double t2112;
  double t2113;
  double t2130;
  double t2131;
  double t2132;
  double t2139;
  double t2255;
  double t2008;
  double t2014;
  double t2017;
  double t2018;
  double t2285;
  double t2288;
  double t2289;
  double t2304;
  double t2309;
  double t2317;
  double t2326;
  double t2097;
  double t2098;
  double t2099;
  double t2101;
  double t2365;
  double t2374;
  double t2266;
  double t2273;
  double t2259;
  double t2263;
  double t2188;
  double t2191;
  double t2195;
  double t2199;
  double t2404;
  double t2411;
  double t2418;
  double t2419;
  double t2422;
  double t2424;
  double t2428;
  double t2432;
  double t2434;
  double t2436;
  double t2438;
  double t2443;
  double t2338;
  double t2344;
  double t2421;
  double t2433;
  double t2445;
  double t2446;
  double t2466;
  double t2473;
  double t2485;
  double t2486;
  double t2311;
  double t2315;
  double t2328;
  double t2330;
  double t2451;
  double t2452;
  double t2461;
  double t2462;
  double t2333;
  double t2361;
  double t2376;
  double t2377;
  double t2567;
  double t2575;
  double t2580;
  double t2583;
  double t2522;
  double t2531;
  double t2535;
  double t2536;
  double t2587;
  double t2589;
  double t2591;
  double t2605;
  double t2293;
  double t2295;
  double t2390;
  double t2391;
  double t2394;
  double t2402;
  double t2496;
  double t2502;
  double t2508;
  double t2510;
  double t2680;
  double t2682;
  double t2688;
  double t2689;
  double t2708;
  double t2709;
  double t2713;
  double t2728;
  double t2264;
  double t2281;
  double t2299;
  double t2300;
  double t2812;
  double t2814;
  double t2815;
  double t2797;
  double t2798;
  double t2800;
  double t2622;
  double t2629;
  double t2632;
  double t2634;
  double t2818;
  double t2821;
  double t2874;
  double t2876;
  double t2862;
  double t2867;
  double t2653;
  double t2666;
  double t2668;
  double t2669;
  double t2829;
  double t2830;
  double t2903;
  double t2905;
  double t2910;
  double t2911;
  double t2918;
  double t2925;
  double t2926;
  double t2927;
  double t2933;
  double t2937;
  double t2938;
  double t2942;
  double t2833;
  double t2838;
  double t2915;
  double t2930;
  double t2943;
  double t2945;
  double t2961;
  double t2964;
  double t2967;
  double t2968;
  double t2848;
  double t2853;
  double t2884;
  double t2887;
  double t2897;
  double t2898;
  double t2981;
  double t2993;
  double t2998;
  double t2999;
  double t3018;
  double t3019;
  double t3020;
  double t3022;
  double t2809;
  double t2817;
  double t2822;
  double t2823;
  double t2950;
  double t2954;
  double t2959;
  double t2960;
  double t2832;
  double t2842;
  double t2845;
  double t2846;
  double t3050;
  double t3053;
  double t3056;
  double t3066;
  double t3004;
  double t3006;
  double t3007;
  double t3011;
  double t3069;
  double t3072;
  double t3076;
  double t3079;
  double t2858;
  double t2869;
  double t2877;
  double t2879;
  double t3025;
  double t3027;
  double t3028;
  double t3029;
  double t3121;
  double t3122;
  double t3125;
  double t3127;
  double t3129;
  double t3132;
  double t3135;
  double t3136;
  t25 = Cos(var1[3]);
  t35 = Sin(var1[3]);
  t106 = Cos(var1[4]);
  t108 = Sin(var1[4]);
  t174 = Cos(var1[5]);
  t191 = Sin(var1[5]);
  t178 = t25*t174*t108;
  t193 = t35*t191;
  t194 = t178 + t193;
  t282 = Cos(var1[6]);
  t291 = -1.*t174*t35;
  t292 = t25*t108*t191;
  t293 = t291 + t292;
  t309 = Sin(var1[6]);
  t205 = t174*t35*t108;
  t209 = -1.*t25*t191;
  t210 = t205 + t209;
  t332 = -1.*t282;
  t336 = 1. + t332;
  t338 = 0.091*t336;
  t340 = 0. + t338;
  t354 = 0.091*t309;
  t355 = 0. + t354;
  t344 = t25*t174;
  t345 = t35*t108*t191;
  t351 = t344 + t345;
  t352 = t340*t351;
  t384 = t210*t355;
  t385 = 0. + var1[1] + t352 + t384;
  t401 = -1.*var1[2];
  t403 = -1.*t106*t340*t191;
  t404 = -1.*t106*t174*t355;
  t406 = 0. + t401 + t403 + t404;
  t408 = t282*t351;
  t410 = -1.*t210*t309;
  t414 = t408 + t410;
  t443 = t282*t210;
  t456 = t351*t309;
  t458 = t443 + t456;
  t13 = -1.*var1[0];
  t386 = t106*t282*t191;
  t390 = -1.*t106*t174*t309;
  t395 = t386 + t390;
  t299 = t282*t293;
  t310 = -1.*t194*t309;
  t316 = t299 + t310;
  t494 = -1.*t340*t293;
  t496 = -1.*t194*t355;
  t498 = 0. + t13 + t494 + t496;
  t438 = t106*t174*t282;
  t440 = t106*t191*t309;
  t441 = t438 + t440;
  t505 = t106*t340*t191;
  t511 = t106*t174*t355;
  t514 = 0. + var1[2] + t505 + t511;
  t318 = t282*t194;
  t320 = t293*t309;
  t322 = t318 + t320;
  t581 = -1.*var1[1];
  t593 = -1.*t340*t351;
  t594 = -1.*t210*t355;
  t599 = 0. + t581 + t593 + t594;
  t567 = t340*t293;
  t574 = t194*t355;
  t576 = 0. + var1[0] + t567 + t574;
  t740 = Sin(var1[7]);
  t697 = Cos(var1[7]);
  t706 = -1.*t697;
  t713 = 1. + t706;
  t796 = 0.366501*t740;
  t800 = 0. + t796;
  t730 = -0.04500040093286238*t713;
  t745 = -0.930418*t740;
  t748 = 0. + t745;
  t752 = 0.07877663122399998*t748;
  t754 = -0.366501*t740;
  t756 = 0. + t754;
  t758 = 0.031030906668*t756;
  t760 = 0. + t730 + t752 + t758;
  t771 = -3.2909349868922137e-7*var1[7];
  t784 = 0.03103092645718495*t713;
  t803 = -0.045000372235*t800;
  t806 = t771 + t784 + t803;
  t822 = 1.296332362046933e-7*var1[7];
  t832 = 0.07877668146182712*t713;
  t837 = 0.930418*t740;
  t840 = 0. + t837;
  t846 = -0.045000372235*t840;
  t851 = t822 + t832 + t846;
  t871 = -0.134322983001*t713;
  t873 = 1. + t871;
  t761 = t108*t760;
  t814 = -1.*t441*t806;
  t863 = -1.*t395*t851;
  t866 = 0. + t401 + t403 + t404 + t761 + t814 + t863;
  t890 = t106*t35*t760;
  t895 = t458*t806;
  t902 = t414*t851;
  t904 = 0. + var1[1] + t352 + t384 + t890 + t895 + t902;
  t937 = -0.8656776547239999*t713;
  t942 = 1. + t937;
  t1000 = -0.340999127418*t713*t316;
  t1003 = t873*t322;
  t1007 = t25*t106*t800;
  t1010 = t1000 + t1003 + t1007;
  t905 = -0.340999127418*t713*t395;
  t906 = t873*t441;
  t917 = -1.*t108*t800;
  t920 = t905 + t906 + t917;
  t1056 = -1.*t108*t760;
  t1066 = t441*t806;
  t1068 = t395*t851;
  t1071 = 0. + var1[2] + t505 + t511 + t1056 + t1066 + t1068;
  t1024 = t942*t316;
  t1030 = -0.340999127418*t713*t322;
  t1031 = t25*t106*t840;
  t1035 = t1024 + t1030 + t1031;
  t1074 = -1.*t25*t106*t760;
  t1077 = -1.*t322*t806;
  t1080 = -1.*t316*t851;
  t1086 = 0. + t13 + t494 + t496 + t1074 + t1077 + t1080;
  t955 = t942*t395;
  t962 = -0.340999127418*t713*t441;
  t965 = -1.*t108*t840;
  t969 = t955 + t962 + t965;
  t979 = -1.000000637725*t713;
  t980 = 1. + t979;
  t870 = -0.340999127418*t713*t414;
  t875 = t873*t458;
  t879 = t106*t35*t800;
  t880 = t870 + t875 + t879;
  t945 = t942*t414;
  t948 = -0.340999127418*t713*t458;
  t951 = t106*t35*t840;
  t953 = t945 + t948 + t951;
  t1168 = -1.*t106*t35*t760;
  t1170 = -1.*t458*t806;
  t1179 = -1.*t414*t851;
  t1180 = 0. + t581 + t593 + t594 + t1168 + t1170 + t1179;
  t1187 = t25*t106*t760;
  t1188 = t322*t806;
  t1189 = t316*t851;
  t1191 = 0. + var1[0] + t567 + t574 + t1187 + t1188 + t1189;
  t1121 = t106*t980*t35;
  t1126 = t414*t748;
  t1132 = t458*t756;
  t1139 = t1121 + t1126 + t1132;
  t1318 = Cos(var1[8]);
  t1325 = -1.*t1318;
  t1327 = 1. + t1325;
  t1336 = Sin(var1[8]);
  t1239 = -1.*t980*t108;
  t1240 = t395*t748;
  t1246 = t441*t756;
  t1247 = t1239 + t1240 + t1246;
  t1339 = -0.366501*t1336;
  t1342 = 0. + t1339;
  t1317 = 3.2909349868922137e-7*var1[8];
  t1330 = 0.055653945343889656*t1327;
  t1343 = -0.045000372235*t1342;
  t1346 = t1317 + t1330 + t1343;
  t1350 = -0.04500040093286238*t1327;
  t1354 = -0.930418*t1336;
  t1359 = 0. + t1354;
  t1362 = -0.141285834136*t1359;
  t1370 = 0.366501*t1336;
  t1379 = 0. + t1370;
  t1381 = 0.055653909852*t1379;
  t1387 = 0. + t1350 + t1362 + t1381;
  t1392 = 1.296332362046933e-7*var1[8];
  t1393 = -0.14128592423750855*t1327;
  t1394 = 0.930418*t1336;
  t1397 = 0. + t1394;
  t1401 = -0.045000372235*t1397;
  t1406 = t1392 + t1393 + t1401;
  t1412 = -0.134322983001*t1327;
  t1413 = 1. + t1412;
  t1347 = t953*t1346;
  t1388 = t1139*t1387;
  t1408 = t880*t1406;
  t1409 = 0. + var1[1] + t352 + t384 + t890 + t895 + t902 + t1347 + t1388 + t1408;
  t1428 = -1.*t969*t1346;
  t1430 = -1.*t1247*t1387;
  t1436 = -1.*t920*t1406;
  t1439 = 0. + t401 + t403 + t404 + t761 + t814 + t863 + t1428 + t1430 + t1436;
  t1466 = -0.8656776547239999*t1327;
  t1468 = 1. + t1466;
  t984 = t25*t106*t980;
  t988 = t316*t748;
  t989 = t322*t756;
  t995 = t984 + t988 + t989;
  t1411 = 0.340999127418*t1327*t920;
  t1414 = t1413*t969;
  t1416 = t1247*t1342;
  t1419 = t1411 + t1414 + t1416;
  t1510 = 0.340999127418*t1327*t1010;
  t1512 = t1413*t1035;
  t1518 = t995*t1342;
  t1527 = t1510 + t1512 + t1518;
  t1581 = -1.*t1035*t1346;
  t1582 = -1.*t995*t1387;
  t1585 = -1.*t1010*t1406;
  t1590 = 0. + t13 + t494 + t496 + t1074 + t1077 + t1080 + t1581 + t1582 + t1585;
  t1469 = t1468*t920;
  t1471 = 0.340999127418*t1327*t969;
  t1473 = t1247*t1397;
  t1481 = t1469 + t1471 + t1473;
  t1595 = t969*t1346;
  t1597 = t1247*t1387;
  t1598 = t920*t1406;
  t1605 = 0. + var1[2] + t505 + t511 + t1056 + t1066 + t1068 + t1595 + t1597 + t1598;
  t1562 = t1468*t1010;
  t1563 = 0.340999127418*t1327*t1035;
  t1564 = t995*t1397;
  t1569 = t1562 + t1563 + t1564;
  t1440 = 0.340999127418*t1327*t880;
  t1446 = t1413*t953;
  t1447 = t1139*t1342;
  t1448 = t1440 + t1446 + t1447;
  t1534 = -1.000000637725*t1327;
  t1544 = 1. + t1534;
  t1485 = t1468*t880;
  t1486 = 0.340999127418*t1327*t953;
  t1487 = t1139*t1397;
  t1492 = t1485 + t1486 + t1487;
  t1677 = -1.*t953*t1346;
  t1678 = -1.*t1139*t1387;
  t1679 = -1.*t880*t1406;
  t1680 = 0. + t581 + t593 + t594 + t1168 + t1170 + t1179 + t1677 + t1678 + t1679;
  t1689 = t1035*t1346;
  t1695 = t995*t1387;
  t1696 = t1010*t1406;
  t1699 = 0. + var1[0] + t567 + t574 + t1187 + t1188 + t1189 + t1689 + t1695 + t1696;
  t1785 = Cos(var1[9]);
  t1790 = -1.*t1785;
  t1791 = 1. + t1790;
  t1794 = Sin(var1[9]);
  t1728 = t1544*t1247;
  t1736 = t920*t1359;
  t1744 = t969*t1379;
  t1747 = t1728 + t1736 + t1744;
  t1858 = -0.930418*t1794;
  t1859 = 0. + t1858;
  t1643 = t1544*t1139;
  t1649 = t880*t1359;
  t1653 = t953*t1379;
  t1666 = t1643 + t1649 + t1653;
  t1892 = -0.8656776547239999*t1791;
  t1893 = 1. + t1892;
  t1784 = -1.5981976069815686e-7*var1[9];
  t1793 = 0.08675267452931407*t1791;
  t1798 = 0.366501*t1794;
  t1807 = 0. + t1798;
  t1812 = 0.039853013046*t1807;
  t1814 = t1784 + t1793 + t1812;
  t1823 = 0.039853038461262744*t1791;
  t1827 = -0.366501*t1794;
  t1830 = 0. + t1827;
  t1837 = 0.086752619205*t1830;
  t1840 = 0.930418*t1794;
  t1844 = 0. + t1840;
  t1848 = -0.22023459268999998*t1844;
  t1851 = 0. + t1823 + t1837 + t1848;
  t1854 = -6.295460977284962e-8*var1[9];
  t1855 = -0.22023473313910558*t1791;
  t1861 = 0.039853013046*t1859;
  t1864 = t1854 + t1855 + t1861;
  t1820 = -1.*t1814*t1419;
  t1852 = -1.*t1851*t1747;
  t1880 = -1.*t1864*t1481;
  t1884 = 0. + t401 + t403 + t404 + t761 + t814 + t863 + t1428 + t1820 + t1430 + t1852 + t1436 + t1880;
  t1931 = -0.134322983001*t1791;
  t1935 = 1. + t1931;
  t1912 = t1814*t1448;
  t1915 = t1851*t1666;
  t1917 = t1864*t1492;
  t1926 = 0. + var1[1] + t352 + t384 + t890 + t895 + t902 + t1347 + t1912 + t1388 + t1915 + t1408 + t1917;
  t1550 = t1544*t995;
  t1554 = t1010*t1359;
  t1555 = t1035*t1379;
  t1556 = t1550 + t1554 + t1555;
  t1975 = 0.340999127418*t1791*t1527;
  t1977 = t1859*t1556;
  t1981 = t1893*t1569;
  t1983 = t1975 + t1977 + t1981;
  t1906 = 0.340999127418*t1791*t1419;
  t1909 = t1859*t1747;
  t1910 = t1893*t1481;
  t1911 = t1906 + t1909 + t1910;
  t2033 = t1814*t1419;
  t2035 = t1851*t1747;
  t2037 = t1864*t1481;
  t2038 = 0. + var1[2] + t505 + t511 + t1056 + t1066 + t1068 + t1595 + t2033 + t1597 + t2035 + t1598 + t2037;
  t1985 = t1935*t1527;
  t1986 = t1807*t1556;
  t1987 = 0.340999127418*t1791*t1569;
  t1996 = t1985 + t1986 + t1987;
  t1958 = t1935*t1419;
  t1961 = t1807*t1747;
  t1962 = 0.340999127418*t1791*t1481;
  t1963 = t1958 + t1961 + t1962;
  t2057 = -1.*t1814*t1527;
  t2060 = -1.*t1851*t1556;
  t2061 = -1.*t1864*t1569;
  t2065 = 0. + t13 + t494 + t496 + t1074 + t1077 + t1080 + t1581 + t2057 + t1582 + t2060 + t1585 + t2061;
  t1887 = 0.340999127418*t1791*t1448;
  t1888 = t1859*t1666;
  t1896 = t1893*t1492;
  t1899 = t1887 + t1888 + t1896;
  t1936 = t1935*t1448;
  t1937 = t1807*t1666;
  t1942 = 0.340999127418*t1791*t1492;
  t1943 = t1936 + t1937 + t1942;
  t2011 = -1.000000637725*t1791;
  t2013 = 1. + t2011;
  t2109 = t1814*t1527;
  t2111 = t1851*t1556;
  t2112 = t1864*t1569;
  t2113 = 0. + var1[0] + t567 + t574 + t1187 + t1188 + t1189 + t1689 + t2109 + t1695 + t2111 + t1696 + t2112;
  t2130 = -1.*t1814*t1448;
  t2131 = -1.*t1851*t1666;
  t2132 = -1.*t1864*t1492;
  t2139 = 0. + t581 + t593 + t594 + t1168 + t1170 + t1179 + t1677 + t2130 + t1678 + t2131 + t1679 + t2132;
  t2255 = Sin(var1[10]);
  t2008 = t1830*t1527;
  t2014 = t2013*t1556;
  t2017 = t1844*t1569;
  t2018 = t2008 + t2014 + t2017;
  t2285 = Cos(var1[10]);
  t2288 = -1.*t2285;
  t2289 = 1. + t2288;
  t2304 = -0.8656776547239999*t2289;
  t2309 = 1. + t2304;
  t2317 = -0.930418*t2255;
  t2326 = 0. + t2317;
  t2097 = t1830*t1448;
  t2098 = t2013*t1666;
  t2099 = t1844*t1492;
  t2101 = t2097 + t2098 + t2099;
  t2365 = 0.366501*t2255;
  t2374 = 0. + t2365;
  t2266 = -0.366501*t2255;
  t2273 = 0. + t2266;
  t2259 = 0.930418*t2255;
  t2263 = 0. + t2259;
  t2188 = t1830*t1419;
  t2191 = t2013*t1747;
  t2195 = t1844*t1481;
  t2199 = t2188 + t2191 + t2195;
  t2404 = 2.281945176511838e-8*var1[10];
  t2411 = -0.5905366811997648*t2289;
  t2418 = -0.262809976934*t2326;
  t2419 = t2404 + t2411 + t2418;
  t2422 = 5.7930615939377813e-8*var1[10];
  t2424 = 0.23261833304643187*t2289;
  t2428 = -0.262809976934*t2374;
  t2432 = t2422 + t2424 + t2428;
  t2434 = -0.26281014453449253*t2289;
  t2436 = 0.23261818470000004*t2273;
  t2438 = -0.5905363046000001*t2263;
  t2443 = 0. + t2434 + t2436 + t2438;
  t2338 = -0.134322983001*t2289;
  t2344 = 1. + t2338;
  t2421 = -1.*t2419*t1911;
  t2433 = -1.*t2432*t1963;
  t2445 = -1.*t2443*t2199;
  t2446 = 0. + t401 + t403 + t404 + t761 + t814 + t863 + t2421 + t2433 + t2445 + t1428 + t1820 + t1430 + t1852 + t1436 + t1880;
  t2466 = t2419*t1899;
  t2473 = t2432*t1943;
  t2485 = t2443*t2101;
  t2486 = 0. + var1[1] + t352 + t384 + t890 + t895 + t902 + t2466 + t2473 + t2485 + t1347 + t1912 + t1388 + t1915 + t1408 + t1917;
  t2311 = t2309*t1983;
  t2315 = 0.340999127418*t2289*t1996;
  t2328 = t2326*t2018;
  t2330 = t2311 + t2315 + t2328;
  t2451 = t2309*t1911;
  t2452 = 0.340999127418*t2289*t1963;
  t2461 = t2326*t2199;
  t2462 = t2451 + t2452 + t2461;
  t2333 = 0.340999127418*t2289*t1983;
  t2361 = t2344*t1996;
  t2376 = t2374*t2018;
  t2377 = t2333 + t2361 + t2376;
  t2567 = t2419*t1911;
  t2575 = t2432*t1963;
  t2580 = t2443*t2199;
  t2583 = 0. + var1[2] + t505 + t511 + t1056 + t1066 + t1068 + t2567 + t2575 + t2580 + t1595 + t2033 + t1597 + t2035 + t1598 + t2037;
  t2522 = 0.340999127418*t2289*t1911;
  t2531 = t2344*t1963;
  t2535 = t2374*t2199;
  t2536 = t2522 + t2531 + t2535;
  t2587 = -1.*t2419*t1983;
  t2589 = -1.*t2432*t1996;
  t2591 = -1.*t2443*t2018;
  t2605 = 0. + t13 + t494 + t496 + t1074 + t1077 + t1080 + t2587 + t2589 + t2591 + t1581 + t2057 + t1582 + t2060 + t1585 + t2061;
  t2293 = -1.000000637725*t2289;
  t2295 = 1. + t2293;
  t2390 = t2309*t1899;
  t2391 = 0.340999127418*t2289*t1943;
  t2394 = t2326*t2101;
  t2402 = t2390 + t2391 + t2394;
  t2496 = 0.340999127418*t2289*t1899;
  t2502 = t2344*t1943;
  t2508 = t2374*t2101;
  t2510 = t2496 + t2502 + t2508;
  t2680 = t2419*t1983;
  t2682 = t2432*t1996;
  t2688 = t2443*t2018;
  t2689 = 0. + var1[0] + t567 + t574 + t1187 + t1188 + t1189 + t2680 + t2682 + t2688 + t1689 + t2109 + t1695 + t2111 + t1696 + t2112;
  t2708 = -1.*t2419*t1899;
  t2709 = -1.*t2432*t1943;
  t2713 = -1.*t2443*t2101;
  t2728 = 0. + t581 + t593 + t594 + t1168 + t1170 + t1179 + t2708 + t2709 + t2713 + t1677 + t2130 + t1678 + t2131 + t1679 + t2132;
  t2264 = t2263*t1983;
  t2281 = t2273*t1996;
  t2299 = t2295*t2018;
  t2300 = t2264 + t2281 + t2299;
  t2812 = Cos(var1[11]);
  t2814 = -1.*t2812;
  t2815 = 1. + t2814;
  t2797 = Sin(var1[11]);
  t2798 = 0.366501*t2797;
  t2800 = 0. + t2798;
  t2622 = t2263*t1899;
  t2629 = t2273*t1943;
  t2632 = t2295*t2101;
  t2634 = t2622 + t2629 + t2632;
  t2818 = -0.134322983001*t2815;
  t2821 = 1. + t2818;
  t2874 = -0.366501*t2797;
  t2876 = 0. + t2874;
  t2862 = 0.930418*t2797;
  t2867 = 0. + t2862;
  t2653 = t2263*t1911;
  t2666 = t2273*t1963;
  t2668 = t2295*t2199;
  t2669 = t2653 + t2666 + t2668;
  t2829 = -0.930418*t2797;
  t2830 = 0. + t2829;
  t2903 = 0.06199697675299678*t2815;
  t2905 = 0.324290713329*t2876;
  t2910 = -0.823260828522*t2867;
  t2911 = 0. + t2903 + t2905 + t2910;
  t2918 = 2.95447451120871e-8*var1[11];
  t2925 = -0.8232613535360118*t2815;
  t2926 = 0.061996937216*t2830;
  t2927 = t2918 + t2925 + t2926;
  t2933 = 7.500378623168247e-8*var1[11];
  t2937 = 0.32429092013729516*t2815;
  t2938 = 0.061996937216*t2800;
  t2942 = t2933 + t2937 + t2938;
  t2833 = -0.8656776547239999*t2815;
  t2838 = 1. + t2833;
  t2915 = -1.*t2911*t2669;
  t2930 = -1.*t2927*t2462;
  t2943 = -1.*t2942*t2536;
  t2945 = 0. + t401 + t403 + t404 + t761 + t814 + t863 + t2915 + t2930 + t2943 + t2421 + t2433 + t2445 + t1428 + t1820 + t1430 + t1852 + t1436 + t1880;
  t2961 = t2911*t2634;
  t2964 = t2927*t2402;
  t2967 = t2942*t2510;
  t2968 = 0. + var1[1] + t352 + t384 + t890 + t895 + t902 + t2961 + t2964 + t2967 + t2466 + t2473 + t2485 + t1347 + t1912 + t1388 + t1915 + t1408 + t1917;
  t2848 = -1.000000637725*t2815;
  t2853 = 1. + t2848;
  t2884 = t2800*t2634;
  t2887 = 0.340999127418*t2815*t2402;
  t2897 = t2821*t2510;
  t2898 = t2884 + t2887 + t2897;
  t2981 = t2830*t2634;
  t2993 = t2838*t2402;
  t2998 = 0.340999127418*t2815*t2510;
  t2999 = t2981 + t2993 + t2998;
  t3018 = t2853*t2634;
  t3019 = t2867*t2402;
  t3020 = t2876*t2510;
  t3022 = t3018 + t3019 + t3020;
  t2809 = t2800*t2300;
  t2817 = 0.340999127418*t2815*t2330;
  t2822 = t2821*t2377;
  t2823 = t2809 + t2817 + t2822;
  t2950 = t2800*t2669;
  t2954 = 0.340999127418*t2815*t2462;
  t2959 = t2821*t2536;
  t2960 = t2950 + t2954 + t2959;
  t2832 = t2830*t2300;
  t2842 = t2838*t2330;
  t2845 = 0.340999127418*t2815*t2377;
  t2846 = t2832 + t2842 + t2845;
  t3050 = t2911*t2669;
  t3053 = t2927*t2462;
  t3056 = t2942*t2536;
  t3066 = 0. + var1[2] + t505 + t511 + t1056 + t1066 + t1068 + t3050 + t3053 + t3056 + t2567 + t2575 + t2580 + t1595 + t2033 + t1597 + t2035 + t1598 + t2037;
  t3004 = t2830*t2669;
  t3006 = t2838*t2462;
  t3007 = 0.340999127418*t2815*t2536;
  t3011 = t3004 + t3006 + t3007;
  t3069 = -1.*t2911*t2300;
  t3072 = -1.*t2927*t2330;
  t3076 = -1.*t2942*t2377;
  t3079 = 0. + t13 + t494 + t496 + t1074 + t1077 + t1080 + t3069 + t3072 + t3076 + t2587 + t2589 + t2591 + t1581 + t2057 + t1582 + t2060 + t1585 + t2061;
  t2858 = t2853*t2300;
  t2869 = t2867*t2330;
  t2877 = t2876*t2377;
  t2879 = t2858 + t2869 + t2877;
  t3025 = t2853*t2669;
  t3027 = t2867*t2462;
  t3028 = t2876*t2536;
  t3029 = t3025 + t3027 + t3028;
  t3121 = t2911*t2300;
  t3122 = t2927*t2330;
  t3125 = t2942*t2377;
  t3127 = 0. + var1[0] + t567 + t574 + t1187 + t1188 + t1189 + t3121 + t3122 + t3125 + t2680 + t2682 + t2688 + t1689 + t2109 + t1695 + t2111 + t1696 + t2112;
  t3129 = -1.*t2911*t2634;
  t3132 = -1.*t2927*t2402;
  t3135 = -1.*t2942*t2510;
  t3136 = 0. + t581 + t593 + t594 + t1168 + t1170 + t1179 + t3129 + t3132 + t3135 + t2708 + t2709 + t2713 + t1677 + t2130 + t1678 + t2131 + t1679 + t2132;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=1.;
  p_output1[8]=0;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=0;
  p_output1[13]=0;
  p_output1[14]=1.;
  p_output1[15]=0;
  p_output1[16]=0;
  p_output1[17]=0;
  p_output1[18]=var1[1];
  p_output1[19]=t13;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=1.;
  p_output1[24]=-1.*t25*var1[2];
  p_output1[25]=-1.*t35*var1[2];
  p_output1[26]=t25*var1[0] + t35*var1[1];
  p_output1[27]=-1.*t35;
  p_output1[28]=t25;
  p_output1[29]=0;
  p_output1[30]=-1.*t108*var1[1] - 1.*t106*t35*var1[2];
  p_output1[31]=t108*var1[0] + t106*t25*var1[2];
  p_output1[32]=t106*t35*var1[0] - 1.*t106*t25*var1[1];
  p_output1[33]=t106*t25;
  p_output1[34]=t106*t35;
  p_output1[35]=-1.*t108;
  p_output1[36]=0.091*t194 + t108*var1[1] + t106*t35*var1[2];
  p_output1[37]=0.091*t210 - 1.*t108*var1[0] - 1.*t106*t25*var1[2];
  p_output1[38]=0.091*t106*t174 - 1.*t106*t35*var1[0] + t106*t25*var1[1];
  p_output1[39]=0. - 1.*t106*t25;
  p_output1[40]=0. - 1.*t106*t35;
  p_output1[41]=0. + t108;
  p_output1[42]=-0.084668*t106*t25 - 0.041869*t316 - 0.016493*t322 + 0.366501*(t385*t395 + t406*t414) - 0.930418*(t385*t441 + t406*t458);
  p_output1[43]=-0.084668*t106*t35 - 0.041869*t414 - 0.016493*t458 + 0.366501*(t395*t498 + t316*t514) - 0.930418*(t441*t498 + t322*t514);
  p_output1[44]=0.084668*t108 - 0.041869*t395 - 0.016493*t441 + 0.366501*(t414*t576 + t316*t599) - 0.930418*(t458*t576 + t322*t599);
  p_output1[45]=0. + 0.366501*t316 - 0.930418*t322;
  p_output1[46]=0. + 0.366501*t414 - 0.930418*t458;
  p_output1[47]=0. + 0.366501*t395 - 0.930418*t441;
  p_output1[48]=-0.041869*t1010 + 0.016493*t1035 + 0.366501*(t866*t880 + t904*t920) + 0.930418*(t866*t953 + t904*t969) + 0.151852*t995;
  p_output1[49]=0.151852*t1139 - 0.041869*t880 + 0.366501*(t1010*t1071 + t1086*t920) + 0.016493*t953 + 0.930418*(t1035*t1071 + t1086*t969);
  p_output1[50]=0.151852*t1247 + 0.366501*(t1010*t1180 + t1191*t880) - 0.041869*t920 + 0.930418*(t1035*t1180 + t1191*t953) + 0.016493*t969;
  p_output1[51]=0. + 0.366501*t1010 + 0.930418*t1035;
  p_output1[52]=0. + 0.366501*t880 + 0.930418*t953;
  p_output1[53]=0. + 0.366501*t920 + 0.930418*t969;
  p_output1[54]=-0.930418*(t1409*t1419 + t1439*t1448) - 0.366501*(t1409*t1481 + t1439*t1492) + 0.014606*t1527 - 0.236705*t1556 - 0.03708*t1569;
  p_output1[55]=0.014606*t1448 - 0.03708*t1492 - 0.930418*(t1419*t1590 + t1527*t1605) - 0.366501*(t1481*t1590 + t1569*t1605) - 0.236705*t1666;
  p_output1[56]=0.014606*t1419 - 0.03708*t1481 - 0.930418*(t1527*t1680 + t1448*t1699) - 0.366501*(t1569*t1680 + t1492*t1699) - 0.236705*t1747;
  p_output1[57]=0. - 0.930418*t1527 - 0.366501*t1569;
  p_output1[58]=0. - 0.930418*t1448 - 0.366501*t1492;
  p_output1[59]=0. - 0.930418*t1419 - 0.366501*t1481;
  p_output1[60]=-0.366501*(t1884*t1899 + t1911*t1926) - 0.930418*(t1884*t1943 + t1926*t1963) + 0.244523*t1983 - 0.09632*t1996 - 0.6347*t2018;
  p_output1[61]=0.244523*t1899 - 0.09632*t1943 - 0.366501*(t1983*t2038 + t1911*t2065) - 0.930418*(t1996*t2038 + t1963*t2065) - 0.6347*t2101;
  p_output1[62]=0.244523*t1911 - 0.09632*t1963 - 0.366501*(t1899*t2113 + t1983*t2139) - 0.930418*(t1943*t2113 + t1996*t2139) - 0.6347*t2199;
  p_output1[63]=0. - 0.366501*t1983 - 0.930418*t1996;
  p_output1[64]=0. - 0.366501*t1899 - 0.930418*t1943;
  p_output1[65]=0. - 0.366501*t1911 - 0.930418*t1963;
  p_output1[66]=-0.884829*t2300 - 0.057683*t2330 + 0.022722*t2377 - 0.366501*(t2402*t2446 + t2462*t2486) - 0.930418*(t2446*t2510 + t2486*t2536);
  p_output1[67]=-0.057683*t2402 + 0.022722*t2510 - 0.366501*(t2330*t2583 + t2462*t2605) - 0.930418*(t2377*t2583 + t2536*t2605) - 0.884829*t2634;
  p_output1[68]=-0.057683*t2462 + 0.022722*t2536 - 0.884829*t2669 - 0.366501*(t2402*t2689 + t2330*t2728) - 0.930418*(t2510*t2689 + t2377*t2728);
  p_output1[69]=0. - 0.366501*t2330 - 0.930418*t2377;
  p_output1[70]=0. - 0.366501*t2402 - 0.930418*t2510;
  p_output1[71]=0. - 0.366501*t2462 - 0.930418*t2536;
  p_output1[72]=-0.671277*t2823 - 0.337139*t2846 + 0.050068*t2879 - 0.218018*(t2898*t2945 + t2960*t2968) + 0.553471*(t2945*t2999 + t2968*t3011) + 0.803828*(t2945*t3022 + t2968*t3029);
  p_output1[73]=-0.671277*t2898 - 0.337139*t2999 + 0.050068*t3022 - 0.218018*(t2823*t3066 + t2960*t3079) + 0.553471*(t2846*t3066 + t3011*t3079) + 0.803828*(t2879*t3066 + t3029*t3079);
  p_output1[74]=-0.671277*t2960 - 0.337139*t3011 + 0.050068*t3029 - 0.218018*(t2898*t3127 + t2823*t3136) + 0.553471*(t2999*t3127 + t2846*t3136) + 0.803828*(t3022*t3127 + t2879*t3136);
  p_output1[75]=0. - 0.218018*t2823 + 0.553471*t2846 + 0.803828*t2879;
  p_output1[76]=0. - 0.218018*t2898 + 0.553471*t2999 + 0.803828*t3022;
  p_output1[77]=0. - 0.218018*t2960 + 0.553471*t3011 + 0.803828*t3029;
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
}



void Js_LeftToeBottomBack_src(double *p_output1, const double *var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
