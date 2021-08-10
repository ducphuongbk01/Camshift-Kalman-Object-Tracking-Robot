#include <stdio.h>
#include <math.h>
#include "fuzzyController.h"

double hlt_hinhthang(double x, double L, double c1, double c2, double R);
double fuzzyController(double ek_std, double edot_std);
double controller(double ek_fuzzy, double Ts_fuzzy);
double saturation( double in_put, double rangeBot, double rangeTop);

double ek_1_fuzzy = 0;
double output_fuzzy = 0;


double hlt_hinhthang(double x, double L, double c1, double c2, double R)
{
    if (x < L)
        return 0;
    else if (x<c1)
        return (x-L)/(c1-L);
    else if (x<c2)
        return 1;
    else if (x<R)
        return (R-x)/(R-c2);
    else
        return 0;
}

double fuzzyController(double ek_std, double edot_std)
{
    double e_NB, e_NS, e_ZE, e_PS, e_PB;
    double edot_NE, edot_ZE, edot_PO;
    double y_NB, y_NM, y_NS, y_ZE, y_PS, y_PM, y_PB;
    double y[15];
    double beta[15];
    double num = 0, den = 0;
    double u_fuzzy = 0;

    //Mo hoa e
    e_NB = hlt_hinhthang(ek_std, -3, -2, -0.66, -0.59);
    e_NS = hlt_hinhthang(ek_std, -0.66, -0.59, -0.25, -0.18);
    e_ZE = hlt_hinhthang(ek_std, -0.25, -0.18, 0.18, 0.25);
    e_PS = hlt_hinhthang(ek_std, 0.18, 0.25, 0.59, 0.66);
    e_PB = hlt_hinhthang(ek_std, 0.59, 0.66, 2, 3);

    //Mo hoa edot
    edot_NE = hlt_hinhthang(edot_std, -3, -2, -0.7, -0.5);
    edot_ZE = hlt_hinhthang(edot_std, -0.7, -0.5, 0.5, 0.7);
    edot_PO = hlt_hinhthang(edot_std,0.5, 0.7, 2, 3);


    //Khai bao bien ra
    y_NB = 0.4;
    y_NM = 0.48;
    y_NS = 0.56;
    y_ZE = 0.64;
    y_PS = 0.73;
    y_PM = 0.81;
    y_PB = 0.9;

    //Cac quy tac mo - dung menh de dieu kien
    beta[0] = e_NB * edot_NE; y[0] = y_PB;
    beta[1] = e_NB * edot_ZE; y[1] = y_PM;
    beta[2] = e_NB * edot_PO; y[2] = y_PS;

    beta[3] = e_NS * edot_NE; y[3] = y_PM;
    beta[4] = e_NS * edot_ZE; y[4] = y_PS;
    beta[5] = e_NS * edot_PO; y[5] = y_ZE;

    beta[6] = e_ZE * edot_NE; y[6] = y_PS;
    beta[7] = e_ZE * edot_ZE; y[7] = y_ZE;
    beta[8] = e_ZE * edot_PO; y[8] = y_NS;

    beta[9] = e_PS * edot_NE; y[9] = y_ZE;
    beta[10] = e_PS * edot_ZE; y[10] = y_NS;
    beta[11] = e_PS * edot_PO; y[11] = y_NM;

    beta[12] = e_PB * edot_NE; y[12] = y_NS;
    beta[13] = e_PB * edot_ZE; y[13] = y_NM;
    beta[14] = e_PB * edot_PO; y[14] = y_NB;

    //Giai mo dung phuong phap trung binh co trong so
    for (int i = 0; i<15; i++)
        {
            //printf("%f  %f \n", beta[i], y[i]);
            num += beta[i]*y[i];
            den += beta[i];
        }
    u_fuzzy = num/den;
    return u_fuzzy;
}

double controller(double ek_fuzzy, double Ts_fuzzy)
{
    double edot;
		double u_fuzzy;
    edot = (ek_fuzzy - ek_1_fuzzy)/Ts_fuzzy;
	  ek_1_fuzzy = ek_fuzzy;

    //Standardize ek_fuzzy and edot to [-1;1]
    ek_fuzzy = saturation(ek_fuzzy/640,-1,1);
    edot = saturation(edot/100,-1,1);

    //Fuzzy Controller
    u_fuzzy = fuzzyController(ek_fuzzy, edot);
    output_fuzzy = u_fuzzy*180;
    output_fuzzy = saturation(output_fuzzy,80,150);
    return output_fuzzy;
}


double saturation( double in_put, double rangeBot, double rangeTop)
{
    if (in_put>rangeTop)
        in_put=rangeTop;
    else if (in_put<rangeBot)
        in_put=rangeBot;
    else
        in_put=in_put;
    return in_put;
}
