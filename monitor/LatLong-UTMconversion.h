// Taken from http://ereimer.net/programs/ !
// LatLong-UTM.c++
// Conversions:  LatLong to UTM;  and UTM to LatLong;
// by Eugene Reimer, ereimer@shaw.ca, 2002-December;

#include <cmath>			//2010-08-11: was <math.h>
#include <cstdio>			//2010-08-11: was <stdio.h>
#include <cstdlib>			//2010-08-11: was <stdlib.h>
#include <cstring>			//2010-08-11: was <string.h>
#include <cctype>			//2010-08-11: was <ctype.h>
#include <iostream>			//2010-08-11: was <iostream.h>
#include <iomanip>			//2010-08-11: was <iomanip.h>
using namespace std;			//2010-08-11: added

const double deg2rad  =	(2 * M_PI) / 360;
const double rad2deg  =	180/M_PI;
const double k0       =	0.9996;

class Ellipsoid{
public:
	Ellipsoid(){};
	Ellipsoid(int id, char* name, double radius, double fr){
		Name=name;  EquatorialRadius=radius;  eccSquared=2/fr-1/(fr*fr);
	}
	char* Name;
	double EquatorialRadius;
	double eccSquared;
};
static Ellipsoid ellip[] = {		//converted from PeterDana website, by Eugene Reimer 2002dec
//		 eId,  Name,		   EquatorialRadius,    1/flattening;
	Ellipsoid( 0, "Airy1830",		6377563.396,	299.3249646),
	Ellipsoid( 1, "AiryModified",		6377340.189,	299.3249646),
	Ellipsoid( 2, "AustralianNational",	6378160,	298.25),
	Ellipsoid( 3, "Bessel1841Namibia",	6377483.865,	299.1528128),
	Ellipsoid( 4, "Bessel1841",		6377397.155,	299.1528128),
	Ellipsoid( 5, "Clarke1866",		6378206.4,	294.9786982),
	Ellipsoid( 6, "Clarke1880",		6378249.145,	293.465),
	Ellipsoid( 7, "EverestIndia1830",	6377276.345,	300.8017),
	Ellipsoid( 8, "EverestSabahSarawak",	6377298.556,	300.8017),
	Ellipsoid( 9, "EverestIndia1956",	6377301.243,	300.8017),
	Ellipsoid(10, "EverestMalaysia1969",	6377295.664,	300.8017),	//Dana has no datum that uses this ellipsoid!
	Ellipsoid(11, "EverestMalay_Sing",	6377304.063,	300.8017),
	Ellipsoid(12, "EverestPakistan",	6377309.613,	300.8017),
	Ellipsoid(13, "Fischer1960Modified",	6378155,	298.3),
	Ellipsoid(14, "Helmert1906",		6378200,	298.3),
	Ellipsoid(15, "Hough1960",		6378270,	297),
	Ellipsoid(16, "Indonesian1974",		6378160,	298.247),
	Ellipsoid(17, "International1924",	6378388,	297),
	Ellipsoid(18, "Krassovsky1940",		6378245,	298.3),
	Ellipsoid(19, "GRS80",			6378137,	298.257222101),
	Ellipsoid(20, "SouthAmerican1969",	6378160,	298.25),
	Ellipsoid(21, "WGS72",			6378135,	298.26),
	Ellipsoid(22, "WGS84",			6378137,	298.257223563)
};
#define	eClarke1866	5		//names for ellipsoidId's
#define	eGRS80		19
#define	eWGS72		21
#define	eWGS84		22



void LLtoUTM(int eId, double Lat, double Long,  double& Northing, double& Easting, int& Zone){
   // converts LatLong to UTM coords;  3/22/95: by ChuckGantz chuck.gantz@globalstar.com, from USGS Bulletin 1532.
   // Lat and Long are in degrees;  North latitudes and East Longitudes are positive.
   double a = ellip[eId].EquatorialRadius;
   double ee= ellip[eId].eccSquared;
   Long -= int((Long+180)/360)*360;			//ensure longitude within -180.00..179.9
   double N, T, C, A, M;
   double LatRad = Lat*deg2rad;
   double LongRad = Long*deg2rad;

   Zone = int((Long + 186)/6);
   if( Lat >= 56.0 && Lat < 64.0 && Long >= 3.0 && Long < 12.0 )  Zone = 32;
   if( Lat >= 72.0 && Lat < 84.0 ){			//Special zones for Svalbard
      if(      Long >= 0.0  && Long <  9.0 )  Zone = 31;
      else if( Long >= 9.0  && Long < 21.0 )  Zone = 33;
      else if( Long >= 21.0 && Long < 33.0 )  Zone = 35;
      else if( Long >= 33.0 && Long < 42.0 )  Zone = 37;
   }
   double LongOrigin = Zone*6 - 183;			//origin in middle of zone
   double LongOriginRad = LongOrigin * deg2rad;

   double EE = ee/(1-ee);

   N = a/sqrt(1-ee*sin(LatRad)*sin(LatRad));
   T = tan(LatRad)*tan(LatRad);
   C = EE*cos(LatRad)*cos(LatRad);
   A = cos(LatRad)*(LongRad-LongOriginRad);

   M= a*((1 - ee/4    - 3*ee*ee/64 - 5*ee*ee*ee/256  ) *LatRad
	    - (3*ee/8 + 3*ee*ee/32 + 45*ee*ee*ee/1024) *sin(2*LatRad)
	    + (15*ee*ee/256 + 45*ee*ee*ee/1024	  ) *sin(4*LatRad)
	    - (35*ee*ee*ee/3072			  ) *sin(6*LatRad));

   Easting = k0*N*(A+(1-T+C)*A*A*A/6+(5-18*T+T*T+72*C-58*EE)*A*A*A*A*A/120) + 500000.0;

   Northing = k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
			    + (61-58*T+T*T+600*C-330*EE)*A*A*A*A*A*A/720));
}

/*
void UTMtoLL(int eId, double Northing, double Easting, int Zone,  double& Lat, double& Long){
   // converts UTM coords to LatLong;  3/22/95: by ChuckGantz chuck.gantz@globalstar.com, from USGS Bulletin 1532.
   // Lat and Long are in degrees;  North latitudes and East Longitudes are positive.
   double a = ellip[eId].EquatorialRadius;
   double ee = ellip[eId].eccSquared;
   double EE = ee/(1-ee);
   double e1 = (1-sqrt(1-ee))/(1+sqrt(1-ee));
   double N1, T1, C1, R1, D, M, mu, phi1Rad;
   double x = Easting - 500000.0;			//remove 500,000 meter offset for longitude
   double y = Northing;
   double LongOrigin = Zone*6 - 183;			//origin in middle of zone

   M = y / k0;
   mu = M/(a*(1-ee/4-3*ee*ee/64-5*ee*ee*ee/256));

   phi1Rad = mu + (3*e1/2-27*e1*e1*e1/32) *sin(2*mu)
		+ (21*e1*e1/16-55*e1*e1*e1*e1/32) *sin(4*mu)
		+ (151*e1*e1*e1/96) *sin(6*mu);
   N1 = a/sqrt(1-ee*sin(phi1Rad)*sin(phi1Rad));
   T1 = tan(phi1Rad)*tan(phi1Rad);
   C1 = EE*cos(phi1Rad)*cos(phi1Rad);
   R1 = a*(1-ee)/pow(1-ee*sin(phi1Rad)*sin(phi1Rad), 1.5);
   D = x/(N1*k0);

   Lat = phi1Rad - (N1*tan(phi1Rad)/R1)*(D*D/2-(5+3*T1+10*C1-4*C1*C1-9*EE)*D*D*D*D/24
		   +(61+90*T1+298*C1+45*T1*T1-252*EE-3*C1*C1)*D*D*D*D*D*D/720);
   Lat *= rad2deg;
   Long = (D-(1+2*T1+C1)*D*D*D/6+(5-2*C1+28*T1-3*C1*C1+8*EE+24*T1*T1)*D*D*D*D*D/120) / cos(phi1Rad);
   Long = LongOrigin + Long*rad2deg;
}*/
