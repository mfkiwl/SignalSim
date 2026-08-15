//----------------------------------------------------------------------
// NavBit.h:
//   Declaration of navigation bit synthesis base class
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#ifndef __NAV_BIT_H__
#define __NAV_BIT_H__

#include "BasicTypes.h"

#define COMPOSE_BITS(data, start, width) (((data) & ((1UL << (width)) - 1)) << (start))

typedef union
{
	double d_data;
	unsigned int i_data[2];
} DOUBLE_INT_UNION;

class NavBit
{
public:
	enum NavParamType { ParamTypeSTO, ParamTypeEOP, ParamTypeIonKModel, ParamTypeIonGModel, ParamTypeIonBModel };

	NavBit();
	~NavBit();

	virtual int GetFrameData(GNSS_TIME StartTime, int svid, int Param, int *NavBits) = 0;	// Param reserved for same Navigation bit structure in different signal
	virtual int SetEphemeris(int svid, PGPS_EPHEMERIS Eph) = 0;
	virtual int SetAlmanac(GPS_ALMANAC Alm[]) = 0;
	virtual int SetIonoUtc(PIONO_PARAM IonoParam, PUTC_PARAM UtcParam) = 0;
	virtual int SetNavParam(NavParamType ParamType, void *Param) { return 0; }
	int roundi(double data);
	int roundu(double data);
	double UnscaleDouble(double value, int scale);
	int UnscaleInt(double value, int scale);
	unsigned int UnscaleUint(double value, int scale);
	long long int UnscaleLong(double value, int scale);
	unsigned long long int UnscaleULong(double value, int scale);
	int AssignBits(unsigned int Data, int BitNumber, int BitStream[]);
	unsigned char ConvolutionEncode(unsigned char EncodeBits);
	unsigned int Crc24qEncode(unsigned int *BitStream, int Length);
	GPS_EPHEMERIS AlignToe300s(PGPS_EPHEMERIS Eph);

	static const unsigned char ConvEncodeTable[256];
	static const unsigned int Crc24q[256];
};

#define NAVBIT_TYPES \
	X(LNAV,    0) \
	X(CNAV,    1) \
	X(CNAV2,   2) \
	X(GNAV,    3) \
	X(GNAV2,   4) \
	X(D1D2,    5) \
	X(BCNAV1,  6) \
	X(BCNAV2,  7) \
	X(BCNAV3,  8) \
	X(INAV,    9) \
	X(FNAV,   10) \
	X(ECNAV,  11) \
	X(SBAS,   12)

#define MAX_NAVBIT_TYPE 13

enum
{
#define X(name, val) NAVBIT_TYPE_##name = val,
	NAVBIT_TYPES
#undef X
};

enum
{
#define X(name, val) NAVBIT_MASK_##name = (1U << NAVBIT_TYPE_##name),
	NAVBIT_TYPES
#undef X
};

int CreateNavBitArray(unsigned int TypeMask);
int ReleaseNavBitArray();
int NavBitSetEphemeris(int NavBitType, int svid, PGPS_EPHEMERIS pEph);
int NavBitSetAlmanac(int NavBitType, GPS_ALMANAC Alm[]);
int NavBitSetIonoUtc(int NavBitType, PIONO_PARAM IonoParam, PUTC_PARAM UtcParam);
NavBit* GetNavBit(GnssSystem SatSystem, int SatSignalIndex);

#endif // __NAV_BIT_H__
