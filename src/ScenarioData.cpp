//----------------------------------------------------------------------
// ScenarioData.cpp:
//   class than implement scenario data analysis and process
//
//          Copyright (C) 2020-2029 by Jun Mo, All rights reserved.
//
//----------------------------------------------------------------------

#include "ScenarioData.h"
#include "JsonInterpreter.h"
#include <memory.h>

CScenarioData::CScenarioData()
{
	memset(GpsEph, 0, sizeof(GpsEph));
	memset(BdsEph, 0, sizeof(BdsEph));
	memset(GalEph, 0, sizeof(GalEph));
	memset(GloEph, 0, sizeof(GloEph));
	memset(GpsEphVisible, 0, sizeof(GpsEphVisible));
	memset(BdsEphVisible, 0, sizeof(BdsEphVisible));
	memset(GalEphVisible, 0, sizeof(GalEphVisible));
	memset(GloEphVisible, 0, sizeof(GloEphVisible));
	memset(&OutputParam, 0, sizeof(OutputParam));
	GpsSatNumber = BdsSatNumber = GalSatNumber = GloSatNumber = 0;
}

CScenarioData::~CScenarioData()
{
}


int CScenarioData::LoadScenarioFile(const char* filename)
{
	// load scenario data from JSON file
	if (JsonTree.ReadFile(filename) < 0)
		return -1;
	return LoadScenarioObject(JsonTree.GetRootObject(), filename);
}

int CScenarioData::LoadScenarioObject(JsonObject *Object, const char* JsonFilePath)
{
	int i;
	LLA_POSITION StartPos;
	LOCAL_SPEED StartVel;

	if (Object == NULL)
		return -1;
	// clear previous data
	Trajectory.ClearTrajectoryList();
	PowerControl.Clear();
	// load scenario data from JSON file
	if (JsonFilePath != NULL)
		SetJsonFilePath(JsonFilePath);
	if (!AssignParameters(Object, &UtcTime, &StartPos, &StartVel, &Trajectory, &NavData, &OutputParam, &PowerControl, NULL))
		return -1;
	// reset trajectory time and calculate initial position and velocity
	Trajectory.ResetTrajectoryTime();
	CurPosEcef = LlaToEcef(StartPos);
	CurPosLla = StartPos;
	SpeedLocalToEcef(StartPos, StartVel, CurPosEcef);
	// convert UTC time to GPS, BDS and GLONASS time
	CurTime = UtcToGpsTime(UtcTime);
	UtcTime = GpsTimeToUtc(CurTime, FALSE);	// convert back to UTC represented GPS time (no leap second adjustment)
	PowerControl.ResetTime();

	UpdateSatList();

	// initialize ionospheric model and satellite parameters for each system
	IonoModel.SetIonoParam(NavData.GetGpsIono());
	for (i = 0; i < TOTAL_GPS_SAT; i ++)
		GpsSatParam[i].Initialize(GpsSystem, GpsEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);
	for (i = 0; i < TOTAL_BDS_SAT; i ++)
		BdsSatParam[i].Initialize(BdsSystem, BdsEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);
	for (i = 0; i < TOTAL_GAL_SAT; i ++)
		GalSatParam[i].Initialize(GalileoSystem, GalEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);
	for (i = 0; i < TOTAL_GLO_SAT; i ++)
		GloSatParam[i].Initialize(GlonassSystem, (PGPS_EPHEMERIS)GloEph[i], &IonoModel, PowerControl.InitCN0, PowerControl.Adjust);

	// calculate satellite parameters for visible satellites at start time
	UpdateSatelliteParam(0);

	return 0;
}

int CScenarioData::GetCurPosVel(KINEMATIC_INFO& CurPosVel, LLA_POSITION &CurPosLla)
{
	CurPosVel = this->CurPosEcef;
	CurPosLla = this->CurPosLla;
	return 0;
}

int CScenarioData::GetCurTime(GNSS_TIME& CurGnssTime, UTC_TIME& CurUtcTime)
{
	CurGnssTime = this->CurTime;
	CurUtcTime = this->UtcTime;
	return 0;
}

int CScenarioData::GetSatelliteParam(GnssSystem system, CSatelliteParam* SatParam[])
{
	int i, index;
	int SatNumber = 0;

	switch (system)
	{
	case GpsSystem:
		for (i = 0; i < GpsSatNumber; i ++)
		{
			index = GpsEphVisible[i]->svid - 1;
			SatParam[SatNumber++] = &GpsSatParam[index];
		}
		break;
	case BdsSystem:
		for (i = 0; i < BdsSatNumber; i ++)
		{
			index = BdsEphVisible[i]->svid - 1;
			SatParam[SatNumber++] = &BdsSatParam[index];
		}
		break;
	case GalileoSystem:
		for (i = 0; i < GalSatNumber; i ++)
		{
			index = GalEphVisible[i]->svid - 1;
			SatParam[SatNumber++] = &GalSatParam[index];
		}
		break;
	case GlonassSystem:
		for (i = 0; i < GloSatNumber; i ++)
		{
			index = GloEphVisible[i]->n - 1;
			SatParam[SatNumber++] = &GloSatParam[index];
		}
		break;
	}
	return SatNumber;
}

int CScenarioData::StepForward(BOOL bUpdateSatList, BOOL bUpdateSatParam, int TimeStepMs)
{
	if (TimeStepMs <= 0)
		TimeStepMs = OutputParam.Interval;
	if (TimeStepMs <= 0)
		return -1;
	if (!Trajectory.GetNextPosVelECEF(TimeStepMs / 1000., CurPosEcef))
		return -1;
	CurPosLla = EcefToLla(CurPosEcef);
	CurTime.MilliSeconds += TimeStepMs;
	if (CurTime.MilliSeconds > 604800000)
	{
		CurTime.Week ++;
		CurTime.MilliSeconds -= 604800000;
	}
	UtcTime = GpsTimeToUtc(CurTime, FALSE);

	if (bUpdateSatList)
		UpdateSatList();

	if (bUpdateSatParam)
		UpdateSatelliteParam(TimeStepMs);

	return TimeStepMs;
}

void CScenarioData::UpdateSatelliteParam(int TimeStepMs)
{
	int i, index;
	PSIGNAL_POWER PowerList;
	int ListCount = PowerControl.GetPowerControlList(TimeStepMs, PowerList);

	for (i = 0; i < GpsSatNumber; i ++)
	{
		index = GpsEphVisible[i]->svid - 1;
		GpsSatParam[index].CalculateParam(CurPosEcef, CurPosLla, CurTime);
		GpsSatParam[index].UpdateCN0(ListCount, PowerList);
	}
	for (i = 0; i < BdsSatNumber; i ++)
	{
		index = BdsEphVisible[i]->svid - 1;
		BdsSatParam[index].CalculateParam(CurPosEcef, CurPosLla, CurTime);
		BdsSatParam[index].UpdateCN0(ListCount, PowerList);
	}
	for (i = 0; i < GalSatNumber; i ++)
	{
		index = GalEphVisible[i]->svid - 1;
		GalSatParam[index].CalculateParam(CurPosEcef, CurPosLla, CurTime);
		GalSatParam[index].UpdateCN0(ListCount, PowerList);
	}
	for (i = 0; i < GloSatNumber; i ++)
	{
		index = GloEphVisible[i]->n - 1;
		GloSatParam[index].CalculateParam(CurPosEcef, CurPosLla, CurTime);
		GloSatParam[index].UpdateCN0(ListCount, PowerList);
	}
}

void CScenarioData::UpdateSatList()
{
	BdsTime = UtcToBdsTime(UtcTime);
	GlonassTime = UtcToGlonassTime(UtcTime);
	int i;

	// find ephemeris for all satellites in each system
	for (i = 1; i <= TOTAL_GPS_SAT; i ++)
		GpsSatParam[i-1].UpdateEphemeris(GpsEph[i-1] = NavData.FindEphemeris(GpsSystem, CurTime, i));
	for (i = 1; i <= TOTAL_BDS_SAT; i ++)
		BdsSatParam[i-1].UpdateEphemeris(BdsEph[i-1] = NavData.FindEphemeris(BdsSystem, BdsTime, i));
	for (i = 1; i <= TOTAL_GAL_SAT; i ++)
		GalSatParam[i-1].UpdateEphemeris(GalEph[i-1] = NavData.FindEphemeris(GalileoSystem, CurTime, i));
	for (i = 1; i <= TOTAL_GLO_SAT; i ++)
		GloSatParam[i-1].UpdateEphemeris((PGPS_EPHEMERIS)(GloEph[i-1] = NavData.FindGloEphemeris(GlonassTime, i)));

	// determine visible satellites for each system based on current position and time
	GpsSatNumber = (OutputParam.FreqSelect[GpsSystem]) ? GetVisibleSatellite(CurPosEcef, CurTime, OutputParam, GpsSystem, GpsEph, TOTAL_GPS_SAT, GpsEphVisible) : 0;
	BdsSatNumber = (OutputParam.FreqSelect[BdsSystem]) ? GetVisibleSatellite(CurPosEcef, CurTime, OutputParam, BdsSystem, BdsEph, TOTAL_BDS_SAT, BdsEphVisible) : 0;
	GalSatNumber = (OutputParam.FreqSelect[GalileoSystem]) ? GetVisibleSatellite(CurPosEcef, CurTime, OutputParam, GalileoSystem, GalEph, TOTAL_GAL_SAT, GalEphVisible) : 0;
	GloSatNumber = (OutputParam.FreqSelect[GlonassSystem]) ? GetGlonassVisibleSatellite(CurPosEcef, GlonassTime, OutputParam, GloEph, TOTAL_GLO_SAT, GloEphVisible) : 0;
}
