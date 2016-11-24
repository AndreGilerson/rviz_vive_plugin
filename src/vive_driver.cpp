#include "vive_driver.h"

DriverLog::DriverLog()
{
	
}

void DriverLog::Log(const char* pchLogMessage)
{
	std::cout<<"DriverLog:"<<pchLogMessage;
}






VRSettings::VRSettings()
{
}

const char* VRSettings::GetSettingsErrorNameFromEnum(vr::EVRSettingsError eError)
{
	return "Unknown";
}

bool VRSettings::Sync(bool bForce,vr::EVRSettingsError* peError)
{
	std::cout<<"Sync with "<<bForce<<std::endl;
	if(peError!=0)
		*peError=vr::VRSettingsError_None;
	return false;
}

void VRSettings::SetBool(const char* pchSection,const char* pchSettingsKey,bool bValue,vr::EVRSettingsError* peError)
{
	std::cout<<"SetBool for "<<pchSection<<"/"<<pchSettingsKey<<" with value "<<bValue<<std::endl;
	if(peError!=0)
		*peError=vr::VRSettingsError_None;
}

void VRSettings::SetInt32(const char* pchSection,const char* pchSettingsKey,int32_t nValue,vr::EVRSettingsError* peError)
{
	std::cout<<"SetInt32 for "<<pchSection<<"/"<<pchSettingsKey<<" with value "<<nValue<<std::endl;
	if(peError!=0)
		*peError=vr::VRSettingsError_None;
}

void VRSettings::SetFloat(const char* pchSection,const char* pchSettingsKey,float flValue,vr::EVRSettingsError* peError)
{
	std::cout<<"SetFloat for "<<pchSection<<"/"<<pchSettingsKey<<" with value "<<flValue<<std::endl;
	if(peError!=0)
		*peError=vr::VRSettingsError_None;
}

void VRSettings::SetString(const char* pchSection,const char* pchSettingsKey,const char* pchValue,vr::EVRSettingsError* peError)
{
	std::cout<<"SetString for "<<pchSection<<"/"<<pchSettingsKey<<" with value "<<pchValue<<std::endl;
	if(peError!=0)
		*peError=vr::VRSettingsError_None;
}

bool VRSettings::GetBool(const char* pchSection,const char* pchSettingsKey,vr::EVRSettingsError* peError)
{
	std::cout<<"GetBool for "<<pchSection<<"/"<<pchSettingsKey<<std::endl;
	if(strcmp(pchSettingsKey, "dbhistory")) return false;
	else if(strcmp(pchSettingsKey, "disableimu")) return false;
	else if(strcmp(pchSettingsKey, "fakeDigitalTrigger")) return false;
	else if(strcmp(pchSettingsKey, "powerOffOnExit")) return true;
	else if(strcmp(pchSettingsKey, "trackedCamera")) return false;
	else if(strcmp(pchSettingsKey, "cameraEdgeEnhancement")) return true;
	else if(strcmp(pchSettingsKey, "enableCamera")) return true;
	else if(strcmp(pchSettingsKey, "fakeHtcHmdMainboard")) return false;
	return 0;
}

int32_t VRSettings::GetInt32(const char* pchSection,const char*pchSettingsKey,vr::EVRSettingsError* peError)
{
	std::cout<<"GetInt32 for "<<pchSection<<"/"<<pchSettingsKey<<std::endl;
	if(strcmp(pchSettingsKey, "disambiguationdebug")) return 0;
	if(strcmp(pchSettingsKey, "primarybasestation")) return 0;
	if(strcmp(pchSettingsKey, "cameraFrameRate")) return 30;
	if(strcmp(pchSettingsKey, "cameraSensorFrameRate")) return 30;
	if(strcmp(pchSettingsKey, "cameraISPSyncDivisor")) return 1;
	else if(strcmp(pchSettingsKey, "deactivateStandbyOverride")) return 1;
	return 0;
}

float VRSettings::GetFloat(const char* pchSection,const char* pchSettingsKey,vr::EVRSettingsError* peError)
{
	std::cout<<"GetFloat for "<<pchSection<<"/"<<pchSettingsKey<<std::endl;
	if(strcmp(pchSettingsKey, "cameraFrameCaptureOffsetTime")) return 0.0;
	return 0;
}

void VRSettings::GetString(const char* pchSection,const char* pchSettingsKey,char* pchValue,uint32_t unValueLen,vr::EVRSettingsError* peError)
{
	std::cout<<"GetString for "<<pchSection<<"/"<<pchSettingsKey<<std::endl;
	if(peError!=0)
		*peError=vr::VRSettingsError_None;
	pchValue[0] = 't';
	pchValue[1] = 'd';
	pchValue[2] = 'm';
	pchValue[3] = '\0';
	unValueLen = 3;
}

void VRSettings::RemoveSection(const char* pchSection,vr::EVRSettingsError* peError)
{
	std::cout<<"RemoveSection for section "<<pchSection<<std::endl;
	if(peError!=0)
		*peError=vr::VRSettingsError_None;
}

void VRSettings::RemoveKeyInSection(const char* pchSection,const char* pchSettingsKey,vr::EVRSettingsError* peError)
{
	std::cout<<"RemoveKeyInSection for "<<pchSection<<"/"<<pchSettingsKey<<std::endl;
	if(peError!=0)
		*peError=vr::VRSettingsError_None;
}