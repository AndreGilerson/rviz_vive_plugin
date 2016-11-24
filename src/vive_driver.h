#ifndef VIVE_DRIVER_H
#define VIVE_DRIVER_H

#include "openvr_driver.h"
#include <iostream>
#include <cstring>

class DriverLog:public vr::IDriverLog
{
public:
	DriverLog();
	
	virtual void Log(const char* pchLogMessage);
};

class VRSettings:public vr::IVRSettings
{
public:
	VRSettings();
	
	/* Methods from vr::IVRSettings: */
	virtual const char* GetSettingsErrorNameFromEnum(vr::EVRSettingsError eError);
	virtual bool Sync(bool bForce,vr::EVRSettingsError* peError);
	virtual void SetBool(const char* pchSection,const char* pchSettingsKey,bool bValue,vr::EVRSettingsError* peError);
	virtual void SetInt32(const char* pchSection,const char* pchSettingsKey,int32_t nValue,vr::EVRSettingsError* peError);
	virtual void SetFloat(const char* pchSection,const char* pchSettingsKey,float flValue,vr::EVRSettingsError* peError);
	virtual void SetString(const char* pchSection,const char* pchSettingsKey,const char* pchValue,vr::EVRSettingsError* peError);
	virtual bool GetBool(const char* pchSection,const char* pchSettingsKey,vr::EVRSettingsError* peError);
	virtual int32_t GetInt32(const char* pchSection,const char*pchSettingsKey,vr::EVRSettingsError* peError);
	virtual float GetFloat(const char* pchSection,const char* pchSettingsKey,vr::EVRSettingsError* peError);
	virtual void GetString(const char* pchSection,const char* pchSettingsKey,char* pchValue,uint32_t unValueLen,vr::EVRSettingsError* peError);
	virtual void RemoveSection(const char* pchSection,vr::EVRSettingsError* peError);
	virtual void RemoveKeyInSection(const char* pchSection,const char* pchSettingsKey,vr::EVRSettingsError* peError);
};

#endif