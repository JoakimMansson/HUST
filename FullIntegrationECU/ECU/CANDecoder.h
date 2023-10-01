#pragma once


void DecodeCANMsg(long __id, const char* __dta);
double extractBytesToDecimal(const char* __dta, int startByte, int numBytes);
double extractDataNrBytes(const char* __dta, int startByte, int numBytes);