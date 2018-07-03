#include <Windows.h>
#include <SetupAPI.h>
#include <Cfgmgr32.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "hidapi.h"

#pragma comment(lib, "SetupAPI.lib")

enum {
    WATCHDOG_UNKNOWN = 0,
    WATCHDOG_SERIAL,
    WATCHDOG_HID,
};

int active = 1;
int watchdog_type = WATCHDOG_UNKNOWN;
int serial_port = -1;
HANDLE serial_handle = INVALID_HANDLE_VALUE;
hid_device *hid_handle = NULL;
int hid_leading_zero = 1;

int serial_read(unsigned char *buf, int size);
int serial_write(unsigned char *buf, int size);

void watchdog_close(void);

void serial_scan(void)
{
    int exclude = serial_port;
    serial_port = -1;

    // ports class guid
    GUID classGuids = { 0x4D36E978L, 0xE325, 0x11CE, 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 };

    HDEVINFO hDevInfo = SetupDiGetClassDevs((LPGUID)&classGuids, NULL, NULL, 0);
    if (hDevInfo)
    {
        SP_DEVINFO_DATA SpDevInfo = { sizeof(SP_DEVINFO_DATA) };
        for (DWORD iDevIndex = 0; SetupDiEnumDeviceInfo(hDevInfo, iDevIndex, &SpDevInfo); iDevIndex++)
        {
            char szName[512] = { 0 };
            if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &SpDevInfo, SPDRP_FRIENDLYNAME,
                NULL, (PBYTE)szName, sizeof(szName), NULL))
            {
                if (strncmp("USB-SERIAL CH340(COM", szName, 20) == 0)
                {
                    int port = atoi(szName + 20);
                    if (exclude == -1)
                    {
                        serial_port = port;
                        break;
                    }
                    else if (exclude == port)
                        exclude = -1;
                }
            }
        }

        SetupDiDestroyDeviceInfoList(hDevInfo);
    }
}

int serial_connect(void)
{
    char port_name[20];
    snprintf(port_name, sizeof(port_name), "\\\\.\\COM%d", serial_port);
    serial_handle = CreateFileA(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (serial_handle == INVALID_HANDLE_VALUE)
        return -1;

    DCB dcbSerialParameters = { 0 };
    if (!GetCommState(serial_handle, &dcbSerialParameters))
        return -2;

    dcbSerialParameters.BaudRate = CBR_9600;
    dcbSerialParameters.ByteSize = 8;
    dcbSerialParameters.StopBits = ONESTOPBIT;
    dcbSerialParameters.Parity = NOPARITY;
    dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

    if (!SetCommState(serial_handle, &dcbSerialParameters))
        return -3;

    PurgeComm(serial_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    return 0;
}

int serial_read(unsigned char *buf, int size)
{
    if (serial_handle == INVALID_HANDLE_VALUE)
        return 0;

    DWORD bytesRead;
    if (ReadFile(serial_handle, buf, size, &bytesRead, NULL))
        return bytesRead;

    return 0;
}

int serial_write(unsigned char *buf, int size)
{
    if (serial_handle == INVALID_HANDLE_VALUE)
        return 0;

    DWORD bytesSend;

    if (WriteFile(serial_handle, buf, size, &bytesSend, 0))
        return bytesSend;

    return 0;
}

void serial_close(void)
{
    if (serial_handle == INVALID_HANDLE_VALUE)
        return;

    CloseHandle(serial_handle);
    serial_handle = INVALID_HANDLE_VALUE;
}

int hidwdg_connect(void)
{
    static int init = 0;
    if (!init)
    {
        // Initialize the hidapi library
        if (hid_init() != 0)
            return -1;
        init = 1;
    }

    // Open the device using the VID, PID,
    // and optionally the Serial number.
    hid_handle = hid_open(0x5131, 0x2007, NULL);
    if (hid_handle != NULL)
    {
        watchdog_type = WATCHDOG_HID;
        return 0;
    }

    return -1;
}

void hidwdg_close(void)
{
    if (hid_handle == NULL)
        return;

    hid_close(hid_handle);
    hid_handle = NULL;
}

int watchdog_find(void)
{
    if (watchdog_type != WATCHDOG_UNKNOWN)
        watchdog_close();

    watchdog_type = WATCHDOG_UNKNOWN;

    if (hidwdg_connect() == 0)
    {
        unsigned char buf[65] = { 0, };
        buf[1] = 0x80;
        buf[2] = 0;
        hid_write(hid_handle, buf, 3);
        hid_read(hid_handle, buf, 2);
        if (buf[0] == 0x82)
        {
            hid_leading_zero = 1;
            return 0;
        }

        buf[0] = 0x80;
        buf[1] = 0;
        hid_write(hid_handle, buf, 2);
        hid_read(hid_handle, buf, 2);
        if (buf[0] == 0x82)
        {
            hid_leading_zero = 0;
            return 0;
        }

        hidwdg_close();
        watchdog_type = WATCHDOG_UNKNOWN;
    }

    serial_port = -1;
    while (1)
    {
        if (serial_handle != INVALID_HANDLE_VALUE)
            CloseHandle(serial_handle);
        serial_handle = INVALID_HANDLE_VALUE;

        serial_scan();
        if (serial_port <= 0)
            return -1;

        if (serial_connect() < 0)
            continue;

        unsigned char buf[3] = { 0x80, };
        if (serial_write(buf, 1) != 1)
            continue;
        if (serial_read(buf, 3) != 3)
            continue;
        if (buf[0] != 0x81)
            continue;

        break;
    }

    watchdog_type = WATCHDOG_SERIAL;
    return 0;
}

int watchdog_connect(void)
{
    if (watchdog_type == WATCHDOG_HID)
        return hidwdg_connect();
    else if (watchdog_type == WATCHDOG_SERIAL)
        return serial_connect();
    else
        return -1;
}

void watchdog_close(void)
{
    if (watchdog_type == WATCHDOG_HID)
        hidwdg_close();
    else if (watchdog_type == WATCHDOG_SERIAL)
        serial_close();
}

void watchdog_exit(void)
{
    watchdog_close();
    hid_exit();
    watchdog_type = WATCHDOG_UNKNOWN;
}

void watchdog_signal(int seconds)
{
    if (watchdog_type == WATCHDOG_HID)
    {
        unsigned char buf[65] = {0, };
        if (seconds > 1270)
            seconds = 1270;
        buf[hid_leading_zero] = seconds / 10;
        buf[hid_leading_zero + 1] = 0;
        hid_write(hid_handle, buf, 2 + hid_leading_zero);
        hid_read(hid_handle, buf, 2);
    }
    else if (watchdog_type == WATCHDOG_SERIAL)
    {
        unsigned char buf[1];
        if (seconds > 1270)
            seconds = 1270;
        buf[0] = seconds / 10;
        serial_write(buf, 1);
    }
}

BOOL CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType)
    {
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        active = 0;
        return TRUE;

    default:
        return FALSE;
    }
}

int main(void)
{
    SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE);

    int t;
    while (active)
    {
        watchdog_find();
        watchdog_close();

        if (watchdog_type == WATCHDOG_UNKNOWN)
        {
            printf("watchdog device not found\n");
            for (t = 5; active && t; t--)
                Sleep(1000);
            continue;
        }

        while (active)
        {
            if (watchdog_connect() != 0)
                break;

            watchdog_signal(180);
            watchdog_close();

            char date[32];
            time_t now = time(NULL);
            struct tm local;
            localtime_s(&local, &now);
            strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &local);
            printf("[%s] watchdog signal\n", date);

            for (t = 60; active && t; t--)
                Sleep(1000);
        }
    }

    watchdog_exit();

    return 0;
}