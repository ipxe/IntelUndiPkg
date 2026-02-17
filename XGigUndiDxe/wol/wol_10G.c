/**************************************************************************

Copyright (c) 2014 - 2024, Intel Corporation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/
#include <wol.h>

typedef enum {
    LKV_WOL_DISABLED = 0x01,
    LKV_WOL_SUPPORTED_PORT_0_ENABLED_PORT_0 = 0x02,
    LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_1 = 0x03,
    LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_BOTH_PORTS = 0x04,
    LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_0 = 0x05,
    LKV_WOL_SUPPORTED_BOTH_PORTS_DISABLED_BOTH_PORTS = 0x06,
    LKV_WOL_SUPPORTED_PORT_0_DISABLED_PORT_0 = 0x07
} LKV_WOL_Status;
#define LKV_WOL_FEATURE_ID                  0x28

WOL_STATUS
_WolGetOffsetBitmask_IXGBE (
  IN    WOL_ADAPTER_HANDLE_TYPE     Handle,
  OUT   UINT16                      *Offset,
  OUT   UINT16                      *Bitmask
  )
{
  UINT16  LanPort = 0;
  LanPort = _WolGetLanPort(Handle);

  switch (LanPort)
  {
    case 0:
      /* EEPROM Control Word 3 */
      *Offset = 0x0038;
      /* APM Enable Port 0     */
      *Bitmask = 0x0001;
      return WOL_SUCCESS;
    case 1:
      /* EEPROM Control Word 3 */
      *Offset = 0x0038;
      /* APM Enable Port 1     */
      *Bitmask = 0x0002;
      return WOL_SUCCESS;
  }
  return WOL_FEATURE_NOT_SUPPORTED;
}

BOOLEAN _WolGetInfoFromEeprom_10G(WOL_ADAPTER_HANDLE_TYPE Handle)
{
  UINT16 Caps;
  if (WOL_SUCCESS == _WolEepromRead16(Handle, 0x2C, &Caps)) {
    switch (Caps & 0x000C) {      /* Get WOL bits                     */
      case 0x0004:                /* WOL supported on both ports      */
        return TRUE;
      case 0x0008:                /* WOL supported on the first port  */
        return _WolGetFunction(Handle) == 0;
    }
  }
  return FALSE;
}

WOL_STATUS WolGetWakeOnLanStatus_Lkv (WOL_ADAPTER_HANDLE_TYPE Handle, BOOLEAN *WolStatus)
{
  UINT8                              FunctionNumber;
  UINT16                             FeatureId;
  UINT16                             ElemCount;
  WOL_STATUS                         Status;
  struct ixgbe_aci_cmd_nvm_cfg_data  ConfigData = {0};

  FunctionNumber   = _WolGetFunction (Handle);
  FeatureId        = LKV_WOL_FEATURE_ID;
  ElemCount        = 1;

#ifndef WOL_HAF
  DEBUGPRINT (WOL, ("WolGetWakeOnLanStatus_Lkv() entry\n"));
  DEBUGPRINT (WOL, ("FunctionNumber = %d\n", FunctionNumber));
  DEBUGPRINT (WOL, ("FeatureId = 0x%X\n", FeatureId));
#endif /* !WOL_HAF */

  Status = _WolReadNvmFeatureConfig_Lkv (Handle, FeatureId, (UINT8 *)&ConfigData, sizeof(ConfigData), &ElemCount);
  if (Status != WOL_SUCCESS) {
    return Status;
  }

  switch (ConfigData.field_value) {
    case LKV_WOL_DISABLED:
    case LKV_WOL_SUPPORTED_PORT_0_DISABLED_PORT_0:
    case LKV_WOL_SUPPORTED_BOTH_PORTS_DISABLED_BOTH_PORTS:
      *WolStatus = FALSE;
      break;
    case LKV_WOL_SUPPORTED_PORT_0_ENABLED_PORT_0:
    case LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_0:
      *WolStatus = (FunctionNumber == 0);
      break;
    case LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_1:
      *WolStatus = (FunctionNumber == 1);
      break;
    case LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_BOTH_PORTS:
      *WolStatus = TRUE;
      break;
    default:
#ifndef WOL_HAF
      DEBUGPRINT (WOL, ("WOL configuration is incorrect = 0x%X\n", ConfigData.field_value));
#endif /* !WOL_HAF */
      return WOL_FEATURE_NOT_SUPPORTED;
  }

#ifndef WOL_HAF
  DEBUGPRINT (WOL, ("*WolStatus: %d\n", *WolStatus));
#endif /* !WOL_HAF */

  return WOL_SUCCESS;
}

WOL_STATUS WolEnableWakeOnLan_Lkv (WOL_ADAPTER_HANDLE_TYPE Handle, BOOLEAN Enable)
{
  WOL_STATUS                        Status;
  struct ixgbe_aci_cmd_nvm_cfg_data ReadData  = {0};
  struct ixgbe_aci_cmd_nvm_cfg_data WriteData = {0};
  UINT8                             FunctionNumber;
  UINT16                            FeatureId;
  UINT16                            ElemCount;

  FunctionNumber   = _WolGetFunction (Handle);
  FeatureId        = LKV_WOL_FEATURE_ID;
  ElemCount        = 1;

  if (FunctionNumber > 1) {
#ifndef WOL_HAF
    DEBUGPRINT (WOL, ("FunctionNumber for E610 different than 0 and 1: %d\n", FunctionNumber));
#endif /* !WOL_HAF */
    return WOL_FEATURE_NOT_SUPPORTED;
  }

  Status = _WolReadNvmFeatureConfig_Lkv (Handle, FeatureId, (UINT8 *)&ReadData, sizeof(ReadData), &ElemCount);
  if (Status != WOL_SUCCESS) {
#ifndef WOL_HAF
    DEBUGPRINT (WOL, ("_WolReadNvmFeatureConfig_Lkv failed with: %d\n", Status));
#endif
    return Status;
  }

  WriteData.field_id = FeatureId;

  if (FunctionNumber == 0) {
    switch (ReadData.field_value) {
      case LKV_WOL_DISABLED:
        return WOL_FEATURE_NOT_SUPPORTED;
      case LKV_WOL_SUPPORTED_PORT_0_ENABLED_PORT_0:
      case LKV_WOL_SUPPORTED_PORT_0_DISABLED_PORT_0:
        WriteData.field_value = Enable ? LKV_WOL_SUPPORTED_PORT_0_ENABLED_PORT_0 : LKV_WOL_SUPPORTED_PORT_0_DISABLED_PORT_0;
        break;
      case LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_1:
      case LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_BOTH_PORTS:
        WriteData.field_value = Enable ? LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_BOTH_PORTS : LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_1;
        break;
      case LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_0:
      case LKV_WOL_SUPPORTED_BOTH_PORTS_DISABLED_BOTH_PORTS:
        WriteData.field_value = Enable ? LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_0 : LKV_WOL_SUPPORTED_BOTH_PORTS_DISABLED_BOTH_PORTS;
        break;
      default:
#ifndef WOL_HAF
        DEBUGPRINT (WOL, ("WOL configuration (READ) is incorrect = 0x%X\n", ReadData.field_value));
#endif /* !WOL_HAF */
        return WOL_FEATURE_NOT_SUPPORTED;
    }
  } else { // (FunctionNumber == 1)
    switch (ReadData.field_value) {
      case LKV_WOL_DISABLED:
      case LKV_WOL_SUPPORTED_PORT_0_ENABLED_PORT_0:
      case LKV_WOL_SUPPORTED_PORT_0_DISABLED_PORT_0:
        return WOL_FEATURE_NOT_SUPPORTED;
        break;
      case LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_1:
      case LKV_WOL_SUPPORTED_BOTH_PORTS_DISABLED_BOTH_PORTS:
        WriteData.field_value = Enable ? LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_1 : LKV_WOL_SUPPORTED_BOTH_PORTS_DISABLED_BOTH_PORTS;
        break;
      case LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_BOTH_PORTS:
      case LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_0:
        WriteData.field_value = Enable ? LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_BOTH_PORTS : LKV_WOL_SUPPORTED_BOTH_PORTS_ENABLED_PORT_0;
        break;
      default:
#ifndef WOL_HAF
        DEBUGPRINT (WOL, ("WOL configuration (READ) is incorrect = 0x%X\n", ReadData.field_value));
#endif /* !WOL_HAF */
        return WOL_FEATURE_NOT_SUPPORTED;
    }
  }

#ifndef WOL_HAF
    DEBUGPRINT (WOL, ("WriteData.field_value: 0x%X\n", WriteData.field_value));
#endif /* !WOL_HAF */

  if (ReadData.field_value == WriteData.field_value) {
#ifndef WOL_HAF
    DEBUGPRINT (WOL, ("ReadData.field_value == WriteData.field_value\n"));
#endif /* !WOL_HAF */
    return WOL_SUCCESS;
  }

  Status = _WolWriteNvmFeatureConfig_Lkv (Handle, (UINT8 *)&WriteData, sizeof(WriteData), ElemCount);
  if (Status != WOL_SUCCESS)
  {
#ifndef WOL_HAF
    DEBUGPRINT (WOL, ("_WolWriteNvmFeatureConfig_Lkv failed with: %d\n", Status));
#endif /* !WOL_HAF */
  }

  return _WolEepromUpdateChecksum (Handle);
}

WOL_STATUS WolSetApmRegister_10G (WOL_ADAPTER_HANDLE_TYPE Handle, BOOLEAN Enable)
{
  UINT32        Address = 0;
  UINT32        ReadWord = 0;
  BOOLEAN       IsCurrentlySet;

  if (Handle == NULL) {
    return WOL_FEATURE_NOT_SUPPORTED;
  }

  if (_WolGetMacType (Handle) < WOL_MAKE_MACTYPE (WOL_10G, ixgbe_mac_X540)) {
    Address = IXGBE_GRC;
  } else {
#ifndef WOL_HAF
    Address = IXGBE_GRC_BY_MAC (&Handle->NicInfo.Hw);
#else /* WOL_HAF */
    Address = IXGBE_GRC_BY_MAC (IXGBE_HW_PTR (Handle));
#endif /* !WOL_HAF */
  }

#ifndef WOL_HAF
  ReadWord = IXGBE_READ_REG (&Handle->NicInfo.Hw, Address);
#else /* WOL_HAF */
  if (NAL_SUCCESS != NalReadMacRegister32 (Handle, Address, &ReadWord)) {
    return WOL_ERROR;
  }
#endif /* !WOL_HAF */

  /* If set to 10b, APM Wakeup is enabled */
  IsCurrentlySet = (ReadWord & IXGBE_GRC_APME) != 0;

  if (IsCurrentlySet != Enable) {
    ReadWord ^= IXGBE_GRC_APME;
#ifndef WOL_HAF
    IXGBE_WRITE_REG (&Handle->NicInfo.Hw, Address, ReadWord);
#else /* WOL_HAF */
    if (NAL_SUCCESS != NalWriteMacRegister32 (Handle, Address, ReadWord)) {
      return WOL_ERROR;
    }
#endif /* !WOL_HAF */
  }

  return WOL_SUCCESS;
}

